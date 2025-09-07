#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <ctime>

// Recast & Detour includes
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "ChunkyTriMesh.h"

// Sample area and poly types
enum SamplePolyAreas
{
    SAMPLE_POLYAREA_GROUND,
    SAMPLE_POLYAREA_WATER,
    SAMPLE_POLYAREA_ROAD,
    SAMPLE_POLYAREA_DOOR,
    SAMPLE_POLYAREA_GRASS,
    SAMPLE_POLYAREA_JUMP,
};

enum SamplePolyFlags
{
    SAMPLE_POLYFLAGS_WALK       = 0x01,
    SAMPLE_POLYFLAGS_SWIM       = 0x02,
    SAMPLE_POLYFLAGS_DOOR       = 0x04,
    SAMPLE_POLYFLAGS_JUMP       = 0x08,
    SAMPLE_POLYFLAGS_DISABLED   = 0x10,
    SAMPLE_POLYFLAGS_ALL        = 0xffff
};

enum SamplePartitionType
{
    SAMPLE_PARTITION_WATERSHED,
    SAMPLE_PARTITION_MONOTONE,
    SAMPLE_PARTITION_LAYERS,
};

struct CheckSettings
{
    float cellSize = 0.26f;
    float cellHeight = 0.20f;
    float agentHeight = 1.0f;
    float agentRadius = 0.1f;
    float agentMaxClimb = 0.6f;
    float agentMaxSlope = 60.0f;
    float regionMinSize = 8.0f;
    float regionMergeSize = 20.0f;
    float edgeMaxLen = 12.0f;
    float edgeMaxError = 1.3f;
    int vertsPerPoly = 6;
    float detailSampleDist = 6.0f;
    float detailSampleMaxError = 1.0f;
    float tileSize = 256.0f;
    bool filterLowHangingObstacles = true;
    bool filterLedgeSpans = true;
    bool filterWalkableLowHeightSpans = true;
    SamplePartitionType partitionType = SAMPLE_PARTITION_WATERSHED;
};

// Enhanced analysis structures
struct TileAnalysis
{
    int x, y;
    int tileIndex;
    int inputTriangles;
    int estimatedPolygons;
    int estimatedVertices;
    float worldBounds[6];
    float tileWorldSize[2];
    float tileCenterWorld[3];
    bool hasGeometry;
    bool exceedsPolyLimit;
    bool exceedsVertLimit;
    float triangleDensity;
    float polygonDensity;
    std::string warningMessages;

    TileAnalysis() : x(0), y(0), tileIndex(0), inputTriangles(0), estimatedPolygons(0),
                    estimatedVertices(0), hasGeometry(false), exceedsPolyLimit(false), 
                    exceedsVertLimit(false), triangleDensity(0.0f), polygonDensity(0.0f)
    {
        memset(worldBounds, 0, sizeof(worldBounds));
        memset(tileWorldSize, 0, sizeof(tileWorldSize));
        memset(tileCenterWorld, 0, sizeof(tileCenterWorld));
    }
};

// Simple mesh loader
class SimpleMeshLoader
{
private:
    std::vector<float> m_verts;
    std::vector<int> m_tris;
    std::vector<float> m_normals;
    float m_scale;
    std::string m_lastLoadedFilename;

    void addVertex(float x, float y, float z)
    {
        m_verts.push_back(x * m_scale);
        m_verts.push_back(y * m_scale);
        m_verts.push_back(z * m_scale);
    }

    void addTriangle(int a, int b, int c)
    {
        m_tris.push_back(a);
        m_tris.push_back(b);
        m_tris.push_back(c);
    }

    static char* parseRow(char* buf, char* bufEnd, char* row, int len)
    {
        bool start = true;
        bool done = false;
        int n = 0;
        while (!done && buf < bufEnd)
        {
            char c = *buf;
            buf++;
            switch (c)
            {
                case '\\': break;
                case '\n':
                    if (start) break;
                    done = true;
                    break;
                case '\r': break;
                case '\t':
                case ' ':
                    if (start) break;
                default:
                    start = false;
                    row[n++] = c;
                    if (n >= len-1) done = true;
                    break;
            }
        }
        row[n] = '\0';
        return buf;
    }

    static int parseFace(char* row, int* data, int n, int vcnt)
    {
        int j = 0;
        while (*row != '\0')
        {
            while (*row != '\0' && (*row == ' ' || *row == '\t')) row++;
            char* s = row;
            while (*row != '\0' && *row != ' ' && *row != '\t')
            {
                if (*row == '/') *row = '\0';
                row++;
            }
            if (*s == '\0') continue;
            int vi = atoi(s);
            data[j++] = vi < 0 ? vi + vcnt : vi - 1;
            if (j >= n) return j;
        }
        return j;
    }

    void calculateNormals()
    {
        const int triCount = (int)m_tris.size() / 3;
        m_normals.resize(triCount * 3);

        for (int i = 0; i < triCount; ++i)
        {
            const float* v0 = &m_verts[(size_t)m_tris[(size_t)i*3+0] * 3];
            const float* v1 = &m_verts[(size_t)m_tris[(size_t)i*3+1] * 3];
            const float* v2 = &m_verts[(size_t)m_tris[(size_t)i*3+2] * 3];

            float e0[3], e1[3];
            for (int j = 0; j < 3; ++j)
            {
                e0[j] = v1[j] - v0[j];
                e1[j] = v2[j] - v0[j];
            }

            float* n = &m_normals[(size_t)i * 3];
            n[0] = e0[1]*e1[2] - e0[2]*e1[1];
            n[1] = e0[2]*e1[0] - e0[0]*e1[2];
            n[2] = e0[0]*e1[1] - e0[1]*e1[0];

            float d = sqrtf(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
            if (d > 0)
            {
                d = 1.0f/d;
                n[0] *= d; n[1] *= d; n[2] *= d;
            }
        }
    }

public:
    SimpleMeshLoader() : m_scale(1.0f) {}

    bool load(const std::string& filename)
    {
        std::cout << "Loading OBJ: " << filename << std::endl;
        m_lastLoadedFilename = filename;

        FILE* fp = fopen(filename.c_str(), "rb");
        if (!fp)
        {
            std::cerr << "Failed to open: " << filename << std::endl;
            return false;
        }

        fseek(fp, 0, SEEK_END);
        long bufSize = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        if (bufSize <= 0)
        {
            fclose(fp);
            std::cerr << "Invalid file size" << std::endl;
            return false;
        }

        std::vector<char> buf(bufSize);
        size_t readLen = fread(buf.data(), bufSize, 1, fp);
        fclose(fp);

        if (readLen != 1)
        {
            std::cerr << "Failed to read file" << std::endl;
            return false;
        }

        m_verts.reserve(1000000 * 3);
        m_tris.reserve(2000000 * 3);

        char* src = buf.data();
        char* srcEnd = buf.data() + bufSize;
        char row[512];
        int face[32];
        float x, y, z;
        int nv;
        int vertCount = 0;
        int triCount = 0;

        while (src < srcEnd)
        {
            row[0] = '\0';
            src = parseRow(src, srcEnd, row, sizeof(row));

            if (row[0] == '#') continue;

            if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
            {
                if (sscanf(row + 1, "%f %f %f", &x, &y, &z) == 3)
                {
                    addVertex(x, y, z);
                    vertCount++;

                    if (vertCount % 100000 == 0)
                        std::cout << "  Loaded " << vertCount << " vertices..." << std::endl;
                }
            }
            else if (row[0] == 'f')
            {
                nv = parseFace(row + 1, face, 32, getVertCount());
                for (int i = 2; i < nv; ++i)
                {
                    const int a = face[0];
                    const int b = face[i-1];
                    const int c = face[i];
                    if (a >= 0 && a < getVertCount() &&
                        b >= 0 && b < getVertCount() &&
                        c >= 0 && c < getVertCount())
                    {
                        addTriangle(a, b, c);
                        triCount++;

                        if (triCount % 100000 == 0)
                            std::cout << "  Loaded " << triCount << " triangles..." << std::endl;
                    }
                }
            }
        }

        calculateNormals();

        std::cout << "Loaded: " << getVertCount() << " verts, " << getTriCount() << " tris" << std::endl;
        return !m_verts.empty() && !m_tris.empty();
    }

    const float* getVerts() const { return m_verts.empty() ? nullptr : m_verts.data(); }
    const int* getTris() const { return m_tris.empty() ? nullptr : m_tris.data(); }
    const float* getNormals() const { return m_normals.empty() ? nullptr : m_normals.data(); }
    int getVertCount() const { return (int)(m_verts.size() / 3); }
    int getTriCount() const { return (int)(m_tris.size() / 3); }
    std::string getLastLoadedFilename() const { return m_lastLoadedFilename; }

    void getBounds(float* bmin, float* bmax) const
    {
        if (m_verts.empty()) return;

        bmin[0] = bmin[1] = bmin[2] = FLT_MAX;
        bmax[0] = bmax[1] = bmax[2] = -FLT_MAX;

        for (size_t i = 0; i < m_verts.size(); i += 3)
        {
            bmin[0] = std::min(bmin[0], m_verts[i]);
            bmin[1] = std::min(bmin[1], m_verts[i+1]);
            bmin[2] = std::min(bmin[2], m_verts[i+2]);
            bmax[0] = std::max(bmax[0], m_verts[i]);
            bmax[1] = std::max(bmax[1], m_verts[i+1]);
            bmax[2] = std::max(bmax[2], m_verts[i+2]);
        }
    }
};

class NavMeshChecker
{
private:
    CheckSettings m_cfg;
    SimpleMeshLoader m_mesh;
    rcChunkyTriMesh* m_chunkyMesh;
    rcContext* m_ctx;
    float m_meshBMin[3], m_meshBMax[3];
    int m_maxTiles, m_maxPolysPerTile;

public:
    NavMeshChecker() : m_chunkyMesh(nullptr)
    {
        m_ctx = new rcContext();
    }

    ~NavMeshChecker()
    {
        cleanup();
        delete m_ctx;
    }

    void cleanup()
    {
        delete m_chunkyMesh;
        m_chunkyMesh = nullptr;
    }

    bool loadMesh(const std::string& filename)
    {
        if (!m_mesh.load(filename))
            return false;

        m_mesh.getBounds(m_meshBMin, m_meshBMax);

        std::cout << "Mesh bounds: (" << m_meshBMin[0] << ", " << m_meshBMin[1] << ", " << m_meshBMin[2] << ") - "
                  << "(" << m_meshBMax[0] << ", " << m_meshBMax[1] << ", " << m_meshBMax[2] << ")" << std::endl;

        delete m_chunkyMesh;
        m_chunkyMesh = new rcChunkyTriMesh;
        if (!m_chunkyMesh)
        {
            std::cerr << "Failed to allocate chunky tri mesh" << std::endl;
            return false;
        }
        if (!rcCreateChunkyTriMesh(m_mesh.getVerts(), m_mesh.getTris(), m_mesh.getTriCount(), 256, m_chunkyMesh))
        {
            std::cerr << "Failed to create chunky tri mesh" << std::endl;
            return false;
        }

        std::cout << "Created chunky mesh" << std::endl;
        return true;
    }

    bool analyzeNavMeshRequirements()
    {
        if (!m_mesh.getVerts() || !m_mesh.getTris())
        {
            std::cerr << "No mesh data loaded" << std::endl;
            return false;
        }

        int gw = 0, gh = 0;
        rcCalcGridSize(m_meshBMin, m_meshBMax, m_cfg.cellSize, &gw, &gh);
        const int ts = (int)m_cfg.tileSize;
        const int tw = (gw + ts-1) / ts;
        const int th = (gh + ts-1) / ts;

        m_maxTiles = 16384;
        m_maxPolysPerTile = 16384;

        std::cout << "\n=== NAVMESH ANALYSIS REPORT ===" << std::endl;
        std::cout << "Analyzing mesh requirements for NavMesh generation..." << std::endl;
        
        std::cout << "\nMesh Properties:" << std::endl;
        std::cout << "  Vertices: " << m_mesh.getVertCount() << std::endl;
        std::cout << "  Triangles: " << m_mesh.getTriCount() << std::endl;
        std::cout << "  Bounds: (" << std::fixed << std::setprecision(2)
                  << m_meshBMin[0] << ", " << m_meshBMin[1] << ", " << m_meshBMin[2] << ") to "
                  << "(" << m_meshBMax[0] << ", " << m_meshBMax[1] << ", " << m_meshBMax[2] << ")" << std::endl;
        
        float worldSizeX = m_meshBMax[0] - m_meshBMin[0];
        float worldSizeZ = m_meshBMax[2] - m_meshBMin[2];
        float worldSizeY = m_meshBMax[1] - m_meshBMin[1];
        
        std::cout << "  World size: " << worldSizeX << " x " << worldSizeY << " x " << worldSizeZ << " units" << std::endl;

        std::cout << "\nNavMesh Configuration:" << std::endl;
        std::cout << "  Cell size: " << m_cfg.cellSize << " units" << std::endl;
        std::cout << "  Cell height: " << m_cfg.cellHeight << " units" << std::endl;
        std::cout << "  Agent radius: " << m_cfg.agentRadius << " units" << std::endl;
        std::cout << "  Agent height: " << m_cfg.agentHeight << " units" << std::endl;
        std::cout << "  Tile size: " << m_cfg.tileSize << " cells" << std::endl;
        
        const float tcs = m_cfg.tileSize * m_cfg.cellSize;
        std::cout << "  Tile world size: " << tcs << " x " << tcs << " units" << std::endl;
        std::cout << "  Grid size: " << gw << " x " << gh << " cells" << std::endl;
        std::cout << "  Tile grid: " << tw << " x " << th << " (" << (tw*th) << " total tiles)" << std::endl;
        std::cout << "  Max tiles: " << m_maxTiles << std::endl;
        std::cout << "  Max polys per tile: " << m_maxPolysPerTile << std::endl;
        
        // Display dtPolyRef limits with correct 32-bit defaults and 64-bit fixed limits
        std::cout << "\nDetour NavMesh Limits:" << std::endl;
        std::cout << "  32-bit dtPolyRef mode (configurable bit allocation):" << std::endl;
        std::cout << "    Default: 1 salt + 14 tile + 9 poly = 24 bits (8 unused)" << std::endl;
        std::cout << "    Max tiles: 2^14 = 16,384 tiles" << std::endl;
        std::cout << "    Max polys per tile: 2^9 = 512 polygons" << std::endl;
        std::cout << "    High-Density: 1 salt + 14 tile + 16 poly = 31 bits (1 unused)" << std::endl;
        std::cout << "    Max tiles: 2^14 = 16,384 tiles" << std::endl;
        std::cout << "    Max polys per tile: 2^16 = 65,536 polygons" << std::endl;
        std::cout << "  64-bit dtPolyRef mode (fixed allocation):" << std::endl;
        std::cout << "    Fixed configuration: 16 salt + 28 tile + 20 poly = 64 bits" << std::endl;
        std::cout << "    Max tiles: 2^28 = 268,435,456 tiles" << std::endl;
        std::cout << "    Max polys per tile: 2^20 = 1,048,576 polygons" << std::endl;
        std::cout << "    Salt values: 2^16 = 65,536 (0-65535)" << std::endl;

        return analyzeTilesDetailed(tw, th, tcs);
    }

    bool analyzeTilesDetailed(int tw, int th, const float tcs)
    {
        std::cout << "\n=== DETAILED TILE ANALYSIS ===" << std::endl;
        std::cout << "Analyzing " << (tw * th) << " tiles for geometry distribution..." << std::endl;

        // Calculate tile count early for use throughout the function
        int actualTileCount = tw * th;
        
        // Use correct 32-bit defaults, high-density, and 64-bit fixed limits
        const int maxPolysPerTile32Bit = 512;       // 2^9 (default 32-bit)
        const int maxTilesTotal32Bit = 16384;       // 2^14 (default 32-bit)
        const int maxPolysPerTile32BitHD = 65536;   // 2^16 (high-density 32-bit)
        const int maxTilesTotal32BitHD = 16384;     // 2^14 (same tile limit)
        const int maxPolysPerTile64Bit = 1048576;  // 2^20 (fixed 64-bit)
        
        std::cout << "Using 32-bit default limits: " << maxPolysPerTile32Bit << " polygons per tile, " << maxTilesTotal32Bit << " max tiles" << std::endl;
        std::cout << "Using 64-bit fixed limits: " << maxPolysPerTile64Bit << " polygons per tile" << std::endl;

        // Create timestamped report filename
        auto now = std::chrono::system_clock::now();
        time_t time_t_now = std::chrono::system_clock::to_time_t(now);
        struct tm* timeinfo = localtime(&time_t_now);
        char timestamp[32];
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", timeinfo);
        std::string reportFilename = "navmesh_analysis_" + std::string(timestamp) + ".txt";
        
        std::ofstream analysisFile(reportFilename);
        if (!analysisFile.is_open())
        {
            std::cerr << "WARNING: Could not create analysis report file: " << reportFilename << std::endl;
        }

        if (analysisFile.is_open())
        {
            analysisFile << "=============================================" << std::endl;
            analysisFile << "COMPREHENSIVE NAVMESH TILE ANALYSIS REPORT" << std::endl;
            analysisFile << "=============================================" << std::endl;
            analysisFile << "Generated: " << std::ctime(&time_t_now);
            analysisFile << "Mesh file: " << m_mesh.getLastLoadedFilename() << std::endl;
            analysisFile << "Mesh bounds: (" << std::fixed << std::setprecision(2)
                        << m_meshBMin[0] << ", " << m_meshBMin[1] << ", " << m_meshBMin[2] << ") to "
                        << "(" << m_meshBMax[0] << ", " << m_meshBMax[1] << ", " << m_meshBMax[2] << ")" << std::endl;
            analysisFile << std::endl;
            
            analysisFile << "CONFIGURATION:" << std::endl;
            analysisFile << "-------------" << std::endl;
            analysisFile << "Cell size: " << std::fixed << std::setprecision(2) << m_cfg.cellSize << " units" << std::endl;
            analysisFile << "Cell height: " << std::fixed << std::setprecision(2) << m_cfg.cellHeight << " units" << std::endl;
            analysisFile << "Agent radius: " << std::fixed << std::setprecision(2) << m_cfg.agentRadius << " units" << std::endl;
            analysisFile << "Agent height: " << std::fixed << std::setprecision(2) << m_cfg.agentHeight << " units" << std::endl;
            analysisFile << "Agent max climb: " << std::fixed << std::setprecision(2) << m_cfg.agentMaxClimb << " units" << std::endl;
            analysisFile << "Agent max slope: " << std::fixed << std::setprecision(2) << m_cfg.agentMaxSlope << " degrees" << std::endl;
            analysisFile << "Tile size: " << std::fixed << std::setprecision(2) << m_cfg.tileSize << " cells (" << tcs << " world units)" << std::endl;
            analysisFile << "Max polygons per tile: " << maxPolysPerTile32Bit << std::endl;
            analysisFile << "Max vertices per tile: 65535" << std::endl;
            analysisFile << "Tile grid: " << tw << " x " << th << " = " << (tw*th) << " total tiles" << std::endl;
            analysisFile << "Region min size: " << std::fixed << std::setprecision(2) << m_cfg.regionMinSize << std::endl;
            analysisFile << "Region merge size: " << std::fixed << std::setprecision(2) << m_cfg.regionMergeSize << std::endl;
            analysisFile << "Edge max length: " << std::fixed << std::setprecision(2) << m_cfg.edgeMaxLen << std::endl;
            analysisFile << "Edge max error: " << std::fixed << std::setprecision(2) << m_cfg.edgeMaxError << std::endl;
            analysisFile << std::endl;
        }

        std::vector<TileAnalysis> tileAnalyses;
        int tilesWithGeometry = 0;
        int tilesExceeding32BitPolyLimit = 0;
        int tilesExceeding32BitHDPolyLimit = 0;
        int tilesExceeding64BitPolyLimit = 0;
        int tilesExceedingVertLimit = 0;
        int maxPolysFound = 0;
        int maxVertsFound = 0;
        long long totalTriangles = 0;
        
        std::vector<int> triangleDistribution;
        triangleDistribution.reserve(tw * th);

        auto analysisStartTime = std::chrono::high_resolution_clock::now();

        for (int y = 0; y < th; ++y)
        {
            for (int x = 0; x < tw; ++x)
            {
                int currentTile = y * tw + x;
                if (currentTile % 500 == 0 || currentTile == tw*th - 1)
                {
                    int progress = ((currentTile + 1) * 100) / (tw*th);
                    std::cout << "\rAnalyzing tiles... " << progress << "% (" << (currentTile + 1) << "/" << (tw*th) << ")" << std::flush;
                }

                TileAnalysis analysis;
                analysis.x = x;
                analysis.y = y;
                analysis.tileIndex = currentTile;

                float tileBmin[3], tileBmax[3];
                tileBmin[0] = m_meshBMin[0] + x * tcs;
                tileBmin[1] = m_meshBMin[1];
                tileBmin[2] = m_meshBMin[2] + y * tcs;
                tileBmax[0] = m_meshBMin[0] + (x+1) * tcs;
                tileBmax[1] = m_meshBMax[1];
                tileBmax[2] = m_meshBMin[2] + (y+1) * tcs;
                
                // Store tile bounds for reporting
                analysis.worldBounds[0] = tileBmin[0];
                analysis.worldBounds[1] = tileBmin[1]; 
                analysis.worldBounds[2] = tileBmin[2];
                analysis.worldBounds[3] = tileBmax[0];
                analysis.worldBounds[4] = tileBmax[1];
                analysis.worldBounds[5] = tileBmax[2];
                analysis.tileWorldSize[0] = tcs;
                analysis.tileWorldSize[1] = tcs;
                analysis.tileCenterWorld[0] = (tileBmin[0] + tileBmax[0]) * 0.5f;
                analysis.tileCenterWorld[1] = (tileBmin[1] + tileBmax[1]) * 0.5f;
                analysis.tileCenterWorld[2] = (tileBmin[2] + tileBmax[2]) * 0.5f;
                
                const float borderSize = (float)(int)ceilf(m_cfg.agentRadius / m_cfg.cellSize) + 3;
                float expandedBmin[3] = {tileBmin[0] - borderSize * m_cfg.cellSize, tileBmin[1], tileBmin[2] - borderSize * m_cfg.cellSize};
                float expandedBmax[3] = {tileBmax[0] + borderSize * m_cfg.cellSize, tileBmax[1], tileBmax[2] + borderSize * m_cfg.cellSize};

                float tbmin[2] = {expandedBmin[0], expandedBmin[2]};
                float tbmax[2] = {expandedBmax[0], expandedBmax[2]};

                int chunkIds[512];
                int nchunks = rcGetChunksOverlappingRect(m_chunkyMesh, tbmin, tbmax, chunkIds, 512);

                analysis.inputTriangles = 0;
                if (nchunks > 0)
                {
                    for (int i = 0; i < nchunks; ++i)
                    {
                        int chunkId = chunkIds[i];
                        const rcChunkyTriMeshNode& node = m_chunkyMesh->nodes[chunkId];
                        analysis.inputTriangles += node.n;
                    }
                }
                
                totalTriangles += analysis.inputTriangles;
                triangleDistribution.push_back(analysis.inputTriangles);
                analysis.hasGeometry = (analysis.inputTriangles > 0);

                if (analysis.hasGeometry)
                {
                    tilesWithGeometry++;
                    // Conservative estimate: 1 polygon per 2 triangles
                    analysis.estimatedPolygons = std::max(1, analysis.inputTriangles / 2);
                    analysis.estimatedVertices = analysis.estimatedPolygons * 4;

                    // Check against all three configurations
                    bool exceeds32BitPolyLimit = (analysis.estimatedPolygons > maxPolysPerTile32Bit);
                    bool exceeds32BitHDPolyLimit = (analysis.estimatedPolygons > maxPolysPerTile32BitHD);
                    bool exceeds64BitPolyLimit = (analysis.estimatedPolygons > maxPolysPerTile64Bit);
                    bool exceedsVertLimit = (analysis.estimatedVertices > 65535);

                    if (exceedsVertLimit)
                    {
                        tilesExceedingVertLimit++;
                        analysis.warningMessages += "VERT_LIMIT ";
                    }

                    if (exceeds32BitPolyLimit) 
                    {
                        tilesExceeding32BitPolyLimit++;
                        analysis.warningMessages += "32BIT_POLY_LIMIT ";
                    }
                    if (exceeds32BitHDPolyLimit)
                    {
                        tilesExceeding32BitHDPolyLimit++;
                        analysis.warningMessages += "32BIT_HD_LIMIT ";
                    }
                    if (exceeds64BitPolyLimit)
                    {
                        tilesExceeding64BitPolyLimit++;
                        analysis.warningMessages += "64BIT_POLY_LIMIT ";
                    }
                    
                    // Calculate density
                    float tileArea = tcs * tcs;
                    analysis.triangleDensity = analysis.inputTriangles / tileArea;
                    analysis.polygonDensity = analysis.estimatedPolygons / tileArea;
                }

                maxPolysFound = std::max(maxPolysFound, analysis.estimatedPolygons);
                maxVertsFound = std::max(maxVertsFound, analysis.estimatedVertices);
                tileAnalyses.push_back(analysis);
            }
        }
        
        auto analysisEndTime = std::chrono::high_resolution_clock::now();
        float analysisTimeSecs = std::chrono::duration<float>(analysisEndTime - analysisStartTime).count();

        std::cout << std::endl;

        // Calculate statistics
        std::sort(triangleDistribution.begin(), triangleDistribution.end());
        int medianTriangles = triangleDistribution[triangleDistribution.size() / 2];
        double avgTriangles = (double)totalTriangles / triangleDistribution.size();
        
        // Find percentiles
        int p95Index = (int)(triangleDistribution.size() * 0.95);
        int p99Index = (int)(triangleDistribution.size() * 0.99);
        int p95Triangles = triangleDistribution[p95Index];
        int p99Triangles = triangleDistribution[p99Index];

        std::cout << "\n=== ANALYSIS RESULTS ===" << std::endl;
        std::cout << "Total tiles analyzed: " << tileAnalyses.size() << std::endl;
        std::cout << "Tiles with geometry: " << tilesWithGeometry << " (" << std::fixed << std::setprecision(1) 
                  << (100.0 * tilesWithGeometry / tileAnalyses.size()) << "%)" << std::endl;
        std::cout << "Empty tiles: " << (tileAnalyses.size() - tilesWithGeometry) << std::endl;
        
        std::cout << "\nTriangle Distribution:" << std::endl;
        std::cout << "  Total triangles: " << totalTriangles << std::endl;
        std::cout << "  Average per tile: " << std::fixed << std::setprecision(0) << avgTriangles << std::endl;
        std::cout << "  Median per tile: " << medianTriangles << std::endl;
        std::cout << "  95th percentile: " << p95Triangles << std::endl;
        std::cout << "  99th percentile: " << p99Triangles << std::endl;
        std::cout << "  Maximum per tile: " << *std::max_element(triangleDistribution.begin(), triangleDistribution.end()) << std::endl;

        std::cout << "\nEstimated NavMesh Requirements:" << std::endl;
        std::cout << "  Max estimated polygons per tile: " << maxPolysFound << std::endl;
        std::cout << "  Max estimated vertices per tile: " << maxVertsFound << std::endl;
        
        std::cout << "\nLimit Analysis:" << std::endl;
        std::cout << "  32-bit dtPolyRef default (512 polys/tile):" << std::endl;
        if (tilesExceeding32BitPolyLimit > 0)
        {
            std::cout << "    ⚠️  " << tilesExceeding32BitPolyLimit << " tiles exceed 32-bit polygon limit (" << maxPolysPerTile32Bit << ")" << std::endl;
        }
        else
        {
            std::cout << "    ✅ All tiles within 32-bit polygon limit (" << maxPolysPerTile32Bit << ")" << std::endl;
        }
        
        // Check if tile count exceeds 32-bit tile limit
        if (actualTileCount > maxTilesTotal32Bit)
        {
            std::cout << "    ⚠️  Total tiles (" << actualTileCount << ") exceed 32-bit tile limit (" << maxTilesTotal32Bit << ")" << std::endl;
        }
        else
        {
            std::cout << "    ✅ Total tiles (" << actualTileCount << ") within 32-bit tile limit (" << maxTilesTotal32Bit << ")" << std::endl;
        }
        
        std::cout << "  32-bit dtPolyRef high-density (65,536 polys/tile):" << std::endl;
        if (tilesExceeding32BitHDPolyLimit > 0)
        {
            std::cout << "    ⚠️  " << tilesExceeding32BitHDPolyLimit << " tiles exceed 32-bit HD polygon limit (65,536)" << std::endl;
        }
        else
        {
            std::cout << "    ✅ All tiles within 32-bit HD polygon limit (65,536)" << std::endl;
        }
        
        std::cout << "  64-bit dtPolyRef compatibility (fixed: " << maxPolysPerTile64Bit << " polys/tile):" << std::endl;
        if (tilesExceeding64BitPolyLimit > 0)
        {
            std::cout << "    ⚠️  " << tilesExceeding64BitPolyLimit << " tiles exceed 64-bit polygon limit (" << maxPolysPerTile64Bit << ")" << std::endl;
        }
        else
        {
            std::cout << "    ✅ All tiles within 64-bit polygon limit (" << maxPolysPerTile64Bit << ")" << std::endl;
        }
        
        std::cout << "  Vertex limit (both modes):" << std::endl;
        if (tilesExceedingVertLimit > 0)
        {
            std::cout << "    ⚠️  " << tilesExceedingVertLimit << " tiles exceed vertex limit (65,535)" << std::endl;
        }
        else
        {
            std::cout << "    ✅ All tiles within vertex limit (65,535)" << std::endl;
        }
        
        if (tilesExceeding32BitPolyLimit > 0 || tilesExceeding64BitPolyLimit > 0 || tilesExceedingVertLimit > 0 || actualTileCount > maxTilesTotal32Bit)
        {
            std::cout << "\nRecommendations:" << std::endl;
            if (tilesExceeding32BitPolyLimit > 0)
            {
                std::cout << "  - For 32-bit mode: Increase tile size or reduce mesh detail" << std::endl;
                std::cout << "  - Alternative: Adjust bit allocation (e.g., 12 poly bits = 4,096 polys/tile)" << std::endl;
            }
            if (actualTileCount > maxTilesTotal32Bit)
            {
                std::cout << "  - For 32-bit mode: Increase tile size to reduce total tile count" << std::endl;
                std::cout << "  - Alternative: Adjust bit allocation (e.g., 18 tile bits = 262,144 max tiles)" << std::endl;
            }
            if (tilesExceeding64BitPolyLimit > 0)
                std::cout << "  - Even 64-bit mode exceeded: Significantly increase tile size or simplify mesh" << std::endl;
            if (tilesExceedingVertLimit > 0)
                std::cout << "  - Vertex limit exceeded: Consider mesh simplification for extremely dense areas" << std::endl;
                
            std::cout << "\nCommon 32-bit configurations:" << std::endl;
            std::cout << "  - Large worlds: 18 tile + 6 poly bits = 262K tiles, 64 polys/tile" << std::endl;
            std::cout << "  - Detailed areas: 11 tile + 12 poly bits = 2K tiles, 4K polys/tile" << std::endl;
        }
        else
        {
            std::cout << "\n✅ No issues detected - mesh should build successfully in both modes!" << std::endl;
        }

        std::cout << "\nAnalysis completed in " << std::fixed << std::setprecision(2) << analysisTimeSecs << " seconds" << std::endl;

        // Write detailed report to file
        if (analysisFile.is_open())
        {
            analysisFile << "SUMMARY STATISTICS:" << std::endl;
            analysisFile << "  Total tiles: " << tileAnalyses.size() << std::endl;
            analysisFile << "  Tiles with geometry: " << tilesWithGeometry << std::endl;
            analysisFile << "  Average triangles per tile: " << avgTriangles << std::endl;
            analysisFile << "  Median triangles per tile: " << medianTriangles << std::endl;
            analysisFile << "  Max triangles per tile: " << *std::max_element(triangleDistribution.begin(), triangleDistribution.end()) << std::endl;
            analysisFile << std::endl;
            
            analysisFile << "DETOUR NAVMESH LIMITS & COMPATIBILITY:" << std::endl;
            analysisFile << "  32-bit dtPolyRef (default configuration: 1+14+9 bits):" << std::endl;
            analysisFile << "    Max tiles: " << maxTilesTotal32Bit << ", Max polys/tile: " << maxPolysPerTile32Bit << std::endl;
            analysisFile << "    Actual tile count: " << actualTileCount << std::endl;
            analysisFile << "    Tiles exceeding 32-bit poly limit: " << tilesExceeding32BitPolyLimit << std::endl;
            analysisFile << "    Tile count within 32-bit limit: " << (actualTileCount <= maxTilesTotal32Bit ? "YES" : "NO") << std::endl;
            
            // ADD THIS NEW SECTION FOR HIGH-DENSITY:
            analysisFile << "  32-bit dtPolyRef (high-density configuration: 1+14+16 bits):" << std::endl;
            analysisFile << "    Max tiles: " << maxTilesTotal32BitHD << ", Max polys/tile: " << maxPolysPerTile32BitHD << std::endl;
            analysisFile << "    Actual tile count: " << actualTileCount << std::endl;
            analysisFile << "    Tiles exceeding 32-bit HD poly limit: " << tilesExceeding32BitHDPolyLimit << std::endl;
            analysisFile << "    Tile count within 32-bit HD limit: " << (actualTileCount <= maxTilesTotal32BitHD ? "YES" : "NO") << std::endl;
            
            analysisFile << "  64-bit dtPolyRef (fixed configuration: 16+28+20 bits):" << std::endl;
            analysisFile << "    Max tiles: 268,435,456, Max polys/tile: " << maxPolysPerTile64Bit << std::endl;
            analysisFile << "    Tiles exceeding 64-bit poly limit: " << tilesExceeding64BitPolyLimit << std::endl;
            analysisFile << "  Vertex limit (both modes): 65,535" << std::endl;
            analysisFile << "    Tiles exceeding vertex limit: " << tilesExceedingVertLimit << std::endl;
            analysisFile << std::endl;
            
            analysisFile << "COMPATIBILITY SUMMARY:" << std::endl;
            analysisFile << "  32-bit mode (default) compatibility: " << (tilesExceeding32BitPolyLimit == 0 && actualTileCount <= maxTilesTotal32Bit ? "COMPATIBLE" : "INCOMPATIBLE") << std::endl;
            // ADD THIS LINE FOR HIGH-DENSITY COMPATIBILITY:
            analysisFile << "  32-bit mode (high-density) compatibility: " << (tilesExceeding32BitHDPolyLimit == 0 && actualTileCount <= maxTilesTotal32BitHD ? "COMPATIBLE" : "INCOMPATIBLE") << std::endl;
            analysisFile << "  64-bit mode compatibility: " << (tilesExceeding64BitPolyLimit == 0 ? "COMPATIBLE" : "INCOMPATIBLE") << std::endl;
            analysisFile << "  Vertex compatibility: " << (tilesExceedingVertLimit == 0 ? "COMPATIBLE" : "INCOMPATIBLE") << std::endl;
            analysisFile << std::endl;
            
            analysisFile << "PER-TILE DETAILED ANALYSIS:" << std::endl;
            analysisFile << "---------------------------" << std::endl;
            analysisFile << std::left 
                        << std::setw(10) << "Tile(X,Y)"
                        << std::setw(8) << "Index"
                        << std::setw(10) << "InputTris"
                        << std::setw(10) << "EstPolys"
                        << std::setw(10) << "EstVerts"
                        << std::setw(12) << "TriDensity"
                        << std::setw(12) << "PolyDensity"
                        << std::setw(35) << "WorldBounds"
                        << std::setw(20) << "Center"
                        << "Warnings" << std::endl;
            analysisFile << std::string(130, '-') << std::endl;
            
            for (const auto& tile : tileAnalyses)
            {
                if (tile.hasGeometry)
                {
                    std::string boundsStr = "(" + std::to_string((int)tile.worldBounds[0]) + "," + 
                                          std::to_string((int)tile.worldBounds[2]) + ")-(" +
                                          std::to_string((int)tile.worldBounds[3]) + "," +
                                          std::to_string((int)tile.worldBounds[5]) + ")";
                    std::string centerStr = "(" + std::to_string((int)tile.tileCenterWorld[0]) + "," + 
                                          std::to_string((int)tile.tileCenterWorld[1]) + "," +
                                          std::to_string((int)tile.tileCenterWorld[2]) + ")";
                    
                    analysisFile << std::left
                                << std::setw(10) << ("(" + std::to_string(tile.x) + "," + std::to_string(tile.y) + ")")
                                << std::setw(8) << tile.tileIndex
                                << std::setw(10) << tile.inputTriangles
                                << std::setw(10) << tile.estimatedPolygons
                                << std::setw(10) << tile.estimatedVertices
                                << std::setw(12) << std::fixed << std::setprecision(2) << tile.triangleDensity
                                << std::setw(12) << std::fixed << std::setprecision(2) << tile.polygonDensity
                                << std::setw(35) << boundsStr
                                << std::setw(20) << centerStr
                                << tile.warningMessages << std::endl;
                }
            }
            
            // Add section for tiles exceeding vertex limits
            if (tilesExceedingVertLimit > 0)
            {
                analysisFile << std::endl;
                analysisFile << "TILES EXCEEDING VERTEX LIMIT (65,535):" << std::endl;
                analysisFile << "---------------------------------------" << std::endl;
                analysisFile << std::left 
                            << std::setw(10) << "Tile(X,Y)"
                            << std::setw(8) << "Index"
                            << std::setw(10) << "InputTris"
                            << std::setw(10) << "EstPolys"
                            << std::setw(10) << "EstVerts"
                            << std::setw(35) << "WorldBounds"
                            << std::setw(20) << "Center"
                            << "Warnings" << std::endl;
                analysisFile << std::string(120, '-') << std::endl;
                
                for (const auto& tile : tileAnalyses)
                {
                    if (tile.hasGeometry && tile.estimatedVertices > 65535)
                    {
                        std::string boundsStr = "(" + std::to_string((int)tile.worldBounds[0]) + "," + 
                                              std::to_string((int)tile.worldBounds[2]) + ")-(" +
                                              std::to_string((int)tile.worldBounds[3]) + "," +
                                              std::to_string((int)tile.worldBounds[5]) + ")";
                        std::string centerStr = "(" + std::to_string((int)tile.tileCenterWorld[0]) + "," + 
                                              std::to_string((int)tile.tileCenterWorld[1]) + "," +
                                              std::to_string((int)tile.tileCenterWorld[2]) + ")";
                        
                        analysisFile << std::left
                                    << std::setw(10) << ("(" + std::to_string(tile.x) + "," + std::to_string(tile.y) + ")")
                                    << std::setw(8) << tile.tileIndex
                                    << std::setw(10) << tile.inputTriangles
                                    << std::setw(10) << tile.estimatedPolygons
                                    << std::setw(10) << tile.estimatedVertices
                                    << std::setw(35) << boundsStr
                                    << std::setw(20) << centerStr
                                    << tile.warningMessages << std::endl;
                    }
                }
            }
            
            analysisFile.close();
            std::cout << "Detailed report saved to: " << reportFilename << std::endl;
        }

        return true;
    }

    void printConfiguration()
    {
        std::cout << "\n=== CURRENT NAVMESH SETTINGS ===" << std::endl;
        std::cout << "Cell size: " << m_cfg.cellSize << " units" << std::endl;
        std::cout << "Cell height: " << m_cfg.cellHeight << " units" << std::endl;
        std::cout << "Agent height: " << m_cfg.agentHeight << " units" << std::endl;
        std::cout << "Agent radius: " << m_cfg.agentRadius << " units" << std::endl;
        std::cout << "Agent max climb: " << m_cfg.agentMaxClimb << " units" << std::endl;
        std::cout << "Agent max slope: " << m_cfg.agentMaxSlope << " degrees" << std::endl;
        std::cout << "Tile size: " << m_cfg.tileSize << " cells" << std::endl;
        std::cout << "Region min size: " << m_cfg.regionMinSize << std::endl;
        std::cout << "Region merge size: " << m_cfg.regionMergeSize << std::endl;
        std::cout << "Edge max length: " << m_cfg.edgeMaxLen << std::endl;
        std::cout << "Edge max error: " << m_cfg.edgeMaxError << std::endl;
        std::cout << "Vertices per polygon: " << m_cfg.vertsPerPoly << std::endl;
        std::cout << "Detail sample distance: " << m_cfg.detailSampleDist << std::endl;
        std::cout << "Detail sample max error: " << m_cfg.detailSampleMaxError << std::endl;
    }

    // Configuration setters for easy modification
    void setCellSize(float size) { m_cfg.cellSize = size; }
    void setCellHeight(float height) { m_cfg.cellHeight = height; }
    void setAgentHeight(float height) { m_cfg.agentHeight = height; }
    void setAgentRadius(float radius) { m_cfg.agentRadius = radius; }
    void setTileSize(float size) { m_cfg.tileSize = size; }
    void setMaxPolysPerTile(int maxPolys) { m_maxPolysPerTile = maxPolys; }
};

int main(int argc, char* argv[])
{
    std::cout << "NavMesh Checker v1.0 - Mesh Analysis Tool" << std::endl;
    std::cout << "==========================================" << std::endl;

    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <input.obj> [options]" << std::endl;
        std::cout << "\nAnalyzes an OBJ mesh file for NavMesh generation requirements." << std::endl;
        std::cout << "Provides detailed tile analysis and recommendations." << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  --config     Show current NavMesh configuration" << std::endl;
        std::cout << "  --quick      Run quick analysis (less detailed)" << std::endl;
        std::cout << "\nExample:" << std::endl;
        std::cout << "  " << argv[0] << " large_world.obj" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    bool showConfig = false;
    bool quickMode = false;

    // Parse command line options
    for (int i = 2; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--config")
            showConfig = true;
        else if (arg == "--quick")
            quickMode = true;
    }

    std::cout << "\nInput file: " << inputFile << std::endl;

    NavMeshChecker checker;

    if (showConfig)
    {
        checker.printConfiguration();
        std::cout << "\nPress Enter to continue with analysis, or Ctrl+C to exit...";
        std::cin.get();
    }

    std::cout << "\n=== Phase 1: Loading Mesh ===" << std::endl;
    if (!checker.loadMesh(inputFile))
    {
        std::cerr << "\nERROR: Failed to load mesh: " << inputFile << std::endl;
        std::cerr << "Please check that the file exists and is a valid OBJ file." << std::endl;
        return 1;
    }

    std::cout << "\n=== Phase 2: Analyzing NavMesh Requirements ===" << std::endl;
    if (!checker.analyzeNavMeshRequirements())
    {
        std::cerr << "\nERROR: Failed to analyze mesh requirements" << std::endl;
        return 1;
    }

    std::cout << "\n=== ANALYSIS COMPLETE ===" << std::endl;
    std::cout << "NavMesh analysis completed successfully!" << std::endl;
    std::cout << "Check the generated report file for detailed tile information." << std::endl;
    
    std::cout << "\nRecommendations for NavMesh building:" << std::endl;
    std::cout << "1. Review any warning messages about tiles exceeding limits" << std::endl;
    std::cout << "2. Consider adjusting tile size or mesh detail if issues are found" << std::endl;
    std::cout << "3. Use the analysis data to estimate build time and memory requirements" << std::endl;

    return 0;
}