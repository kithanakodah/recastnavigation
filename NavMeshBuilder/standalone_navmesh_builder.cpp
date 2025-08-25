#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <numeric>  // Added for std::accumulate

// Recast & Detour includes
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"

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

struct BuildSettings
{
    float cellSize = 0.25f;
    float cellHeight = 0.20f;
    float agentHeight = 1.0f;
    float agentRadius = 0.25f;
    float agentMaxClimb = 0.6f;
    float agentMaxSlope = 60.0f;
    float regionMinSize = 8.0f;
    float regionMergeSize = 20.0f;
    float edgeMaxLen = 12.0f;
    float edgeMaxError = 1.3f;
    int vertsPerPoly = 6;
    float detailSampleDist = 6.0f;
    float detailSampleMaxError = 1.0f;
    float tileSize = 368.0f;
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
    
    // Geometry measurements
    int inputTriangles;
    int estimatedPolygons;
    int estimatedVertices;
    int actualPolygons;      // After build
    int actualVertices;      // After build
    
    // Spatial information
    float worldBounds[6];    // minX, minY, minZ, maxX, maxY, maxZ
    float tileWorldSize[2];  // width, height in world units
    float tileCenterWorld[3]; // center point in world coordinates
    
    // Analysis flags
    bool hasGeometry;
    bool exceedsPolyLimit;
    bool exceedsVertLimit;
    bool buildSuccessful;
    
    // Build timing and memory
    float buildTimeMs;
    int memoryUsedKB;
    
    // Quality metrics
    float triangleDensity;   // triangles per square unit
    float polygonDensity;    // estimated polygons per square unit
    
    // Error information
    std::string buildError;
    std::string warningMessages;
    
    TileAnalysis() : x(0), y(0), tileIndex(0), inputTriangles(0), estimatedPolygons(0), 
                    estimatedVertices(0), actualPolygons(0), actualVertices(0),
                    hasGeometry(false), exceedsPolyLimit(false), exceedsVertLimit(false),
                    buildSuccessful(false), buildTimeMs(0.0f), memoryUsedKB(0),
                    triangleDensity(0.0f), polygonDensity(0.0f) 
    {
        memset(worldBounds, 0, sizeof(worldBounds));
        memset(tileWorldSize, 0, sizeof(tileWorldSize));
        memset(tileCenterWorld, 0, sizeof(tileCenterWorld));
    }
};

struct TileData
{
    std::string buildError;
    std::string warningMessages;
    float buildTimeMs;
    int memoryUsedKB;
    int actualPolygons;
    int actualVertices;
    
    TileData() : buildTimeMs(0.0f), memoryUsedKB(0), actualPolygons(0), actualVertices(0) {}
};

// Simple chunky triangle mesh implementation
struct ChunkyTriMeshNode
{
    float bmin[2];
    float bmax[2];
    int i; // Index to first triangle
    int n; // Number of triangles
};

class SimpleChunkyTriMesh
{
public:
    std::vector<ChunkyTriMeshNode> nodes;
    std::vector<int> tris;
    int nnodes;
    int maxTrisPerChunk;
    
    SimpleChunkyTriMesh() : nnodes(0), maxTrisPerChunk(0) {}
    
    bool build(const float* verts, const int* triangles, int ntris, int trisPerChunk)
    {
        // Simple implementation: just store all triangles in one chunk for now
        // This is less efficient than the real chunky mesh but will work for our purposes
        maxTrisPerChunk = ntris;
        
        tris.resize(ntris * 3);
        memcpy(tris.data(), triangles, ntris * 3 * sizeof(int));
        
        nodes.resize(1);
        nodes[0].i = 0;
        nodes[0].n = ntris;
        
        // Calculate bounds
        nodes[0].bmin[0] = nodes[0].bmin[1] = FLT_MAX;
        nodes[0].bmax[0] = nodes[0].bmax[1] = -FLT_MAX;
        
        for (int i = 0; i < ntris; ++i)
        {
            const float* v0 = &verts[triangles[i*3+0] * 3];
            const float* v1 = &verts[triangles[i*3+1] * 3];
            const float* v2 = &verts[triangles[i*3+2] * 3];
            
            float minX = std::min({v0[0], v1[0], v2[0]});
            float maxX = std::max({v0[0], v1[0], v2[0]});
            float minZ = std::min({v0[2], v1[2], v2[2]});
            float maxZ = std::max({v0[2], v1[2], v2[2]});
            
            nodes[0].bmin[0] = std::min(nodes[0].bmin[0], minX);
            nodes[0].bmax[0] = std::max(nodes[0].bmax[0], maxX);
            nodes[0].bmin[1] = std::min(nodes[0].bmin[1], minZ);
            nodes[0].bmax[1] = std::max(nodes[0].bmax[1], maxZ);
        }
        
        nnodes = 1;
        return true;
    }
    
    std::vector<int> getChunksOverlappingRect(float bmin[2], float bmax[2])
    {
        std::vector<int> chunks;
        
        for (int i = 0; i < nnodes; ++i)
        {
            const ChunkyTriMeshNode& node = nodes[i];
            bool overlap = !(node.bmax[0] < bmin[0] || node.bmin[0] > bmax[0] ||
                           node.bmax[1] < bmin[1] || node.bmin[1] > bmax[1]);
            if (overlap)
            {
                chunks.push_back(i);
            }
        }
        
        return chunks;
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

// Utility functions
inline unsigned int nextPow2(unsigned int v)
{
    v--;
    v |= v >> 1;  v |= v >> 2;  v |= v >> 4;  v |= v >> 8;  v |= v >> 16;
    v++;
    return v;
}

inline unsigned int ilog2(unsigned int v)
{
    unsigned int r, shift;
    r = (v > 0xffff) << 4; v >>= r;
    shift = (v > 0xff) << 3; v >>= shift; r |= shift;
    shift = (v > 0xf) << 2; v >>= shift; r |= shift;
    shift = (v > 0x3) << 1; v >>= shift; r |= shift;
    r |= (v >> 1);
    return r;
}

class TiledNavMeshBuilder
{
private:
    BuildSettings m_cfg;
    SimpleMeshLoader m_mesh;
    SimpleChunkyTriMesh m_chunkyMesh;
    dtNavMesh* m_navMesh;
    rcContext* m_ctx;
    float m_meshBMin[3], m_meshBMax[3];
    int m_maxTiles, m_maxPolysPerTile;
    
public:
    TiledNavMeshBuilder() : m_navMesh(nullptr)
    {
        m_ctx = new rcContext();
    }
    
    ~TiledNavMeshBuilder()
    {
        cleanup();
        delete m_ctx;
    }
    
    void cleanup()
    {
        dtFreeNavMesh(m_navMesh);
        m_navMesh = nullptr;
    }
    
    bool loadMesh(const std::string& filename)
    {
        if (!m_mesh.load(filename))
            return false;
        
        m_mesh.getBounds(m_meshBMin, m_meshBMax);
        
        std::cout << "Mesh bounds: (" << m_meshBMin[0] << ", " << m_meshBMin[1] << ", " << m_meshBMin[2] << ") - "
                  << "(" << m_meshBMax[0] << ", " << m_meshBMax[1] << ", " << m_meshBMax[2] << ")" << std::endl;
        
        if (!m_chunkyMesh.build(m_mesh.getVerts(), m_mesh.getTris(), m_mesh.getTriCount(), 256))
        {
            std::cerr << "Failed to create chunky tri mesh" << std::endl;
            return false;
        }
        
        std::cout << "Created chunky mesh" << std::endl;
        return true;
    }
    
    // ENHANCED COMPREHENSIVE TILE ANALYSIS
    bool analyzeTilesComprehensive(int tw, int th, const float tcs)
    {
        std::cout << "\n=== COMPREHENSIVE TILE ANALYSIS ===" << std::endl;
        std::cout << "Analyzing " << (tw * th) << " tiles with detailed measurements..." << std::endl;
        std::cout << "This will measure geometry, estimate performance, and predict build issues." << std::endl;
        
        // Create comprehensive analysis report file
        std::string reportFilename = "comprehensive_tile_analysis.txt";
        std::ofstream analysisFile(reportFilename);
        if (!analysisFile.is_open())
        {
            std::cerr << "ERROR: Could not create analysis report file: " << reportFilename << std::endl;
            return false;
        }
        
        // Write detailed header
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        analysisFile << "=============================================" << std::endl;
        analysisFile << "COMPREHENSIVE NAVMESH TILE ANALYSIS REPORT" << std::endl;
        analysisFile << "=============================================" << std::endl;
        analysisFile << "Generated: " << std::ctime(&time_t);
        analysisFile << "Mesh bounds: (" << std::fixed << std::setprecision(2) 
                    << m_meshBMin[0] << ", " << m_meshBMin[1] << ", " << m_meshBMin[2] << ") to "
                    << "(" << m_meshBMax[0] << ", " << m_meshBMax[1] << ", " << m_meshBMax[2] << ")" << std::endl;
        analysisFile << std::endl;
        
        analysisFile << "CONFIGURATION:" << std::endl;
        analysisFile << "-------------" << std::endl;
        analysisFile << "Cell size: " << m_cfg.cellSize << " units" << std::endl;
        analysisFile << "Cell height: " << m_cfg.cellHeight << " units" << std::endl;
        analysisFile << "Agent radius: " << m_cfg.agentRadius << " units" << std::endl;
        analysisFile << "Agent height: " << m_cfg.agentHeight << " units" << std::endl;
        analysisFile << "Agent max climb: " << m_cfg.agentMaxClimb << " units" << std::endl;
        analysisFile << "Agent max slope: " << m_cfg.agentMaxSlope << " degrees" << std::endl;
        analysisFile << "Tile size: " << m_cfg.tileSize << " cells (" << tcs << " world units)" << std::endl;
        analysisFile << "Max polygons per tile: " << m_maxPolysPerTile << std::endl;
        analysisFile << "Max vertices per tile: 65535" << std::endl;
        analysisFile << "Tile grid: " << tw << " x " << th << " = " << (tw*th) << " total tiles" << std::endl;
        analysisFile << "Region min size: " << m_cfg.regionMinSize << std::endl;
        analysisFile << "Region merge size: " << m_cfg.regionMergeSize << std::endl;
        analysisFile << "Edge max length: " << m_cfg.edgeMaxLen << std::endl;
        analysisFile << "Edge max error: " << m_cfg.edgeMaxError << std::endl;
        analysisFile << std::endl;
        
        analysisFile << "PER-TILE DETAILED ANALYSIS:" << std::endl;
        analysisFile << "---------------------------" << std::endl;
        analysisFile << std::left 
                    << std::setw(8) << "Tile(X,Y)"
                    << std::setw(8) << "Index"
                    << std::setw(10) << "InputTris" 
                    << std::setw(10) << "EstPolys"
                    << std::setw(10) << "EstVerts"
                    << std::setw(12) << "TriDensity"
                    << std::setw(12) << "PolyDensity"
                    << std::setw(12) << "Status"
                    << std::setw(30) << "WorldBounds"
                    << std::setw(20) << "Center"
                    << "Warnings" << std::endl;
        analysisFile << std::string(140, '-') << std::endl;
        
        std::vector<TileAnalysis> tileAnalyses;
        int tilesWithGeometry = 0;
        int tilesExceedingPolyLimit = 0;
        int tilesExceedingVertLimit = 0;
        int maxPolysFound = 0;
        int maxVertsFound = 0;
        int maxTrianglesFound = 0;
        float maxTriDensity = 0.0f;
        float maxPolyDensity = 0.0f;
        float totalWorldArea = 0.0f;
        int totalTriangles = 0;
        
        // Progress tracking
        auto analysisStartTime = std::chrono::high_resolution_clock::now();
        
        for (int y = 0; y < th; ++y)
        {
            for (int x = 0; x < tw; ++x)
            {
                int currentTile = y * tw + x;
                if (currentTile % 200 == 0 || currentTile == tw*th - 1)
                {
                    int progress = ((currentTile + 1) * 100) / (tw*th);
                    std::cout << "\rAnalyzing tiles... " << progress << "% (" << (currentTile + 1) << "/" << (tw*th) << ")" << std::flush;
                }
                
                TileAnalysis analysis;
                analysis.x = x;
                analysis.y = y;
                analysis.tileIndex = currentTile;
                
                // Calculate precise tile bounds
                float tileBmin[3], tileBmax[3];
                tileBmin[0] = m_meshBMin[0] + x * tcs;
                tileBmin[1] = m_meshBMin[1];
                tileBmin[2] = m_meshBMin[2] + y * tcs;
                tileBmax[0] = m_meshBMin[0] + (x+1) * tcs;
                tileBmax[1] = m_meshBMax[1];
                tileBmax[2] = m_meshBMin[2] + (y+1) * tcs;
                
                // Store world bounds
                analysis.worldBounds[0] = tileBmin[0];  // minX
                analysis.worldBounds[1] = tileBmin[1];  // minY  
                analysis.worldBounds[2] = tileBmin[2];  // minZ
                analysis.worldBounds[3] = tileBmax[0];  // maxX
                analysis.worldBounds[4] = tileBmax[1];  // maxY
                analysis.worldBounds[5] = tileBmax[2];  // maxZ
                
                // Calculate tile world size and center
                analysis.tileWorldSize[0] = tileBmax[0] - tileBmin[0];  // width
                analysis.tileWorldSize[1] = tileBmax[2] - tileBmin[2];  // height
                analysis.tileCenterWorld[0] = (tileBmin[0] + tileBmax[0]) * 0.5f;
                analysis.tileCenterWorld[1] = (tileBmin[1] + tileBmax[1]) * 0.5f;
                analysis.tileCenterWorld[2] = (tileBmin[2] + tileBmax[2]) * 0.5f;
                
                float tileArea = analysis.tileWorldSize[0] * analysis.tileWorldSize[1];
                totalWorldArea += tileArea;
                
                // Expand bounds for border calculation (same as in buildTileMesh)
                const float borderSize = (float)(int)ceilf(m_cfg.agentRadius / m_cfg.cellSize) + 3;
                float expandedBmin[3] = {tileBmin[0] - borderSize * m_cfg.cellSize, tileBmin[1], tileBmin[2] - borderSize * m_cfg.cellSize};
                float expandedBmax[3] = {tileBmax[0] + borderSize * m_cfg.cellSize, tileBmax[1], tileBmax[2] + borderSize * m_cfg.cellSize};
                
                // Find overlapping triangles with expanded bounds
                float tbmin[2] = {expandedBmin[0], expandedBmin[2]};
                float tbmax[2] = {expandedBmax[0], expandedBmax[2]};
                std::vector<int> chunks = m_chunkyMesh.getChunksOverlappingRect(tbmin, tbmax);
                
                analysis.inputTriangles = 0;
                for (int chunkId : chunks)
                {
                    const ChunkyTriMeshNode& node = m_chunkyMesh.nodes[chunkId];
                    
                    // More precise triangle counting - check if triangles actually overlap tile
                    const int* ctris = &m_chunkyMesh.tris[(size_t)node.i * 3];
                    for (int t = 0; t < node.n; ++t)
                    {
                        const float* v0 = &m_mesh.getVerts()[ctris[t*3+0] * 3];
                        const float* v1 = &m_mesh.getVerts()[ctris[t*3+1] * 3];
                        const float* v2 = &m_mesh.getVerts()[ctris[t*3+2] * 3];
                        
                        // Check if triangle overlaps expanded tile bounds
                        float triMinX = std::min({v0[0], v1[0], v2[0]});
                        float triMaxX = std::max({v0[0], v1[0], v2[0]});
                        float triMinZ = std::min({v0[2], v1[2], v2[2]});
                        float triMaxZ = std::max({v0[2], v1[2], v2[2]});
                        
                        if (!(triMaxX < expandedBmin[0] || triMinX > expandedBmax[0] ||
                              triMaxZ < expandedBmin[2] || triMinZ > expandedBmax[2]))
                        {
                            analysis.inputTriangles++;
                        }
                    }
                }
                
                totalTriangles += analysis.inputTriangles;
                maxTrianglesFound = std::max(maxTrianglesFound, analysis.inputTriangles);
                
                analysis.hasGeometry = (analysis.inputTriangles > 0);
                
                if (analysis.hasGeometry)
                {
                    // Calculate density metrics
                    analysis.triangleDensity = (tileArea > 0) ? (analysis.inputTriangles / tileArea) : 0.0f;
                    maxTriDensity = std::max(maxTriDensity, analysis.triangleDensity);
                    
                    // Enhanced polygon estimation based on triangle complexity
                    // More sophisticated estimation considering various factors
                    float complexityFactor = 1.0f;
                    
                    // High triangle density usually means more complex geometry = fewer polys per triangle
                    if (analysis.triangleDensity > 100.0f) complexityFactor = 0.15f;      // Very dense
                    else if (analysis.triangleDensity > 50.0f) complexityFactor = 0.20f;  // Dense
                    else if (analysis.triangleDensity > 20.0f) complexityFactor = 0.25f;  // Moderate
                    else if (analysis.triangleDensity > 5.0f) complexityFactor = 0.30f;   // Sparse
                    else complexityFactor = 0.35f;                                        // Very sparse
                    
                    // Apply region settings influence
                    float regionInfluence = std::max(0.1f, std::min(1.0f, m_cfg.regionMinSize / 20.0f));
                    complexityFactor *= regionInfluence;
                    
                    analysis.estimatedPolygons = std::max(1, (int)(analysis.inputTriangles * complexityFactor));
                    
                    // Vertex estimation (polygons typically have 3-6 vertices, many shared)
                    analysis.estimatedVertices = std::max(analysis.estimatedPolygons * 3, 
                                                        std::min(analysis.estimatedPolygons * 5, 
                                                               analysis.inputTriangles * 2));
                    
                    analysis.polygonDensity = (tileArea > 0) ? (analysis.estimatedPolygons / tileArea) : 0.0f;
                    maxPolyDensity = std::max(maxPolyDensity, analysis.polygonDensity);
                    
                    tilesWithGeometry++;
                    
                    // Check limits and add warnings
                    analysis.exceedsPolyLimit = (analysis.estimatedPolygons > m_maxPolysPerTile);
                    analysis.exceedsVertLimit = (analysis.estimatedVertices > 65535);
                    
                    if (analysis.exceedsPolyLimit) 
                    {
                        tilesExceedingPolyLimit++;
                        analysis.warningMessages += "POLY_LIMIT ";
                    }
                    if (analysis.exceedsVertLimit) 
                    {
                        tilesExceedingVertLimit++;
                        analysis.warningMessages += "VERT_LIMIT ";
                    }
                    if (analysis.triangleDensity > 100.0f)
                        analysis.warningMessages += "HIGH_DENSITY ";
                    if (analysis.inputTriangles > 50000)
                        analysis.warningMessages += "LARGE_TRIANGLE_COUNT ";
                }
                else
                {
                    analysis.estimatedPolygons = 0;
                    analysis.estimatedVertices = 0;
                    analysis.triangleDensity = 0.0f;
                    analysis.polygonDensity = 0.0f;
                }
                
                maxPolysFound = std::max(maxPolysFound, analysis.estimatedPolygons);
                maxVertsFound = std::max(maxVertsFound, analysis.estimatedVertices);
                
                tileAnalyses.push_back(analysis);
                
                // Write detailed tile information to file
                std::string statusStr;
                if (!analysis.hasGeometry) statusStr = "EMPTY";
                else if (analysis.exceedsPolyLimit || analysis.exceedsVertLimit) statusStr = "LIMIT_RISK";
                else statusStr = "OK";
                
                std::string boundsStr = "(" + std::to_string((int)analysis.worldBounds[0]) + "," + 
                                       std::to_string((int)analysis.worldBounds[2]) + ")-(" +
                                       std::to_string((int)analysis.worldBounds[3]) + "," + 
                                       std::to_string((int)analysis.worldBounds[5]) + ")";
                
                std::string centerStr = "(" + std::to_string((int)analysis.tileCenterWorld[0]) + "," +
                                       std::to_string((int)analysis.tileCenterWorld[1]) + "," +
                                       std::to_string((int)analysis.tileCenterWorld[2]) + ")";
                
                analysisFile << std::left 
                            << std::setw(8) << ("(" + std::to_string(x) + "," + std::to_string(y) + ")")
                            << std::setw(8) << analysis.tileIndex
                            << std::setw(10) << analysis.inputTriangles
                            << std::setw(10) << analysis.estimatedPolygons
                            << std::setw(10) << analysis.estimatedVertices
                            << std::setw(12) << std::fixed << std::setprecision(2) << analysis.triangleDensity
                            << std::setw(12) << std::fixed << std::setprecision(2) << analysis.polygonDensity
                            << std::setw(12) << statusStr
                            << std::setw(30) << boundsStr
                            << std::setw(20) << centerStr
                            << analysis.warningMessages << std::endl;
            }
        }
        
        auto analysisEndTime = std::chrono::high_resolution_clock::now();
        float analysisTimeMs = std::chrono::duration<float, std::milli>(analysisEndTime - analysisStartTime).count();
        
        std::cout << std::endl; // New line after progress
        
        // Write comprehensive summary
        analysisFile << std::endl << std::string(140, '=') << std::endl;
        analysisFile << "COMPREHENSIVE ANALYSIS SUMMARY" << std::endl;
        analysisFile << std::string(140, '=') << std::endl;
        
        analysisFile << "\nGEOMETRY STATISTICS:" << std::endl;
        analysisFile << "-------------------" << std::endl;
        analysisFile << "Total tiles: " << tileAnalyses.size() << std::endl;
        analysisFile << "Tiles with geometry: " << tilesWithGeometry << " (" << std::fixed << std::setprecision(1) 
                    << (tilesWithGeometry * 100.0f / tileAnalyses.size()) << "%)" << std::endl;
        analysisFile << "Empty tiles: " << (tileAnalyses.size() - tilesWithGeometry) << " (" << std::fixed << std::setprecision(1) 
                    << ((tileAnalyses.size() - tilesWithGeometry) * 100.0f / tileAnalyses.size()) << "%)" << std::endl;
        analysisFile << "Total input triangles: " << totalTriangles << std::endl;
        analysisFile << "Average triangles per non-empty tile: " << (tilesWithGeometry > 0 ? (totalTriangles / tilesWithGeometry) : 0) << std::endl;
        analysisFile << "Total world area covered: " << std::fixed << std::setprecision(0) << totalWorldArea << " sq units" << std::endl;
        
        analysisFile << "\nPERFORMANCE PREDICTIONS:" << std::endl;
        analysisFile << "----------------------" << std::endl;
        analysisFile << "Tiles exceeding polygon limit (" << m_maxPolysPerTile << "): " << tilesExceedingPolyLimit;
        if (tilesExceedingPolyLimit > 0) analysisFile << " ⚠️  WARNING";
        analysisFile << std::endl;
        analysisFile << "Tiles exceeding vertex limit (65535): " << tilesExceedingVertLimit;
        if (tilesExceedingVertLimit > 0) analysisFile << " ⚠️  WARNING";
        analysisFile << std::endl;
        analysisFile << "Estimated total polygons: " << std::accumulate(tileAnalyses.begin(), tileAnalyses.end(), 0, 
                                                                        [](int sum, const TileAnalysis& t) { return sum + t.estimatedPolygons; }) << std::endl;
        analysisFile << "Estimated total vertices: " << std::accumulate(tileAnalyses.begin(), tileAnalyses.end(), 0, 
                                                                        [](int sum, const TileAnalysis& t) { return sum + t.estimatedVertices; }) << std::endl;
        
        analysisFile << "\nEXTREME VALUES:" << std::endl;
        analysisFile << "--------------" << std::endl;
        analysisFile << "Max triangles in any tile: " << maxTrianglesFound << std::endl;
        analysisFile << "Max estimated polygons in any tile: " << maxPolysFound << std::endl;
        analysisFile << "Max estimated vertices in any tile: " << maxVertsFound << std::endl;
        analysisFile << "Max triangle density: " << std::fixed << std::setprecision(2) << maxTriDensity << " tri/sq unit" << std::endl;
        analysisFile << "Max polygon density: " << std::fixed << std::setprecision(2) << maxPolyDensity << " poly/sq unit" << std::endl;
        
        analysisFile << "\nTIMING:" << std::endl;
        analysisFile << "-------" << std::endl;
        analysisFile << "Analysis completed in: " << std::fixed << std::setprecision(2) << (analysisTimeMs / 1000.0f) << " seconds" << std::endl;
        
        // Risk assessment and recommendations
        analysisFile << "\nRISK ASSESSMENT:" << std::endl;
        analysisFile << "----------------" << std::endl;
        if (tilesExceedingPolyLimit > 0 || tilesExceedingVertLimit > 0)
        {
            analysisFile << "⚠️  HIGH RISK: Some tiles may fail to build or be truncated!" << std::endl;
            analysisFile << "\nRECOMMENDATIONS:" << std::endl;
            if (tilesExceedingPolyLimit > 0)
            {
                analysisFile << "• Increase tile size (current: " << m_cfg.tileSize << " -> try " << (m_cfg.tileSize * 2) << ")" << std::endl;
                analysisFile << "• Increase regionMinSize (current: " << m_cfg.regionMinSize << " -> try " << (m_cfg.regionMinSize * 2) << ")" << std::endl;
                analysisFile << "• Increase edgeMaxLen (current: " << m_cfg.edgeMaxLen << " -> try " << (m_cfg.edgeMaxLen * 1.5f) << ")" << std::endl;
                analysisFile << "• Increase edgeMaxError (current: " << m_cfg.edgeMaxError << " -> try " << (m_cfg.edgeMaxError * 1.5f) << ")" << std::endl;
            }
            if (tilesExceedingVertLimit > 0)
            {
                analysisFile << "• Increase cell size for less detail (current: " << m_cfg.cellSize << ")" << std::endl;
                analysisFile << "• Increase detailSampleDist (current: " << m_cfg.detailSampleDist << ")" << std::endl;
            }
        }
        else if (maxPolysFound > m_maxPolysPerTile * 0.8f)
        {
            analysisFile << "⚠️  MEDIUM RISK: Some tiles are close to polygon limits" << std::endl;
            analysisFile << "Consider increasing tile size or region settings for safety margin." << std::endl;
        }
        else
        {
            analysisFile << "✅ LOW RISK: All tiles should build successfully" << std::endl;
        }
        
        analysisFile << "\nESTIMATED BUILD TIME:" << std::endl;
        analysisFile << "-------------------" << std::endl;
        float estimatedBuildTimeMinutes = (tilesWithGeometry * 0.1f) + (totalTriangles / 1000000.0f * 60.0f);
        analysisFile << "Rough estimate: " << std::fixed << std::setprecision(1) << estimatedBuildTimeMinutes << " minutes" << std::endl;
        analysisFile << "(Actual time depends on hardware and mesh complexity)" << std::endl;
        
        analysisFile.close();
        
        // Console summary
        std::cout << "\n=== ANALYSIS COMPLETE ===" << std::endl;
        std::cout << "Analysis time: " << std::fixed << std::setprecision(2) << (analysisTimeMs / 1000.0f) << " seconds" << std::endl;
        std::cout << "Total tiles: " << tileAnalyses.size() << std::endl;
        std::cout << "Tiles with geometry: " << tilesWithGeometry << " (" << std::fixed << std::setprecision(1) 
                  << (tilesWithGeometry * 100.0f / tileAnalyses.size()) << "%)" << std::endl;
        std::cout << "Empty tiles: " << (tileAnalyses.size() - tilesWithGeometry) << std::endl;
        std::cout << "Total triangles: " << totalTriangles << std::endl;
        std::cout << "Max estimated polygons per tile: " << maxPolysFound << std::endl;
        std::cout << "Max estimated vertices per tile: " << maxVertsFound << std::endl;
        
        if (tilesExceedingPolyLimit > 0 || tilesExceedingVertLimit > 0)
        {
            std::cout << "\n\033[1;33m" << "⚠️  WARNING: POTENTIAL BUILD ISSUES DETECTED!" << "\033[0m" << std::endl;
            std::cout << "\033[1;31m" << "Tiles exceeding polygon limit: " << tilesExceedingPolyLimit << "\033[0m" << std::endl;
            std::cout << "\033[1;31m" << "Tiles exceeding vertex limit: " << tilesExceedingVertLimit << "\033[0m" << std::endl;
            std::cout << "\033[1;33m" << "These tiles may fail to build or produce truncated results!" << "\033[0m" << std::endl;
        }
        else if (maxPolysFound > m_maxPolysPerTile * 0.8f)
        {
            std::cout << "\n\033[1;33m" << "⚠️  CAUTION: Some tiles are close to limits" << "\033[0m" << std::endl;
            std::cout << "Consider increasing tile size for safety margin." << std::endl;
        }
        else
        {
            std::cout << "\n\033[1;32m" << "✅ All tiles look good to build!" << "\033[0m" << std::endl;
        }
        
        std::cout << "\n\033[1;36m" << "Detailed report saved to: " << reportFilename << "\033[0m" << std::endl;
        std::cout << "Estimated build time: ~" << std::fixed << std::setprecision(1) 
                  << estimatedBuildTimeMinutes << " minutes" << std::endl;
        
        // User decision prompt
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "\033[1;37m" << "Do you want to proceed with the full navmesh build?" << "\033[0m" << std::endl;
        std::cout << "This will build " << tilesWithGeometry << " tiles with geometry." << std::endl;
        std::cout << "Enter your choice:" << std::endl;
        std::cout << "  [Y] Yes - Proceed with full build" << std::endl;
        std::cout << "  [N] No - Cancel build" << std::endl;
        std::cout << "  [S] Show risky tiles only" << std::endl;
        std::cout << "Choice (Y/N/S): ";
        
        char response;
        std::cin >> response;
        response = std::toupper(response);
        
        if (response == 'S')
        {
            // Show details of risky tiles
            std::cout << "\nRISKY TILES DETAIL:" << std::endl;
            std::cout << std::string(80, '-') << std::endl;
            bool foundRisky = false;
            
            for (const auto& tile : tileAnalyses)
            {
                if (tile.exceedsPolyLimit || tile.exceedsVertLimit)
                {
                    foundRisky = true;
                    std::cout << "Tile (" << tile.x << "," << tile.y << "): " 
                             << tile.inputTriangles << " triangles, "
                             << tile.estimatedPolygons << " est. polygons, "
                             << tile.estimatedVertices << " est. vertices";
                    if (tile.exceedsPolyLimit) std::cout << " [POLY LIMIT]";
                    if (tile.exceedsVertLimit) std::cout << " [VERT LIMIT]";
                    std::cout << std::endl;
                }
            }
            
            if (!foundRisky)
            {
                std::cout << "No risky tiles found!" << std::endl;
            }
            
            std::cout << "\nProceed anyway? (Y/N): ";
            std::cin >> response;
            response = std::toupper(response);
        }
        
        bool shouldContinue = (response == 'Y');
        
        if (!shouldContinue)
        {
            std::cout << "\n\033[1;33m" << "Build cancelled by user." << "\033[0m" << std::endl;
            std::cout << "You can:" << std::endl;
            std::cout << "1. Adjust configuration settings and try again" << std::endl;
            std::cout << "2. Review the detailed analysis report: " << reportFilename << std::endl;
            std::cout << "3. Consider the recommendations in the report" << std::endl;
            
            if (tilesExceedingPolyLimit > 0)
            {
                std::cout << "\nTo fix polygon limit issues:" << std::endl;
                std::cout << "- Increase tile size (current: " << m_cfg.tileSize << ")" << std::endl;
                std::cout << "- Increase regionMinSize (current: " << m_cfg.regionMinSize << ")" << std::endl;
                std::cout << "- Increase edgeMaxLen (current: " << m_cfg.edgeMaxLen << ")" << std::endl;
                std::cout << "- Increase edgeMaxError (current: " << m_cfg.edgeMaxError << ")" << std::endl;
            }
            
            if (tilesExceedingVertLimit > 0)
            {
                std::cout << "\nTo fix vertex limit issues:" << std::endl;
                std::cout << "- Increase cell size (current: " << m_cfg.cellSize << ")" << std::endl;
                std::cout << "- Increase detailSampleDist (current: " << m_cfg.detailSampleDist << ")" << std::endl;
                std::cout << "- Reduce detail mesh resolution" << std::endl;
            }
        }
        else
        {
            std::cout << "\n\033[1;32m" << "Proceeding with navmesh build..." << "\033[0m" << std::endl;
            std::cout << "Building " << tilesWithGeometry << " tiles..." << std::endl;
        }
        
        return shouldContinue;
    }
    
    bool buildNavMesh()
    {
        if (!m_mesh.getVerts() || !m_mesh.getTris())
        {
            std::cerr << "No mesh data loaded" << std::endl;
            return false;
        }
        
        cleanup();
        
        int gw = 0, gh = 0;
        rcCalcGridSize(m_meshBMin, m_meshBMax, m_cfg.cellSize, &gw, &gh);
        const int ts = (int)m_cfg.tileSize;
        const int tw = (gw + ts-1) / ts;
        const int th = (gh + ts-1) / ts;
        
        int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
        if (tileBits > 14) tileBits = 14;
        int polyBits = 22 - tileBits;
        m_maxTiles = 1 << tileBits;
        m_maxPolysPerTile = 1 << polyBits;
        
        std::cout << "\nNavMesh Configuration:" << std::endl;
        std::cout << "  Grid size: " << gw << " x " << gh << std::endl;
        std::cout << "  Tiles: " << tw << " x " << th << " (" << (tw*th) << " total)" << std::endl;
        std::cout << "  Max tiles: " << m_maxTiles << std::endl;
        std::cout << "  Max polys per tile: " << m_maxPolysPerTile << std::endl;
        
        // Run comprehensive analysis first
        const float tcs = m_cfg.tileSize * m_cfg.cellSize;
        if (!analyzeTilesComprehensive(tw, th, tcs))
        {
            return false; // User cancelled the build
        }
        
        m_navMesh = dtAllocNavMesh();
        if (!m_navMesh)
        {
            m_ctx->log(RC_LOG_ERROR, "Could not allocate navmesh.");
            return false;
        }
        
        dtNavMeshParams params;
        rcVcopy(params.orig, m_meshBMin);
        params.tileWidth = tcs;
        params.tileHeight = tcs;
        params.maxTiles = m_maxTiles;
        params.maxPolys = m_maxPolysPerTile;
        
        dtStatus status = m_navMesh->init(&params);
        if (dtStatusFailed(status))
        {
            m_ctx->log(RC_LOG_ERROR, "Could not init navmesh.");
            return false;
        }
        
        std::cout << "\nBuilding tiles..." << std::endl;
        
        m_ctx->startTimer(RC_TIMER_TEMP);
        
        // Create build results log file
        std::ofstream buildLog("navmesh_build_results.txt");
        if (buildLog.is_open())
        {
            buildLog << "NAVMESH BUILD RESULTS LOG" << std::endl;
            buildLog << "=========================" << std::endl;
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            buildLog << "Build started: " << std::ctime(&time_t) << std::endl;
            buildLog << std::left 
                    << std::setw(8) << "Tile(X,Y)"
                    << std::setw(8) << "Status"
                    << std::setw(10) << "ActPolys"
                    << std::setw(10) << "ActVerts"
                    << std::setw(12) << "BuildTime"
                    << std::setw(12) << "MemoryKB"
                    << "Messages" << std::endl;
            buildLog << std::string(80, '-') << std::endl;
        }
        
        int successfulTiles = 0;
        int failedTiles = 0;
        int totalActualPolygons = 0;
        int totalActualVertices = 0;
        
        for (int y = 0; y < th; ++y)
        {
            for (int x = 0; x < tw; ++x)
            {
                int progress = ((y*tw + x + 1) * 100) / (tw*th);
                std::cout << "\rBuilding tile (" << x << ", " << y << ") - " << progress << "%" << std::flush;
                
                float tileBmin[3], tileBmax[3];
                tileBmin[0] = m_meshBMin[0] + x * tcs;
                tileBmin[1] = m_meshBMin[1];
                tileBmin[2] = m_meshBMin[2] + y * tcs;
                tileBmax[0] = m_meshBMin[0] + (x+1) * tcs;
                tileBmax[1] = m_meshBMax[1];
                tileBmax[2] = m_meshBMin[2] + (y+1) * tcs;
                
                int dataSize = 0;
                TileData tileData;
                unsigned char* data = buildTileMesh(x, y, tileBmin, tileBmax, dataSize, tileData);
                
                std::string tileStatus = "FAILED";
                if (data)
                {
                    m_navMesh->removeTile(m_navMesh->getTileRefAt(x, y, 0), 0, 0);
                    status = m_navMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);
                    if (dtStatusFailed(status))
                    {
                        tileData.buildError = "Failed to add tile to navmesh";
                        dtFree(data);
                        failedTiles++;
                    }
                    else
                    {
                        tileStatus = "SUCCESS";
                        successfulTiles++;
                        totalActualPolygons += tileData.actualPolygons;
                        totalActualVertices += tileData.actualVertices;
                    }
                }
                else
                {
                    failedTiles++;
                    if (tileData.buildError.empty()) tileData.buildError = "Unknown build failure";
                }
                
                // Log tile build result
                if (buildLog.is_open())
                {
                    buildLog << std::left 
                            << std::setw(8) << ("(" + std::to_string(x) + "," + std::to_string(y) + ")")
                            << std::setw(8) << tileStatus
                            << std::setw(10) << tileData.actualPolygons
                            << std::setw(10) << tileData.actualVertices
                            << std::setw(12) << std::fixed << std::setprecision(1) << tileData.buildTimeMs
                            << std::setw(12) << tileData.memoryUsedKB
                            << tileData.buildError << " " << tileData.warningMessages << std::endl;
                }
            }
        }
        
        std::cout << std::endl; // New line after progress
        
        m_ctx->stopTimer(RC_TIMER_TEMP);
        float totalTime = m_ctx->getAccumulatedTime(RC_TIMER_TEMP) / 1000.0f;
        
        // Write final summary to log
        if (buildLog.is_open())
        {
            buildLog << std::endl << "BUILD SUMMARY:" << std::endl;
            buildLog << "==============" << std::endl;
            buildLog << "Total tiles processed: " << (tw * th) << std::endl;
            buildLog << "Successful tiles: " << successfulTiles << std::endl;
            buildLog << "Failed tiles: " << failedTiles << std::endl;
            buildLog << "Success rate: " << std::fixed << std::setprecision(1) 
                    << (successfulTiles * 100.0f / (tw * th)) << "%" << std::endl;
            buildLog << "Total actual polygons: " << totalActualPolygons << std::endl;
            buildLog << "Total actual vertices: " << totalActualVertices << std::endl;
            buildLog << "Average polygons per successful tile: " 
                    << (successfulTiles > 0 ? (totalActualPolygons / successfulTiles) : 0) << std::endl;
            buildLog << "Average vertices per successful tile: " 
                    << (successfulTiles > 0 ? (totalActualVertices / successfulTiles) : 0) << std::endl;
            buildLog << "Total build time: " << std::fixed << std::setprecision(2) << totalTime << " seconds" << std::endl;
            buildLog.close();
        }
        
        std::cout << "\nNavMesh build completed in " << totalTime << " seconds" << std::endl;
        
        std::cout << "Successfully created " << successfulTiles << " tiles out of " << (tw*th) << " total." << std::endl;
        if (failedTiles > 0)
        {
            std::cout << "\033[1;33m" << "Failed tiles: " << failedTiles << "\033[0m" << std::endl;
            std::cout << "Check navmesh_build_results.txt for details on failures." << std::endl;
        }
        
        return successfulTiles > 0;
    }
    
    unsigned char* buildTileMesh(int tx, int ty, const float* bmin, const float* bmax, int& dataSize, TileData& tileData)
    {
        dataSize = 0;
        auto buildStartTime = std::chrono::high_resolution_clock::now();
        
        const float* verts = m_mesh.getVerts();
        const int nverts = m_mesh.getVertCount();
        
        rcConfig cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.cs = m_cfg.cellSize;
        cfg.ch = m_cfg.cellHeight;
        cfg.walkableSlopeAngle = m_cfg.agentMaxSlope;
        cfg.walkableHeight = (int)ceilf(m_cfg.agentHeight / cfg.ch);
        cfg.walkableClimb = (int)floorf(m_cfg.agentMaxClimb / cfg.ch);
        cfg.walkableRadius = (int)ceilf(m_cfg.agentRadius / cfg.cs);
        cfg.maxEdgeLen = (int)(m_cfg.edgeMaxLen / m_cfg.cellSize);
        cfg.maxSimplificationError = m_cfg.edgeMaxError;
        cfg.minRegionArea = (int)rcSqr(m_cfg.regionMinSize);
        cfg.mergeRegionArea = (int)rcSqr(m_cfg.regionMergeSize);
        cfg.maxVertsPerPoly = m_cfg.vertsPerPoly;
        cfg.tileSize = (int)m_cfg.tileSize;
        cfg.borderSize = cfg.walkableRadius + 3;
        cfg.width = cfg.tileSize + cfg.borderSize * 2;
        cfg.height = cfg.tileSize + cfg.borderSize * 2;
        cfg.detailSampleDist = m_cfg.detailSampleDist < 0.9f ? 0 : m_cfg.cellSize * m_cfg.detailSampleDist;
        cfg.detailSampleMaxError = m_cfg.cellHeight * m_cfg.detailSampleMaxError;
        
        rcVcopy(cfg.bmin, bmin);
        rcVcopy(cfg.bmax, bmax);
        cfg.bmin[0] -= cfg.borderSize * cfg.cs;
        cfg.bmin[2] -= cfg.borderSize * cfg.cs;
        cfg.bmax[0] += cfg.borderSize * cfg.cs;
        cfg.bmax[2] += cfg.borderSize * cfg.cs;
        
        rcHeightfield* solid = rcAllocHeightfield();
        if (!solid || !rcCreateHeightfield(m_ctx, *solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
        {
            if (solid) rcFreeHeightField(solid);
            tileData.buildError = "Failed to create heightfield";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        float tbmin[2] = {cfg.bmin[0], cfg.bmin[2]};
        float tbmax[2] = {cfg.bmax[0], cfg.bmax[2]};
        std::vector<int> chunks = m_chunkyMesh.getChunksOverlappingRect(tbmin, tbmax);
        
        if (chunks.empty())
        {
            rcFreeHeightField(solid);
            tileData.buildError = "No geometry in tile";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        // Count total triangles in chunks
        int nctrisTotal = 0;
        for(int chunkId : chunks) {
            nctrisTotal += m_chunkyMesh.nodes[chunkId].n;
        }
        std::vector<unsigned char> triareas(nctrisTotal, 0);
        int triareasOffset = 0;
        
        for (int chunkId : chunks)
        {
            const ChunkyTriMeshNode& node = m_chunkyMesh.nodes[chunkId];
            const int* ctris = &m_chunkyMesh.tris[(size_t)node.i * 3];
            const int nctris = node.n;
            
            rcMarkWalkableTriangles(m_ctx, cfg.walkableSlopeAngle, verts, nverts, ctris, nctris, &triareas[triareasOffset]);
            
            if (!rcRasterizeTriangles(m_ctx, verts, nverts, ctris, &triareas[triareasOffset], nctris, *solid, cfg.walkableClimb))
            {
                rcFreeHeightField(solid);
                tileData.buildError = "Failed to rasterize triangles";
                auto buildEndTime = std::chrono::high_resolution_clock::now();
                tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
                return nullptr;
            }
            triareasOffset += nctris;
        }
        
        if (m_cfg.filterLowHangingObstacles)
            rcFilterLowHangingWalkableObstacles(m_ctx, cfg.walkableClimb, *solid);
        if (m_cfg.filterLedgeSpans)
            rcFilterLedgeSpans(m_ctx, cfg.walkableHeight, cfg.walkableClimb, *solid);
        if (m_cfg.filterWalkableLowHeightSpans)
            rcFilterWalkableLowHeightSpans(m_ctx, cfg.walkableHeight, *solid);
        
        rcCompactHeightfield* chf = rcAllocCompactHeightfield();
        if (!chf || !rcBuildCompactHeightfield(m_ctx, cfg.walkableHeight, cfg.walkableClimb, *solid, *chf))
        {
            if (chf) rcFreeCompactHeightfield(chf);
            rcFreeHeightField(solid);
            tileData.buildError = "Failed to build compact heightfield";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        rcFreeHeightField(solid);
        
        if (!rcErodeWalkableArea(m_ctx, cfg.walkableRadius, *chf))
        {
            rcFreeCompactHeightfield(chf);
            tileData.buildError = "Failed to erode walkable area";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        bool regionsBuilt = false;
        if (m_cfg.partitionType == SAMPLE_PARTITION_WATERSHED)
        {
            if (rcBuildDistanceField(m_ctx, *chf))
                regionsBuilt = rcBuildRegions(m_ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea);
        }
        else if (m_cfg.partitionType == SAMPLE_PARTITION_MONOTONE)
        {
            regionsBuilt = rcBuildRegionsMonotone(m_ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea);
        }
        else
        {
            regionsBuilt = rcBuildLayerRegions(m_ctx, *chf, cfg.borderSize, cfg.minRegionArea);
        }
        
        if (!regionsBuilt)
        {
            rcFreeCompactHeightfield(chf);
            tileData.buildError = "Failed to build regions";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        rcContourSet* cset = rcAllocContourSet();
        if (!cset || !rcBuildContours(m_ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset))
        {
            if (cset) rcFreeContourSet(cset);
            rcFreeCompactHeightfield(chf);
            tileData.buildError = "Failed to build contours";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        if (cset->nconts == 0)
        {
            rcFreeContourSet(cset);
            rcFreeCompactHeightfield(chf);
            tileData.buildError = "No contours generated (no walkable area)";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        rcPolyMesh* pmesh = rcAllocPolyMesh();
        if (!pmesh || !rcBuildPolyMesh(m_ctx, *cset, cfg.maxVertsPerPoly, *pmesh))
        {
            if (pmesh) rcFreePolyMesh(pmesh);
            rcFreeContourSet(cset);
            rcFreeCompactHeightfield(chf);
            tileData.buildError = "Failed to build polygon mesh";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
        if (!dmesh || !rcBuildPolyMeshDetail(m_ctx, *pmesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh))
        {
            if (dmesh) rcFreePolyMeshDetail(dmesh);
            rcFreePolyMesh(pmesh);
            rcFreeContourSet(cset);
            rcFreeCompactHeightfield(chf);
            tileData.buildError = "Failed to build detail mesh";
            auto buildEndTime = std::chrono::high_resolution_clock::now();
            tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
            return nullptr;
        }
        
        rcFreeCompactHeightfield(chf);
        rcFreeContourSet(cset);
        
        unsigned char* navData = nullptr;
        int navDataSize = 0;
        
        if (cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON && pmesh->nverts < 0xffff)
        {
            // Capture actual polygon and vertex counts
            tileData.actualPolygons = pmesh->npolys;
            tileData.actualVertices = pmesh->nverts;
            
            // Check for limit warnings
            if (tileData.actualPolygons > m_maxPolysPerTile)
            {
                tileData.warningMessages += "ACTUAL_POLY_LIMIT_EXCEEDED ";
            }
            if (tileData.actualVertices > 65535)
            {
                tileData.warningMessages += "ACTUAL_VERT_LIMIT_EXCEEDED ";
            }
            
            for (int i = 0; i < pmesh->npolys; ++i)
            {
                if (pmesh->areas[i] == RC_WALKABLE_AREA)
                    pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
                
                if (pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
                    pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
                    pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
                {
                    pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
                }
                else if (pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
                {
                    pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
                }
                else if (pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
                {
                    pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
                }
            }
            
            dtNavMeshCreateParams params;
            memset(&params, 0, sizeof(params));
            params.verts = pmesh->verts;
            params.vertCount = pmesh->nverts;
            params.polys = pmesh->polys;
            params.polyAreas = pmesh->areas;
            params.polyFlags = pmesh->flags;
            params.polyCount = pmesh->npolys;
            params.nvp = pmesh->nvp;
            params.detailMeshes = dmesh->meshes;
            params.detailVerts = dmesh->verts;
            params.detailVertsCount = dmesh->nverts;
            params.detailTris = dmesh->tris;
            params.detailTriCount = dmesh->ntris;
            params.offMeshConVerts = nullptr;
            params.offMeshConRad = nullptr;
            params.offMeshConDir = nullptr;
            params.offMeshConAreas = nullptr;
            params.offMeshConFlags = nullptr;
            params.offMeshConUserID = nullptr;
            params.offMeshConCount = 0;
            params.walkableHeight = m_cfg.agentHeight;
            params.walkableRadius = m_cfg.agentRadius;
            params.walkableClimb = m_cfg.agentMaxClimb;
            params.tileX = tx;
            params.tileY = ty;
            params.tileLayer = 0;
            rcVcopy(params.bmin, pmesh->bmin);
            rcVcopy(params.bmax, pmesh->bmax);
            params.cs = cfg.cs;
            params.ch = cfg.ch;
            params.buildBvTree = true;
            
            if (dtCreateNavMeshData(&params, &navData, &navDataSize))
            {
                dataSize = navDataSize;
                tileData.memoryUsedKB = navDataSize / 1024;
            }
            else
            {
                tileData.buildError = "Failed to create navmesh data";
            }
        }
        else
        {
            tileData.buildError = "Too many vertices per poly or too many verts in tile";
        }
        
        auto buildEndTime = std::chrono::high_resolution_clock::now();
        tileData.buildTimeMs = std::chrono::duration<float, std::milli>(buildEndTime - buildStartTime).count();
        
        rcFreePolyMeshDetail(dmesh);
        rcFreePolyMesh(pmesh);
        
        return navData;
    }
    
    bool saveNavMesh(const std::string& filename)
    {
        if (!m_navMesh)
        {
            std::cerr << "No navmesh to save" << std::endl;
            return false;
        }
        
        int gw = 0, gh = 0;
        rcCalcGridSize(m_meshBMin, m_meshBMax, m_cfg.cellSize, &gw, &gh);
        const int ts = (int)m_cfg.tileSize;
        const int tw = (gw + ts-1) / ts;
        const int th = (gh + ts-1) / ts;
        
        std::ofstream file(filename, std::ios::binary);
        if (!file)
        {
            std::cerr << "Failed to create file: " << filename << std::endl;
            return false;
        }
        
        const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
        const int NAVMESHSET_VERSION = 1;
        
        file.write(reinterpret_cast<const char*>(&NAVMESHSET_MAGIC), sizeof(int));
        file.write(reinterpret_cast<const char*>(&NAVMESHSET_VERSION), sizeof(int));
        
        int numTiles = 0;
        std::vector<dtTileRef> validTileRefs;
        
        for (int y = 0; y < th; ++y)
        {
            for (int x = 0; x < tw; ++x)
            {
                dtTileRef tileRef = m_navMesh->getTileRefAt(x, y, 0);
                if (tileRef != 0)
                {
                    numTiles++;
                    validTileRefs.push_back(tileRef);
                }
            }
        }
        
        file.write(reinterpret_cast<const char*>(&numTiles), sizeof(int));
        
        const dtNavMeshParams* params = m_navMesh->getParams();
        file.write(reinterpret_cast<const char*>(params), sizeof(dtNavMeshParams));
        
        for (dtTileRef tileRef : validTileRefs)
        {
            const dtMeshTile* tile = nullptr;
            m_navMesh->getTileAndPolyByRef(tileRef, &tile, nullptr);
            if (!tile || !tile->header || !tile->dataSize) continue;
            
            file.write(reinterpret_cast<const char*>(&tileRef), sizeof(dtTileRef));
            file.write(reinterpret_cast<const char*>(&tile->dataSize), sizeof(int));
            file.write(reinterpret_cast<const char*>(tile->data), tile->dataSize);
        }
        
        std::cout << "\nNavMesh saved to: " << filename << std::endl;
        std::cout << "Tiles: " << numTiles << std::endl;
        
        return true;
    }
    
    dtNavMesh* getNavMesh() { return m_navMesh; }
};

int main(int argc, char* argv[])
{
    std::cout << "Standalone Tiled NavMesh Builder v4.0 - Enhanced Analysis Edition" << std::endl;
    std::cout << "=================================================================" << std::endl;
    
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input.obj> <output.bin>" << std::endl;
        std::cout << "\nBuilds a tiled navmesh from large OBJ files with comprehensive analysis." << std::endl;
        std::cout << "Features:" << std::endl;
        std::cout << "- Pre-build tile analysis with detailed measurements" << std::endl;
        std::cout << "- Per-tile polygon/vertex counting and risk assessment" << std::endl;
        std::cout << "- Interactive build confirmation with detailed reports" << std::endl;
        std::cout << "- Build result logging and performance tracking" << std::endl;
        std::cout << "\nExample:" << std::endl;
        std::cout << "  " << argv[0] << " NAVTest137Wipv4v3.obj navmesh.bin" << std::endl;
        return 1;
    }
    
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    
    std::cout << "\nInput:  " << inputFile << std::endl;
    std::cout << "Output: " << outputFile << std::endl;
    
    TiledNavMeshBuilder builder;
    
    std::cout << "\n=== Phase 1: Loading Mesh ===" << std::endl;
    if (!builder.loadMesh(inputFile))
    {
        std::cerr << "\nERROR: Failed to load mesh: " << inputFile << std::endl;
        return 1;
    }
    
    std::cout << "\n=== Phase 2: Building Tiled NavMesh ===" << std::endl;
    if (!builder.buildNavMesh())
    {
        std::cerr << "\nERROR: Failed to build navmesh or build was cancelled by user." << std::endl;
        return 1;
    }
    
    std::cout << "\n=== Phase 3: Saving NavMesh ===" << std::endl;
    if (!builder.saveNavMesh(outputFile))
    {
        std::cerr << "\nERROR: Failed to save navmesh" << std::endl;
        return 1;
    }
    
    std::cout << "\n=== SUCCESS ===" << std::endl;
    std::cout << "Tiled navmesh generation completed!" << std::endl;
    std::cout << "Generated navmesh saved to: " << outputFile << std::endl;
    std::cout << "\nGenerated files:" << std::endl;
    std::cout << "- " << outputFile << " (navmesh binary)" << std::endl;
    std::cout << "- comprehensive_tile_analysis.txt (pre-build analysis)" << std::endl;
    std::cout << "- navmesh_build_results.txt (build results)" << std::endl;
    
    return 0;
}