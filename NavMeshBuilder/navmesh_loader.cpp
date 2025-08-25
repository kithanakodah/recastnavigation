#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"

class NavMeshLoader
{
private:
    dtNavMesh* m_navMesh;
    dtNavMeshQuery* m_navQuery;
    
public:
    NavMeshLoader() : m_navMesh(nullptr), m_navQuery(nullptr) {}
    
    ~NavMeshLoader()
    {
        cleanup();
    }
    
    void cleanup()
    {
        if (m_navQuery)
        {
            dtFreeNavMeshQuery(m_navQuery);
            m_navQuery = nullptr;
        }
        if (m_navMesh)
        {
            dtFreeNavMesh(m_navMesh);
            m_navMesh = nullptr;
        }
    }
    
    bool loadNavMesh(const std::string& filename)
    {
        cleanup();
        
        std::ifstream file(filename, std::ios::binary);
        if (!file)
        {
            std::cerr << "Failed to open navmesh file: " << filename << std::endl;
            return false;
        }
        
        // Read header
        int magic, version;
        file.read(reinterpret_cast<char*>(&magic), sizeof(int));
        file.read(reinterpret_cast<char*>(&version), sizeof(int));
        
        const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
        const int NAVMESHSET_VERSION = 1;
        
        if (magic != NAVMESHSET_MAGIC)
        {
            std::cerr << "Invalid navmesh magic number" << std::endl;
            return false;
        }
        
        if (version != NAVMESHSET_VERSION)
        {
            std::cerr << "Invalid navmesh version" << std::endl;
            return false;
        }
        
        int numTiles;
        file.read(reinterpret_cast<char*>(&numTiles), sizeof(int));
        
        if (numTiles <= 0)
        {
            std::cerr << "No tiles in navmesh" << std::endl;
            return false;
        }
        
        // Read navmesh params
        dtNavMeshParams params;
        file.read(reinterpret_cast<char*>(&params), sizeof(dtNavMeshParams));
        
        // Create navmesh
        m_navMesh = dtAllocNavMesh();
        if (!m_navMesh)
        {
            std::cerr << "Failed to allocate navmesh" << std::endl;
            return false;
        }
        
        dtStatus status = m_navMesh->init(&params);
        if (dtStatusFailed(status))
        {
            std::cerr << "Failed to initialize navmesh" << std::endl;
            return false;
        }
        
        // Read tiles
        int tilesLoaded = 0;
        for (int i = 0; i < numTiles; ++i)
        {
            dtTileRef tileRef;
            int tileDataSize;
            file.read(reinterpret_cast<char*>(&tileRef), sizeof(dtTileRef));
            file.read(reinterpret_cast<char*>(&tileDataSize), sizeof(int));
            
            if (tileDataSize <= 0) continue;
            
            unsigned char* tileData = (unsigned char*)dtAlloc(tileDataSize, DT_ALLOC_PERM);
            if (!tileData)
            {
                std::cerr << "Failed to allocate tile data" << std::endl;
                continue;
            }
            
            file.read(reinterpret_cast<char*>(tileData), tileDataSize);
            
            status = m_navMesh->addTile(tileData, tileDataSize, DT_TILE_FREE_DATA, tileRef, nullptr);
            if (dtStatusFailed(status))
            {
                std::cerr << "Failed to add tile " << i << std::endl;
                dtFree(tileData);
            }
            else
            {
                tilesLoaded++;
            }
        }
        
        std::cout << "NavMesh loaded successfully:" << std::endl;
        std::cout << "  Tiles loaded: " << tilesLoaded << "/" << numTiles << std::endl;
        std::cout << "  Max tiles: " << params.maxTiles << std::endl;
        std::cout << "  Max polys per tile: " << params.maxPolys << std::endl;
        std::cout << "  Tile size: " << params.tileWidth << " x " << params.tileHeight << std::endl;
        
        // Create query
        m_navQuery = dtAllocNavMeshQuery();
        if (!m_navQuery)
        {
            std::cerr << "Failed to allocate navmesh query" << std::endl;
            return false;
        }
        
        status = m_navQuery->init(m_navMesh, 2048);
        if (dtStatusFailed(status))
        {
            std::cerr << "Failed to initialize navmesh query" << std::endl;
            return false;
        }
        
        return true;
    }
    
    bool findNearestPoly(const float* pos, float* nearestPt, dtPolyRef* nearestRef)
    {
        if (!m_navQuery) return false;
        
        const float extents[3] = {5.0f, 5.0f, 5.0f};
        dtQueryFilter filter;
        filter.setIncludeFlags(0xffff);
        filter.setExcludeFlags(0);
        
        dtStatus status = m_navQuery->findNearestPoly(pos, extents, &filter, nearestRef, nearestPt);
        return dtStatusSucceed(status) && *nearestRef != 0;
    }
    
    bool findPath(const float* startPos, const float* endPos, 
                 std::vector<float>& pathPoints, int maxPoints = 256)
    {
        if (!m_navQuery) return false;
        
        dtPolyRef startRef, endRef;
        float nearestStart[3], nearestEnd[3];
        
        if (!findNearestPoly(startPos, nearestStart, &startRef))
        {
            std::cerr << "Could not find start polygon" << std::endl;
            return false;
        }
        
        if (!findNearestPoly(endPos, nearestEnd, &endRef))
        {
            std::cerr << "Could not find end polygon" << std::endl;
            return false;
        }
        
        dtQueryFilter filter;
        filter.setIncludeFlags(0xffff);
        filter.setExcludeFlags(0);
        
        std::vector<dtPolyRef> polys(maxPoints);
        int polyCount = 0;
        
        dtStatus status = m_navQuery->findPath(startRef, endRef, nearestStart, nearestEnd,
                                              &filter, polys.data(), &polyCount, maxPoints);
        
        if (dtStatusFailed(status) || polyCount == 0)
        {
            std::cerr << "Could not find path" << std::endl;
            return false;
        }
        
        // Get path points
        std::vector<float> straightPath(maxPoints * 3);
        std::vector<unsigned char> straightPathFlags(maxPoints);
        std::vector<dtPolyRef> straightPathPolys(maxPoints);
        int straightPathCount = 0;
        
        status = m_navQuery->findStraightPath(nearestStart, nearestEnd, polys.data(), polyCount,
                                             straightPath.data(), straightPathFlags.data(),
                                             straightPathPolys.data(), &straightPathCount, maxPoints);
        
        if (dtStatusFailed(status))
        {
            std::cerr << "Could not find straight path" << std::endl;
            return false;
        }
        
        pathPoints.clear();
        pathPoints.reserve(straightPathCount * 3);
        for (int i = 0; i < straightPathCount * 3; ++i)
        {
            pathPoints.push_back(straightPath[i]);
        }
        
        std::cout << "Path found with " << straightPathCount << " points" << std::endl;
        return true;
    }
    
    void printStats()
    {
        if (!m_navMesh)
        {
            std::cout << "No navmesh loaded" << std::endl;
            return;
        }
        
        int totalPolys = 0;
        int totalVerts = 0;
        int activeTiles = 0;
        
        // Use the navmesh parameters to determine tile grid
        const dtNavMeshParams* params = m_navMesh->getParams();
        int maxTilesX = (int)ceilf(8192.0f / params->tileWidth);  // Estimate based on common world size
        int maxTilesY = (int)ceilf(8192.0f / params->tileHeight);
        
        // Count tiles by checking tile references
        for (int y = 0; y < maxTilesY; ++y)
        {
            for (int x = 0; x < maxTilesX; ++x)
            {
                dtTileRef tileRef = m_navMesh->getTileRefAt(x, y, 0);
                if (tileRef != 0)
                {
                    const dtMeshTile* tile = nullptr;
                    if (dtStatusSucceed(m_navMesh->getTileAndPolyByRef(tileRef, &tile, nullptr)))
                    {
                        if (tile && tile->header)
                        {
                            activeTiles++;
                            totalPolys += tile->header->polyCount;
                            totalVerts += tile->header->vertCount;
                        }
                    }
                }
            }
        }
        
        std::cout << "\nNavMesh Statistics:" << std::endl;
        std::cout << "  Active tiles: " << activeTiles << std::endl;
        std::cout << "  Total polygons: " << totalPolys << std::endl;
        std::cout << "  Total vertices: " << totalVerts << std::endl;
        std::cout << "  Max tiles: " << m_navMesh->getMaxTiles() << std::endl;
    }
    
    dtNavMesh* getNavMesh() { return m_navMesh; }
    dtNavMeshQuery* getNavQuery() { return m_navQuery; }
};

void runPathfindingTest(NavMeshLoader& loader)
{
    std::cout << "\n=== Pathfinding Test ===" << std::endl;
    
    // Test points (you'll need to adjust these based on your mesh)
    float startPos[3] = {0.0f, 0.0f, 0.0f};
    float endPos[3] = {100.0f, 0.0f, 100.0f};
    
    std::cout << "Testing path from (" << startPos[0] << ", " << startPos[1] << ", " << startPos[2] << ")" 
              << " to (" << endPos[0] << ", " << endPos[1] << ", " << endPos[2] << ")" << std::endl;
    
    std::vector<float> pathPoints;
    if (loader.findPath(startPos, endPos, pathPoints))
    {
        std::cout << "Path found! Points:" << std::endl;
        for (size_t i = 0; i < pathPoints.size(); i += 3)
        {
            std::cout << "  " << (i/3) << ": (" << pathPoints[i] << ", " 
                      << pathPoints[i+1] << ", " << pathPoints[i+2] << ")" << std::endl;
        }
    }
    else
    {
        std::cout << "No path found between test points." << std::endl;
        std::cout << "Try adjusting the test coordinates to points within your mesh bounds." << std::endl;
    }
}

int main(int argc, char* argv[])
{
    std::cout << "NavMesh Loader and Tester v1.0" << std::endl;
    std::cout << "===============================" << std::endl;
    
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <navmesh.bin> [test]" << std::endl;
        std::cout << "\nLoad and inspect a navmesh file." << std::endl;
        std::cout << "Add 'test' parameter to run pathfinding tests." << std::endl;
        return 1;
    }
    
    std::string filename = argv[1];
    bool runTest = (argc > 2 && std::string(argv[2]) == "test");
    
    NavMeshLoader loader;
    
    std::cout << "Loading navmesh: " << filename << std::endl;
    if (!loader.loadNavMesh(filename))
    {
        std::cerr << "Failed to load navmesh" << std::endl;
        return 1;
    }
    
    loader.printStats();
    
    if (runTest)
    {
        runPathfindingTest(loader);
    }
    
    std::cout << "\nNavMesh loaded successfully!" << std::endl;
    return 0;
}