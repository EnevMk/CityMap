#pragma once

#include <filesystem>
#include <exception>
#include <unordered_map>

#include "Map.hpp"

namespace fs = std::filesystem;

class MapReader {

    friend class Vertex;
    void updateInDegrees(vector<Vertex>& adjacencyMatrix) const;
    
public:
    Map readMap(const fs::path& filePath) const;
    
};