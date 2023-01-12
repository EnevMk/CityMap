#pragma once

#include <filesystem>
#include <exception>
#include <unordered_map>

#include "Map.hpp"

namespace fs = std::filesystem;

class MapReader {

public:
    Map readMap(const fs::path& filePath);

};