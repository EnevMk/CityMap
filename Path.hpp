#pragma once

#include <vector>

class Map;

struct Path {

    std::vector<unsigned> vertices;
    int length = 0;
public:

    bool equalUntil(size_t k, const Path&obj) const;
    Path getSubPathUntil(size_t i, const Map& map) const;
    
    const std::vector<unsigned>& getVertices() const;
    void pushVertex(unsigned id, unsigned price);
    const unsigned& at(size_t i) const;
    const unsigned& operator[](size_t i) const ;
    const unsigned& destination() const;
    void append(const Path& path, const Map& map);

    size_t size() const;

    void print() const;
    void print(const Map& map) const;
};