#include "Path.hpp"
#include "Map.hpp"

const std::vector<unsigned>& Path::getVertices() const {
    return vertices;
}

void Path::pushVertex(unsigned id, unsigned price) {
    vertices.push_back(id);
    length += price;
}

const unsigned& Path::at(size_t i) const {
    return vertices.at(i);
}

const unsigned& Path::operator[](size_t i) const {
    return vertices[i];
}

const unsigned& Path::destination() const {
    return vertices.back();
}

void Path::append(const Path& path, const Map& map) {
    unsigned previous = destination();
    for (size_t i = 0; i < path.size(); ++i) {
        vertices.push_back(path[i]);
        length += map.getAdjacencyMatrix()[previous][path[i]];
        previous = path[i];
    }
}

size_t Path::size() const {
    return vertices.size();
}

void Path::print() const {

    for (size_t i = 0; i < vertices.size(); ++i) {
        std::cout << vertices[i] << "->";
    }
    std::cout << "END | LENGTH " << this->length << '\n';
}

void Path::print(const Map& map) const {
    for (size_t i = 0; i < vertices.size(); ++i) {
        std::cout << map.getAdjacencyMatrix()[vertices[i]].getName() << "->";
    }
    std::cout << "END | LENGTH " << this->length << '\n';
}

bool Path::equalUntil(size_t k, const Path& obj) const {

    bool flag = true;

    for (size_t i = 0; i < k && flag; ++i) {

        if (vertices[i] != obj.vertices[i]) {
            flag = false;
        }
    }
    return flag;
}

Path Path::getSubPathUntil(size_t k, const Map& map) const {

    Path toReturn;

    if (k == 0) return toReturn;

    toReturn.pushVertex(vertices[0],0);

    auto adjacencyMatrix = map.getAdjacencyMatrix();
    unsigned previous = vertices[0];
    for (size_t i = 1; i < k && i < this->size(); ++i) {
        toReturn.pushVertex(vertices[i], adjacencyMatrix[previous][vertices[i]]);
        previous = vertices[i];
    }
    return toReturn;
}