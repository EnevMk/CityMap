#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_set>
#include <memory>
#include <queue>
#include <cmath>


#include "../utilities/utils.hpp"
#include "../data_structures/Hashmap.hpp"

using std::vector;
using std::string;
using std::pair;
using std::shared_ptr;

class Path;


class Vertex {

    std::shared_ptr<const string> name;

    vector<unsigned> neighbors;

    unsigned outDegree = 0;
    unsigned inDegree = 0;

    friend class MapReader;
public:
    Vertex();
    Vertex(const std::shared_ptr<const std::string> sharedName, std::vector<unsigned>&& neighbors, unsigned outDegree);
    Vertex(unsigned citiesCount);
    const string& getName() const;
    unsigned getOutDegree() const;
    unsigned getInDegree() const;
    void setNeighbors(std::vector<unsigned>&& neighbors);
    //unsigned getID() const;
    const vector<unsigned>& getNeighbors() const;

    void setName(const string& name);

    unsigned& operator[](size_t i);
    const unsigned& operator[](size_t i) const;

};

class Map {

private:
    Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator> nameToID;

    vector<Vertex> adjacencyMatrix;

private:
    //using vertexPriorityQueue = priority_queue<pair<unsigned, int>, vector<pair<unsigned, int>>, HeapCompareGreater>;
    using listVisitedVertices = std::vector<int>;

    bool dfs(std::vector<bool>& visited, unsigned startID, unsigned endID);
    bool isVisited(unsigned id, const vector<int>& listVisited) const;
    //unsigned getNextUnvisitedID(vertexPriorityQueue& queue, const listVisitedVertices& visited) const;

    Path targetedDijkstra(unsigned sourceID, unsigned targetID) const;

    pair<vector<int>, vector<int>> dijkstraSPT(unsigned sourceID, unsigned targetID, std::function<bool(unsigned)> stopAlgorithm) const;
    
    using pathsPriorityQueue = priority_queue<Path, vector<Path>, pathCompare>;
    

    unsigned getCityID(const string& name) const;

    //Path intersectPaths(const vector<Path>& paths) const;
    std::unordered_set<unsigned> getVerticesToAvoidAt(unsigned levelK, const std::vector<Path>& bestPaths, const Path& p) const;

    template <class VerticesCriteria>
    std::vector<Path> getFirstKShortestPaths(unsigned sourceID, unsigned targetID, unsigned k, VerticesCriteria) const;

    template <class Predicate>
    Path getShortestDeviationAt(unsigned ithVertex, 
                                const Path &path,
                                const unordered_set<unsigned>& verticesToAvoid,
                                Predicate predicate) const;

    vector<unsigned> getDeadEndVertices() const;

    // utility method for the Eulerian path functionality
    unsigned findStartNode() const;

    void dfsEulerPath(std::unordered_set<string>& traversedEdges, Path& eulerPath, unsigned at, unsigned edgePrice) const;
public:
    bool isEulerianPathPossible() const;
    
    size_t citiesCount() const;
    Map(vector<Vertex>&& adjacencyMatrix, 
        const Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator>& nameToID);
    Map(const Map& obj);
    void print() const;

    Path findCycle(const string& startCity) const;
    bool findPath(const string& startCity, const string& endCity);
    Path shortestPath(const string& startCity, const string& endCity);
    bool canVisitAllVerticesFrom(const string& city) const;

    std::vector<Path> getFirstKShortestPaths(const string& startCity, const string& endCity, unsigned k) const;
    std::vector<Path> getFirstKShortestPaths(const string& startCity, const string& endCity, unsigned k,
                        const std::unordered_set<string>& closedVertices) const;

    Path getEulerPath() const;

    const vector<Vertex>& getAdjacencyMatrix() const;
    void getPath(const vector<int>& parents, unsigned current, vector<unsigned>& result) const;

    const Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator>& getNameToIDMap() const;

    vector<Path> findAllDeadEndStreets() const;
    
    unsigned getInDegree(const string& name) const;
    unsigned getOutDegree(const string& name) const;
    bool existsPath(const std::vector<unsigned>& neighbors, unsigned targetID) const;
};