#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
//#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <queue>


#include "utils.hpp"
#include "data_structures/Hashmap.hpp"
//#include "data_structures/Hashset.hpp"

//#include "Path.hpp"

using std::vector;
using std::string;
using std::pair;
using std::shared_ptr;

class Path;


class Vertex {

    std::shared_ptr<const string> name;
    //unsigned citiesCount;

    vector<unsigned> neighbors;

public:
    Vertex();
    Vertex(const std::shared_ptr<const std::string> sharedName, std::vector<unsigned>&& neighbors);
    Vertex(unsigned citiesCount);
    const string& getName() const;
    void setNeighbors(std::vector<unsigned>&& neighbors);
    //unsigned getID() const;
    const vector<unsigned>& getNeighbors() const;

    void setName(const string& name);

    unsigned& operator[](size_t i);
    const unsigned& operator[](size_t i) const;

    friend void nodeToDotty(std::ofstream& os, const Map& map);
    //void setID(unsigned id);

    //void addNeighbor(unsigned id, double dist);

};

class Map {

    Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator> nameToID;

    //std::unordered_map<unsigned, Vertex> map;

    vector<Vertex> adjacencyMatrix;

    using vertexPriorityQueue = priority_queue<pair<unsigned, int>, vector<pair<unsigned, int>>, HeapCompareGreater>;
    using listVisitedVertices = std::vector<int>;

    bool dfs(std::vector<bool>& visited, unsigned startID, unsigned endID);
    bool isVisited(unsigned id, const vector<int>& listVisited) const;
    unsigned getNextUnvisitedID(vertexPriorityQueue& queue, const listVisitedVertices& visited) const;
    bool existsPath(const std::vector<unsigned>& neighbors, unsigned targetID) const;

    /* pair<string, unsigned> */Path dijkstraSPT(unsigned sourceID, unsigned targetID) const;
    //std::vector<Path> getFirstKShortestPaths()
    
    using pathsPriorityQueue = priority_queue<Path, vector<Path>, pathCompare>;
    
    template <class Predicate>
    Path getShortestDeviationAt(unsigned ithVertex, const Path &path, const unordered_set<unsigned>& verticesToAvoid, Predicate predicate) const;

    unsigned getCityID(const string& name) const;

    Path intersectPaths(const vector<Path>& paths) const;

    std::unordered_set<unsigned> getVerticesToAvoidAt(unsigned levelK, const std::vector<Path>& bestPaths, const Path& p) const;

public:
    
    size_t citiesCount() const;
    Map(vector<Vertex>&& adjacencyMatrix, 
        const Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator>& nameToID);
    Map(const Map& obj);
    void print() const;

    
    bool findPath(const string& startCity, const string& endCity);

    
    Path shortestPath(const string& startCity, const string& endCity);

    template <class VerticesCriteria>
    std::vector<Path> getFirstKShortestPaths(unsigned sourceID, unsigned targetID, unsigned k, VerticesCriteria) const;

    std::vector<Path> getFirstKShortestPaths(const string& startCity, const string& endCity, unsigned k) const;
    std::vector<Path> getFirstKShortestPaths(const string& startCity, const string& endCity, unsigned k,
                        const std::unordered_set<string>& closedVertices) const;

    const vector<Vertex>& getAdjacencyMatrix() const;
    void getPath(const vector<int>& parents, unsigned current, vector<unsigned>& result) const;

    const Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator>& getNameToIDMap() const;
};