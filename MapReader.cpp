#include "MapReader.hpp"

vector<unsigned> readNeighbors(std::ifstream& is,
                   Hashmap<shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator>& nameToIDMap,
                   unsigned &idAccumulator,
                   unsigned totalNodeCount
                   //vector<vector<unsigned>>& adjacencyMatrix,
                   //size_t nodeIndex,
                   ) 

{   
    vector<unsigned> neighbors(totalNodeCount, 0);
    while (is.peek() != '\n' && !is.eof())
    {
        string neighborName;
        double distance;
        is >> neighborName >> distance;

        auto it = nameToIDMap.find(neighborName);
        unsigned neighborId;

        if (it == nameToIDMap.end()) {
            auto sharedName = make_shared<string>(neighborName);
            nameToIDMap.insert(sharedName, idAccumulator);
            neighborId = idAccumulator++;
        }
        else {
            neighborId = it->second;
        }
        neighbors[neighborId] = distance;
    }

    return neighbors;
}

Map MapReader::readMap(const fs::path &filePath) {

    std::ifstream is(filePath.string());

    if (!is.good()) {
        throw std::invalid_argument("Invalid file path for city map provided!");
    }
    unsigned id = 0;
    Hashmap<shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator> map;

    unsigned citiesCount;
    is >> citiesCount;

    vector<Vertex> adjacencyMatrix(citiesCount, Vertex(citiesCount));

    while (!is.eof())
    {
        string name;
        is >> name;
        
        auto iter = map.find(name);
        unsigned currentId;
        if (iter != map.end()) {
            currentId = iter->second;//v.setID(iter->second);
        }
        else {   
            map.insert(make_shared<string>(name), id);
            currentId = id++;
        }
        
        auto neighbors = readNeighbors(is, map, id, citiesCount);
        adjacencyMatrix[currentId] = Vertex(map.find(name)->first, std::move(neighbors));
        //result.insert(std::make_pair(v.getID(), v));
    }
    is.close();
    return Map(std::move(adjacencyMatrix), std::move(map));
}