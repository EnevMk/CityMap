#pragma once

#include <iostream>
#include <fstream>
//#include "treeutils.cpp"
#include "../headers/Map.hpp"

//#include "BinaryTree.hpp"
/* 
template <typename T>
void nodeToDotty(std::ofstream& os, typename BinaryTree<T>::node* node); */



//template <typename T>
void nodeToDotty(std::ofstream& os, const Map& map) {
    auto nameToID = map.getNameToIDMap();
    auto adjacencyMatrix = map.getAdjacencyMatrix();
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        auto name = adjacencyMatrix[i].getName();
        os << nameToID.find(name)->second << " [label=\"" << name << "\"]" << '\n';
    }
    
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {

        auto neighbors = adjacencyMatrix[i].getNeighbors();
        auto name = adjacencyMatrix[i].getName();
        auto id = nameToID.find(name)->second;

        for (size_t neighborID = 0; neighborID < neighbors.size(); ++neighborID) {
            
            if (neighbors[neighborID] > 0) {
                os << id << "->" << neighborID << 
                    "[color=\"green\", label=\"" << adjacencyMatrix[id][neighborID] << "\"]\n";
            }
        }
    }    
}

//template <typename T>
void toDotty(std::ofstream& os, const Map& map) {

    os << "digraph G {\n";
    nodeToDotty(os, map);
    os << '}';
}
