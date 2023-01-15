#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../headers/MapReader.hpp"

bool equalPaths(const Path& a, const Path& b) {

    if (a.size() != b.size()) return false;

    bool equal = true;
    for (size_t i = 0 ; i < a.size() && equal; ++i) {
        
        if (a[i] != b[i]) equal = false;
    }
    return equal;
}

SCENARIO("MapReader class reads a valid .txt file of a Graph correctly") {

    GIVEN("A MapReader object and valid .txt file of a graph") {

        MapReader reader;
        fs::path path = "maps/map.txt";

        WHEN("Feeding the .txt to that reader") {

            Map map = reader.readMap(path);
            auto adjMatrix = map.getAdjacencyMatrix();

            THEN("Count of vertices is 4") {
                REQUIRE(map.citiesCount() == 4);
            }
            THEN("ID of vertex labeled НДК is 3") {
                REQUIRE(adjMatrix[3].getName() == "НДК");
            }
            THEN("ID of vertex labeled Попа is 0") {
                REQUIRE(adjMatrix[0].getName() == "Попа");
            }
            THEN("Edges from НДК to Попа exist") {
                REQUIRE(map.existsPath(adjMatrix[3].getNeighbors(), 0) == true);
            }
            THEN("Distance from НДК to Попа is 800") {
                REQUIRE(adjMatrix[3][0] == 800);
            }
            THEN("inDegree of Попа is 2") {
                REQUIRE(map.getInDegree("Попа") == 2);
            }
            THEN("outDegree of Попа is also 2") {
                REQUIRE(map.getOutDegree("Попа") == 2);
            }

        }
    }
}

SCENARIO("Test a graph for cycle") {
    MapReader reader;
    fs::path path = "maps/map.txt";

    WHEN("Given this graph") {
        Map map = reader.readMap(path);

        THEN("A cycle with start and end point НДК exists with length 2200") {

            //vector of the IDs of the vertices forming the path
            // names: НДК->Попа->5Кьошета->НДК
            Path expectedPath{vector<unsigned>{3, 0, 2, 3}, 2200};
            REQUIRE(equalPaths(map.findCycle("НДК"), expectedPath) == true);
        }
        /* THEN("A cycle with start and end point БСФС exists") {
            REQUIRE(map.findCycle("НДК") == true);
        } */
    }
}