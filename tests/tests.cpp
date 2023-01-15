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
        fs::path path2 = "maps/map2.txt";
        

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

            Map bigMap = reader.readMap(path2);
            auto bigAdjMatrix = bigMap.getAdjacencyMatrix();

            THEN("ID of vertex labeled Prague is 0") {
                REQUIRE(bigAdjMatrix[0].getName() == "Prague");
            }

            THEN("ID of vertex labeled Sofia is 3") {
                REQUIRE(bigAdjMatrix[3].getName() == "Sofia");
            }

            THEN("Distance from Prague to Sofia is 1066") {
                REQUIRE(bigAdjMatrix[3][0] == 1066);
            }

        }
    }
}

SCENARIO("Test a graph for cycle") {
    MapReader reader;
    fs::path path = "maps/map.txt";

    Map map = reader.readMap(path);
    WHEN("Given this graph") {
        

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



TEST_CASE("Pathfinding") {

    MapReader reader;
    fs::path path = "maps/map2.txt";    
    Map map = reader.readMap(path);

    SECTION("Test pathfinding") {
        REQUIRE(map.findPath("Prague", "Beijing") == true);
    }

    SECTION("Test shortest path finding") {

        Path expectedPath{vector<unsigned>{0, 3, 8, 4}, 11452};
        REQUIRE(equalPaths(map.shortestPath("Prague",  "Beijing"), expectedPath) == true);
    }
}

