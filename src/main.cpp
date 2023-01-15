#include <iostream>
#include "../headers/MapReader.hpp"
#include "../data_structures/Hashmap.hpp"
#include "../utilities/ToDotty.hpp"
#include "../utilities/utils.hpp"

#include <iomanip>
#include <memory>
#include <unordered_map>
#include <set>
#include <version>

//#define __cpp_lib_generic_associative_lookup ;

int main() {



    //Hashmap<shared_ptr<string>, int, MyHashFunction, MyEqualityOperator> h;

    //static_assert(has_type_member<std::less<>, std::less<>::is_transparent>()(2) == false);

    //std::set<shared_ptr<const string>, MyHashFunction, MyEqualityOperator> s;
    //std::cout << has_type_member<MyHashFunction>()() << '\n';
    //Hashmap<int, string, std::hash<int>, std::equal_to<int>> h;

    /* auto shared = make_shared<string>("НДК"),
         share2 = make_shared<string>("БСФС");
    
    h.insert(shared, 4);
    h.insert(share2, 5);

    std::cout << h.find("НДК")->second << '\n'; */

    //std::cout << h[make_shared<string>("milan")];
//    std::cout << h.find() << '\n';
    /* std::unordered_map<shared_ptr<const string>, unsigned, MyHashFunction> map;
    

    shared_ptr<const string> shp = std::make_shared<string>("Name"),
                             shp2 = std::make_shared<string>("Family");

    std::cout << shp.use_count() << '\n';
    map.insert(make_pair(shp, 34));
    map.insert(make_pair(shp2, 11));
    
    std::cout << shp.use_count() << '\n';
    std::cout << MyEqualityOperator()("Name", shp) << '\n';
    std::cout << map.find("Name"); */

    MapReader r;
    fs::path path = "maps/map2.txt";
    auto map = r.readMap(path);

    //map.print();
    /* auto paths = map.getFirstKShortestPaths("Prague", "Beijing", 3);

    for (int i = 0; i < paths.size(); ++i) {
        paths[i].print(map);
    } */
    /* auto cycle = map.findCycle("Tokyo");

    cycle.print(map); */

    //std::cout << map.canVisitAllVerticesFrom("Prague") << '\n';

    /* auto deadEnds = map.findAllDeadEndStreets();
    for (const Path& p : deadEnds) {
        p.print(map);
    } */

    /* std::ofstream os("map2.dot");
    toDotty(os, map);
    os.close(); */

    //std::cout << map.getInDegree("Попа") << ' ' << map.getOutDegree("Попа") << '\n';
    auto c = map.shortestPath("Prague", "Beijing");
    c.print(map);
    /* auto ep = map.getEulerPath();

    ep.print(map); */
    return 0;
}