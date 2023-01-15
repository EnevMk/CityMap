#include "../headers/Map.hpp"
#include "../headers/Path.hpp"

Vertex::Vertex() {}

Vertex::Vertex(unsigned citiesCount) : neighbors(vector<unsigned>(citiesCount, 0)) {}

Vertex::Vertex(const std::shared_ptr<const std::string> sharedName, std::vector<unsigned> &&neighbors, unsigned outDegree)
{
    this->name = sharedName;
    this->neighbors = std::move(neighbors);
    this->outDegree = outDegree;
}

const string &Vertex::getName() const
{
    return *name;
}
/* unsigned Vertex::getID() const {
    return this->id;
} */
const vector<unsigned> &Vertex::getNeighbors() const
{
    return this->neighbors;
}

unsigned Vertex::getOutDegree() const {
    return outDegree;
}

unsigned Vertex::getInDegree() const {
    return inDegree;
}

unsigned &Vertex::operator[](size_t i)
{
    return this->neighbors[i];
}

const unsigned &Vertex::operator[](size_t i) const
{
    return this->neighbors[i];
}

/* void Vertex::setName(const string& name) {
    this->name = name;
} */
/* void Vertex::setID(unsigned id) {
    this->id = id;
} */

void Vertex::setNeighbors(std::vector<unsigned> &&neighbors)
{
    this->neighbors = std::move(neighbors);
}

Map::Map(vector<Vertex> &&adjacencyMatrix,
         const Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator> &nameToID)
//: adjacencyMatrix(adjacencyMatrix), nameToID(std::move(nameToID))

{
    this->adjacencyMatrix = std::move(adjacencyMatrix);
    this->nameToID = nameToID;
}

Map::Map(const Map &obj) : nameToID(obj.nameToID), adjacencyMatrix(obj.adjacencyMatrix)
{
    std::cout << "copy c\n";
}

const vector<Vertex> &Map::getAdjacencyMatrix() const
{
    return adjacencyMatrix;
}

const Hashmap<std::shared_ptr<const string>, unsigned, MyHashFunction, MyEqualityOperator>&
Map::getNameToIDMap() const {
    return nameToID;
}

void Map::print() const
{

    std::cout << adjacencyMatrix.size() << ' ' << nameToID.size() << '\n';
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i)
    {
        // std::cout << "try/\n";
        std::cout << adjacencyMatrix[i].getName() << " | ";
        auto neighborsRef = adjacencyMatrix[i].getNeighbors();
        // std::cout << neighborsRef.size() << ' ';
        for (int neighborID = 0; neighborID < neighborsRef.size(); ++neighborID)
        {

            if (neighborsRef[neighborID] > 0)
            {
                std::cout << adjacencyMatrix[neighborID].getName() << ' '
                          << neighborsRef[neighborID];
            }
        }
        std::cout << '\n';
    }
}

bool Map::dfs(std::vector<bool> &visited, unsigned startID, unsigned endID)
{
    visited[startID] = true;

    if (startID == endID)
        return true;

    auto startVertexNeighbors = adjacencyMatrix[startID].getNeighbors();

    for (size_t i = 0; i < startVertexNeighbors.size(); ++i)
    {

        if (startVertexNeighbors[i] > 0 && !visited[i])
        {
            return dfs(visited, i, endID);
        }
    }

    // visited[startID] = false;
    return false;
}

size_t Map::citiesCount() const
{
    return this->adjacencyMatrix.size();
}

bool Map::findPath(const string &startCity, const string &endCity)
{
    auto startCityIter = this->nameToID.find(startCity),
         endCityIter = this->nameToID.find(endCity);

    if (startCityIter == nameToID.end())
        throw invalid_argument(startCity + " not found on map!");

    if (endCityIter == nameToID.end())
        throw invalid_argument(endCity + " not found on map!");

    // bool *visited = new bool[citiesCount()];
    std::vector<bool> visited(citiesCount(), false);
    return dfs(visited, startCityIter->second, endCityIter->second);
}

bool Map::isVisited(unsigned id, const vector<int> &listVisited) const
{
    return listVisited[id] != -1;
}

/* unsigned Map::getNextUnvisitedID(vertexPriorityQueue &queue, const listVisitedVertices &visited) const
{
    auto currentVertexID = queue.top().first;

    while (isVisited(currentVertexID, visited))
    {
        queue.pop();
        currentVertexID = queue.top().first;
    }
    return currentVertexID;
} */

bool Map::existsPath(const std::vector<unsigned> &neighbors, unsigned targetID) const
{
    return neighbors[targetID] > 0;
}

unsigned Map::getCityID(const string &name) const
{
    auto iterator = this->nameToID.find(name);

    if (iterator == nameToID.end())
        throw invalid_argument(name + " not found on map!");

    return iterator->second;
}

Path Map::shortestPath(const string &startCityName, const string &endCityName)
{

    auto source = getCityID(startCityName),
         target = getCityID(endCityName);

    Path result = targetedDijkstra(source, target);

    if (result.length == 0) {
        throw range_error("No path found!");
    }
    return result;
}

Path Map::targetedDijkstra(unsigned sourceID, unsigned targetID) const {
    auto result = dijkstraSPT(sourceID, targetID, [targetID](unsigned id) { return id == targetID; });

    const std::vector<int> parents = result.first,
                           visited = result.second;

    vector<unsigned> verticesPath;
    getPath(parents, targetID, verticesPath);
    return Path{verticesPath, visited[targetID]};
}

/* Path */pair<vector<int>, vector<int>> Map::dijkstraSPT(unsigned sourceID, unsigned targetID, std::function<bool(unsigned)> shouldStopAlgorithm) const {
    vector<int> parents(this->adjacencyMatrix.size(), -1); // parents[i] holds the parent of vertex with id i
    parents[sourceID] = -1;
    
    vertexPriorityQueue minPriorityQueue;
    std::vector<int> visited(this->adjacencyMatrix.size(), -1);
    
    visited[sourceID] = 0;
    minPriorityQueue.push(make_pair(sourceID, 0));
    unsigned currentVertexID = sourceID;

    while (!minPriorityQueue.empty() && !shouldStopAlgorithm(currentVertexID)) {
        currentVertexID = minPriorityQueue.top().first;
        std::cout << currentVertexID << '\n';
        minPriorityQueue.pop();

        auto neighbors = adjacencyMatrix[currentVertexID].getNeighbors();
        for (unsigned id = 0; id < neighbors.size(); ++id) {

            if (existsPath(neighbors, id) && (!isVisited(id, visited) || neighbors[id] + visited[currentVertexID] < visited[id])) {
                visited[id] = visited[currentVertexID] + neighbors[id];
                parents[id] = currentVertexID;
                minPriorityQueue.push(make_pair(id, visited[id]));
            }
        }
    }
    return make_pair(parents, visited);
}

void Map::getPath(const vector<int> &parents, unsigned current, vector<unsigned> &result) const
{
    if (parents[current] == -1) {
        result.push_back(current);
        return;
    }
    getPath(parents, parents[current], result);
    result.push_back(current);
}

template <class Predicate>
Path Map::getShortestDeviationAt(unsigned ithVertex, const Path &path, const unordered_set<unsigned>& verticesToAvoid, Predicate predicate) const {    
    Path toReturn = path.getSubPathUntil(ithVertex, *this);

    auto vertexID = path.at(ithVertex - 1);
    auto neighbors = adjacencyMatrix[vertexID].getNeighbors();
    
    pathsPriorityQueue minPriorityQueue;
    for (unsigned id = 0; id < neighbors.size(); ++id) {
        auto neighborName = adjacencyMatrix[id].getName();

        if (neighbors[id] > 0 && verticesToAvoid.find(id) == verticesToAvoid.end() && predicate(neighborName)) {   
            minPriorityQueue.push(targetedDijkstra(id, path.destination()));
        }
    }

    if (minPriorityQueue.empty())
        throw std::invalid_argument("No alternative path available");
    
    toReturn.append(minPriorityQueue.top(), *this);
    return toReturn;
}

template <class VerticesCriteria>
std::vector<Path> Map::getFirstKShortestPaths(unsigned sourceID, unsigned targetID, unsigned k, VerticesCriteria predicate) const {
    Path shortestPath = targetedDijkstra(sourceID, targetID);
    pathsPriorityQueue minPriorityQueue;

    vector<Path> toReturn{shortestPath};
    while (toReturn.size() < k) {
        const Path &basePath = toReturn.back();
        
        for (unsigned vertex = 1; vertex < basePath.size(); ++vertex) {  
            auto subPath = basePath.getSubPathUntil(vertex, *this);
            auto verticesToAvoid = getVerticesToAvoidAt(vertex, toReturn, subPath);

            try {
                minPriorityQueue.push(getShortestDeviationAt(vertex, basePath, verticesToAvoid, predicate));
            }
            catch (...) {}
        }

        if (minPriorityQueue.empty())
            throw std::invalid_argument("No alternative paths available");

        toReturn.push_back(minPriorityQueue.top());
        minPriorityQueue.pop();
    }
    return toReturn;
}

std::unordered_set<unsigned> Map::getVerticesToAvoidAt(unsigned levelK, const std::vector<Path>& bestPaths, const Path& p) const {

    std::unordered_set<unsigned> toReturn;

    for (size_t i = 0; i < bestPaths.size(); ++i) {
        //toReturn.insert(bestPaths[i][levelK]);
        if (p.equalUntil(p.size() - 1, bestPaths[i])) {
            toReturn.insert(bestPaths[i][levelK]);
        }
    }

    for (auto i = 0; i < p.size(); ++i) {
        toReturn.insert(p[i]);
    }
    return toReturn;
}

std::vector<Path> Map::getFirstKShortestPaths(const string &startCity, const string &endCity, unsigned k) const
{
    auto source = getCityID(startCity),
         target = getCityID(endCity);

    return getFirstKShortestPaths(source, target, k, [] (const string&) { return true; });
}

std::vector<Path> Map::getFirstKShortestPaths(const string& startCity, const string& endCity, unsigned k,
                        const std::unordered_set<string>& closedVertices) const
{
    auto source = getCityID(startCity),
         target = getCityID(endCity);

    struct ClosedVertices {
        const std::unordered_set<string>& setReference;

        ClosedVertices(const std::unordered_set<string>& set): setReference(set) {}

        bool operator()(const string& cityName) const {
            return setReference.find(cityName) == setReference.end();
        }
    };
    return getFirstKShortestPaths(source, target, k, ClosedVertices(closedVertices));
}

Path Map::findCycle(const string& startCity) const {
    auto source = getCityID(startCity);
    auto neighbors = adjacencyMatrix[source].getNeighbors();

    Path toReturn = Path{vector<unsigned>{source}};

    for (size_t id = 0; id < neighbors.size(); ++id) {
        
        if (existsPath(neighbors, id)) {
            Path path = targetedDijkstra(id, source);

            if (path.length != 0) {
                toReturn.append(path, *this);
                return toReturn;
            }
        }
    }
    throw invalid_argument("Couldn't find a cycle from " + startCity + " to itself!");
}

bool Map::canVisitAllVerticesFrom(const string& city) const {
    auto source = getCityID(city);

    auto result = dijkstraSPT(source, source, [](int) { return false; });

    const std::vector<int>& visited = result.second;

    bool allVisited = true;
    for (size_t i = 0; i < visited.size() && allVisited; ++i) {
        if (visited[i] == -1) {
            allVisited = false;
        }
    }
    return allVisited;
}

vector<unsigned> Map::getDeadEndVertices() const {

    vector<unsigned> toReturn;
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {

        if (adjacencyMatrix[i].getOutDegree() == 0) {
            toReturn.push_back(i);
        }
    }

    return toReturn;
}

vector<Path> Map::findAllDeadEndStreets() const {
    auto deadEnds = getDeadEndVertices();
    vector<Path> toReturn;
    for (unsigned deadEnd = 0; deadEnd < deadEnds.size(); ++deadEnd) {

        for (unsigned source = 0; source < adjacencyMatrix.size(); ++source) {

            if (adjacencyMatrix[source][deadEnd] > 0) {
                vector<unsigned> street{source, deadEnd};
                toReturn.push_back(Path{street, (int)adjacencyMatrix[source][deadEnd]});
            }
        }
    }
    return toReturn;
}

unsigned Map::getInDegree(const string& name) const {
    unsigned id = getCityID(name);

    return adjacencyMatrix[id].getInDegree();
}

unsigned Map::getOutDegree(const string& name) const {
    unsigned id = getCityID(name);

    return adjacencyMatrix[id].getOutDegree();
}

bool Map::isEulerianPathPossible() const {

    int startVertex = 0,
         endVertex = 0;
    for (size_t i = 0; i < adjacencyMatrix.size() && startVertex <= 1 && endVertex <= 1; ++i) {

        int inDeg = adjacencyMatrix[i].getInDegree(),
            outDeg = adjacencyMatrix[i].getOutDegree();

        if (abs(inDeg - outDeg) > 1) {
            return false;
        }
        else if (inDeg - outDeg == 1) {
            startVertex++;
        }
        else if (outDeg - inDeg == 1) {
            endVertex++;
        }
    }
    return endVertex == 0 && startVertex == 0 || endVertex == 1 && startVertex == 1;
}

unsigned Map::findStartNode() const {
    
    unsigned toReturn;
    for (size_t id = 0; id < adjacencyMatrix.size(); ++id) {
        
        if (adjacencyMatrix[id].getOutDegree() - adjacencyMatrix[id].getInDegree() == 1) return id;

        if (adjacencyMatrix[id].getOutDegree() > 0) toReturn = id;
    }
    return toReturn;
}

void Map::dfsEulerPath(std::unordered_set<string>& traversedEdges, Path& eulerPath, unsigned at, unsigned edgePrice) const {

    auto neighbors = adjacencyMatrix[at].getNeighbors();

    for (size_t id = 0; id < neighbors.size(); ++id) {

        if (!existsPath(neighbors, id)) continue;

        string edgeString = to_string(at) + to_string(id);

        if (traversedEdges.find(edgeString) == traversedEdges.end()) {
            traversedEdges.insert(edgeString);
            dfsEulerPath(traversedEdges, eulerPath, id, neighbors[id]);    
        }
    }
    eulerPath.pushVertex(at, edgePrice);
}

Path Map::getEulerPath() const {

    if (isEulerianPathPossible()) {
        unsigned start = findStartNode();

        std::cout << start << '\n';

        std::unordered_set<string> traversedEdges;
        Path toReturn;
        dfsEulerPath(traversedEdges, toReturn, start, 0);

        std::reverse(toReturn.vertices.begin(), toReturn.vertices.end());
        return toReturn;
    }

    throw invalid_argument("No Euler Path possible on this graph!");
}