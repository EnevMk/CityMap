#pragma once
#include <functional>
#include <memory>
#include <functional>
#include <utility>

#include "../headers/Path.hpp"

using namespace std;

struct MyEqualityOperator /* : binary_function<string, shared_ptr<string>, bool> */ {

    using is_transparent = void;

    bool operator()(const shared_ptr<const string>& sharedValue0, const shared_ptr<const string>& sharedValue1) const {
        return *sharedValue0 == *sharedValue1;
    }

    bool operator()(const string& value, const shared_ptr<const string>& sharedValue) const {       
        return value == *sharedValue;
    }

    bool operator()(const shared_ptr<const string>& sharedValue, const string& value) const {
        return value == *sharedValue;
    }
    
};

struct MyHashFunction : std::unary_function<shared_ptr<const string>, size_t> {
    
    using is_transparent = void;
    //using transparent_key_equal = MyEqualityOperator;
    size_t operator()(const string& value) const {
        //std::cout << " - string version\n";
        return std::hash<string>()(value);
    }

    size_t operator()(const shared_ptr<const string>& value) const {
        //std::cout << " - shared ptr version\n";
        return std::hash<string>()(*value);
    }
};

struct HeapCompareGreater {
    bool operator()(const pair<unsigned, unsigned>& p1, const pair<unsigned, unsigned>& p2) const {
        return p1.second > p2.second;
    }
};

struct pathCompare {
    bool operator()(const Path &p1, const Path &p2) {
        return p1.length > p2.length;
    }
};

/* using Edge = std::pair<unsigned, unsigned>;

struct EdgeHash {
    size_t operator()(const Edge& edge) const {
        return std::hash<unsigned>()(edge.first) + std::hash<unsigned>()(edge.second);
    }
}; */

//auto pathCompare = [](const Path &p1, const Path &p2) constexpr { return p1.length > p2.length; } ;
