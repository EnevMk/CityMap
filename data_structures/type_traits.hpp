#pragma once
#include <type_traits>

template <class, class = void>
struct has_type_member /* : std::false_type */ {

    enum {
        value = false,
    };

    bool operator()() {
        std::cout << "not here\n";
        return false;
    }
 };

template <class T>
struct has_type_member<T, std::void_t<typename T::is_transparent>> /* : std::true_type */ {
    enum {
        value = true,
    };
    bool operator()() {
        std::cout << "here\n";
        return true;
    }
};
