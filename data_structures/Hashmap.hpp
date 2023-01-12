#pragma once

#include <forward_list>
#include <vector>
#include "type_traits.hpp"

//#include "utils\type_traits.hpp"

using std::is_const;
using std::enable_if;

template <typename Key, typename Value, typename Hash = std::hash<Key>, typename Compare = std::equal_to<Key>>
class Hashmap {

    typedef std::pair<Key, Value> Item;
    typedef std::forward_list<Item> Bucket;
    typedef std::vector<Bucket> Table;

    typedef typename Bucket::iterator bucket_iterator;
    typedef typename Table::iterator table_iterator;

    typedef typename Bucket::const_iterator const_bucket_iterator;
    typedef typename Table::const_iterator const_table_iterator;

    Table table;

    const float MAX_LOAD_FACTOR = 0.7f;
    int count = 0;

public:

    template <typename TableType = table_iterator, typename BucketType = bucket_iterator, typename Type = std::pair<Key, Value>>
    struct base_iterator {
    
    private:
        TableType bucket;
        BucketType element;
        TableType end;

        void findNext()  {

            while (bucket != end && element == bucket->end()) {
                ++bucket;

                if (bucket != end) {
                    element = bucket->begin();
                }
            }
        }

        base_iterator(TableType begin, BucketType element, TableType end) 
        : bucket(begin),
          element(element),
          end(end)
        {
            findNext();
        }

        friend class Hashmap;
    public:

        template <typename Q = Type>
        const typename enable_if<is_const<Q>::value, Q>::type & operator*() const {
            return reinterpret_cast<const std::pair<Key, Value>&>(*element);;
        }

        template <typename Q = Type>
        typename enable_if<!is_const<Q>::value, Q>::type & operator*() {
            return reinterpret_cast<std::pair<const Key, Value>&>(*element);
        }

        template <typename Q = Type>
        const typename enable_if<is_const<Q>::value, Q>::type * operator->() const {
            return &reinterpret_cast<const std::pair<Key, Value>&>(*element);;
        }

        template <typename Q = Type>
        typename enable_if<!is_const<Q>::value, Q>::type * operator->() {
            return &reinterpret_cast<std::pair<const Key, Value>&>(*element);
        }

        base_iterator& operator++() {
            ++element;
            findNext();
            return *this;
        }

        bool operator==(const base_iterator &other) const {
            return bucket == other.bucket && element == other.element;
        }

        bool operator!=(const base_iterator &other) const {
            return !(*this == other);
        }
    };

    typedef base_iterator<table_iterator, bucket_iterator, std::pair<const Key, Value>> iterator;
    typedef base_iterator<const_table_iterator, const_bucket_iterator, const std::pair<Key, Value>> const_iterator;


    const_iterator cbegin() const {
        return const_iterator(table.cbegin(), table.front().cbegin(), table.cend());
    }

    const_iterator cend() const {
        return const_iterator(table.cend(), table.back().cend(), table.cend());
    }

    const_iterator begin() const {
        return const_iterator(table.cbegin(), table.front().cbegin(), table.cend());
    }

    const_iterator end() const {
        return const_iterator(table.cend(), table.back().cend(), table.cend());
    }

    iterator begin() {
        return iterator(table.begin(), table.front().begin(), table.end());
    }

    iterator end() {
        return iterator(table.end(), table.back().end(), table.end());
    }

    int getIndex(const Key& key) const {

        return Hash()(key) % table.size();
    }
    
    int size() const {
        return this->count;
    }

    const_table_iterator getBucket(const Key& key) const {
        return table.cbegin() + getIndex(key);
    }

    table_iterator getBucket(const Key& key) {
        return table.begin() + getIndex(key);
    }

    float getLoadFactor() const {

        return float(count) / float(table.size());
    }

    bool shouldResize() const {
        return getLoadFactor() > MAX_LOAD_FACTOR;
    }

    void resize() {
        Table oldTable;
        oldTable.resize(size() * 2);
        std::swap(oldTable, table);

        this->count = 0;

        for (table_iterator bucket = oldTable.begin(); bucket != oldTable.end(); ++bucket) {

            for (bucket_iterator element = bucket->begin(); element != bucket->end(); ++element) {
                insert(element->first, element->second);
            }
        }
    }

    Hashmap() {
        table.resize(16);
    }

    Hashmap(const Hashmap& obj) {
        //this->MAX_LOAD_FACTOR = obj.MAX_LOAD_FACTOR;
        this->table = obj.table;
        this->count = obj.count;
        /* this->table.resize(size());

        for (auto bucket = obj.table.cbegin(); bucket != obj.table.cend(); ++bucket) {

            for (auto element = bucket->cbegin(); element != bucket->cend(); ++element) {
                insert(element->first, element->second);
            }
        } */
    }

    Hashmap& operator=(const Hashmap& obj) {
        
        if (this != &obj) {
            /* his->table.resize(size());

            for (auto bucket = obj.table.cbegin(); bucket != obj.table.cend(); ++bucket) {

                for (auto element = bucket->cbegin(); element != bucket->cend(); ++element) {
                    insert(element->first, element->second);
                }
            } */
            this->table = obj.table;
            this->count = obj.count;
        }
        
        return *this;
    }

    /* Hashmap(Hashmap&& obj) {
        this->table = std::move(obj);
        this->count = obj.count;
    } */

    /* Hashmap& operator=(Hashmap&& obj) {
        std::cout << "ss\n";
        if (this != &obj) {
            this->table = std::move(obj.table);
            this->count = std::move(obj.count);
            //this->MAX_LOAD_FACTOR = obj.MAX_LOAD_FACTOR;
        }
        return *this;
    } */

    void insert(const Key& key, const Value& value) {
        //std::cout << "inserting ";
        auto bucketIter = getBucket(key);
        
        for (auto it = bucketIter->begin(); it != bucketIter->end(); ++it) {
            
            if (Compare()(it->first, key)) {
                it->second = value;
                return;
            }
        }

        bucketIter->push_front(Item(key, value));
        count++;
    }

    Value& operator[](const Key& key) {
        table_iterator bucket = getBucket(key);

        for (auto elem = bucket->begin(); elem != bucket->end(); ++elem) {

            if (Compare()(elem->first, key)) {
                return elem->second;
            }
        }
        
        if (this->shouldResize()) {
            this->resize();
        }

        bucket->push_front(Item(key, Value()));
        ++count;

        return bucket->front().second;
    }
    
    iterator find(const Key& key) {
        table_iterator bucket = getBucket(key);

        for (bucket_iterator elem = bucket->begin(); elem != bucket->end(); ++elem) {

            if (Compare()(elem->first, key)) {
                return iterator(bucket, elem, table.end());
            }
        }
        return this->end();
    }

    const_iterator find(const Key& key) const {
        const_table_iterator bucket = getBucket(key);

        for (const_bucket_iterator elem = bucket->cbegin(); elem != bucket->cend(); ++elem) {

            if (Compare()(elem->first, key)) {
                return const_iterator(bucket, elem, table.end());
            }
        }
        return this->cend();
    }

    using _is_transparent = has_type_member<Hash>;

    template <typename K, typename H = Hash, typename Eq = Compare>
    typename enable_if<std::conjunction_v<has_type_member<H>, has_type_member<Eq>>, iterator>::type find(const K& key) {
        //std::cout << "templated find - ";
        table_iterator bucket = table.begin() + Hash()(key) % table.size(); //getBucket(key);

        for (bucket_iterator elem = bucket->begin(); elem != bucket->end(); ++elem) {

            if (Compare()(elem->first, key)) {
                return iterator(bucket, elem, table.end());
            }
        }
        return this->end();
    }

    template <typename K, typename H = Hash, typename Eq = Compare>
    typename enable_if<std::conjunction_v<has_type_member<H>, has_type_member<Eq>>, const_iterator>::type find(const K& key) const {
        //std::cout << "templated find - ";
        const_table_iterator bucket = table.begin() + Hash()(key) % table.size(); //getBucket(key);

        for (const_bucket_iterator elem = bucket->begin(); elem != bucket->end(); ++elem) {

            if (Compare()(elem->first, key)) {
                return const_iterator(bucket, elem, table.end());
            }
        }
        return this->end();
    }

    bool contains(const Key& key) const {
        auto bucket = getBucket(key);

        for (auto elem = bucket->begin(); elem != bucket->end(); ++elem) {

            if (Compare()(elem->first, key)) {
                return true;
            }
        }
        return false;
    }

    void erase(const Key& key) {
        auto bucket = getBucket(key);

        for (auto elem = bucket->before_begin(); elem != bucket->end(); ++elem) {
            auto nextEl = elem;
            nextEl++;

            if (Compare()(nextEl->first, key) && nextEl != bucket->end()) {
                bucket->erase_after(elem);
                count--;
                return;
            }
        }
    }
};