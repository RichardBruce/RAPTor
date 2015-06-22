#pragma once

/* Standard headers */
#include <cassert>

/* Boost headers */
#include "boost/functional/hash.hpp"

/* Physics headers */
#include "physics_object.h"


namespace raptor_physics
{
typedef std::pair<const physics_object *, const physics_object *> contents;

inline void sort(const physics_object *& id0, const physics_object *& id1)
{
    if (id0 > id1)
    {
        std::swap(id0, id1);
    }
}

inline bool is_different_pair(const contents& p, const physics_object *id0, const physics_object *id1)
{
    return (id0 != p.first) | (id1 != p.second);
}

inline unsigned int next_power_of_two(unsigned int x)
{
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return x + 1;
}

const unsigned int INVALID_ID = 0xffffffff;

template<class HashFn = boost::hash<contents>>
class pair_manager
{
    public :
        pair_manager()
        : _hash_func(), _pairs(nullptr), _hash_table(nullptr), _next(nullptr), _load_factor(1.0), _size(0), _mask(0), _number_of_pairs(0) { }

        ~pair_manager()
        {
            delete [] _next;
            delete [] _pairs;
            delete [] _hash_table;
        }

        pair_manager& clear()
        {
            _number_of_pairs  = 0;
            std::fill_n(_hash_table, _size, INVALID_ID);

            return *this;
        }

        unsigned int capacity() const { return _size; }

        pair_manager& reserve(const unsigned int size)
        {
            rehash(size);
            return *this;
        }

        float load_factor() const { return _load_factor; }

        pair_manager& load_factor(const float load_factor)
        {
            assert(load_factor <= 1.0);
            assert(load_factor >  0.0);

            _load_factor = load_factor;
            return *this;
        }

        const contents* find(const physics_object *id0, const physics_object *id1) const
        {
            /* Order the ids */
            sort(id0, id1);

            return find(std::make_pair(id0, id1));
        }

        const contents* find(const contents &p) const
        {
            /* Check something has been added */
            if (_hash_table == nullptr)
            {
                return end();
            }

            /* Compute hash value for this pair */
            const std::size_t hash = _hash_func(p) & _mask;

            /* Look for it in the table */
            unsigned int offset = _hash_table[hash];
            while ((offset != INVALID_ID) && is_different_pair(_pairs[offset], p.first, p.second))
            {
                assert(_pairs[offset].first != nullptr);
                offset = _next[offset];
            }

            if (offset == INVALID_ID)
            {
                return end();
            }

            assert(offset < _number_of_pairs);
            return &_pairs[offset];
        }

        const contents* insert(const physics_object *id0, const physics_object *id1)
        {
            /* Order the ids */
            sort(id0, id1);

            return insert(std::make_pair(id0, id1));
        }

        const contents* insert(const contents &p)
        {
            /* Check for existing pair */
            std::size_t hash = _hash_func(p) & _mask;
            contents* pair = find(p.first, p.second, hash);
            if (pair != nullptr)
            {
                return pair;
            }

            /* Possibly rehash */
            if (_number_of_pairs >= (_size * _load_factor))
            {
                rehash(_number_of_pairs + 1);

                /* Rehash the new entry */
                hash = _hash_func(p) & _mask;
            }

            /* Update */
            contents* new_p = &_pairs[_number_of_pairs];
            new_p->first    = p.first;
            new_p->second   = p.second;

            _next[_number_of_pairs] = _hash_table[hash];
            _hash_table[hash] = _number_of_pairs++;
            return new_p;
        }


        contents* erase(const physics_object *id0, const physics_object *id1)
        {
            /* Order the ids */
            sort(id0, id1);

            return erase(std::make_pair(id0, id1));
        }

        contents* erase(const contents &p)
        {
            /* Find the pair or give up */
            const std::size_t hash = _hash_func(p) & _mask;
            contents* pair = find(p.first, p.second, hash);
            if (pair == nullptr)
            {
                return end();
            }

            assert(pair->first == p.first);
            assert(pair->second == p.second);

            return erase(pair, hash);
        }

        contents* erase(contents *const pair)
        {
            const std::size_t hash = _hash_func(*pair) & _mask;
            return erase(pair, hash);
        }

        contents* erase(contents *const pair, const std::size_t hash)
        {
            /* Search through next to find the previous entry */
            const unsigned int pair_index = get_index(pair);
            unsigned int offset = _hash_table[hash];
            assert(offset != INVALID_ID);

            unsigned int previous = INVALID_ID;
            while (offset != pair_index)
            {
                previous = offset;
                offset = _next[offset];
            }

            /* Repoint next to skip the erase entry */
            if (previous != INVALID_ID)
            {
                assert(_next[previous] == pair_index);
                _next[previous] = _next[pair_index];
            }
            /* We erased the first in the chain so repoint hash to the next */
            else
            {
                _hash_table[hash] = _next[pair_index];
            }
            /* Now we can reuse next[pair_index] */

            /* We removed the last pair, so just drop it */
            const unsigned int last_pair_idx = _number_of_pairs - 1;
            if (last_pair_idx == pair_index)
            {
                --_number_of_pairs;
                return end();
            }

            const contents* last = &_pairs[last_pair_idx];
            const unsigned int last_hash = _hash_func(*last) & _mask;

            /* Find the last pair in its next chain */
            offset = _hash_table[last_hash];
            assert(offset != INVALID_ID);

            previous = INVALID_ID;
            while (offset != last_pair_idx)
            {
                previous = offset;
                offset = _next[offset];
            }

            /* Relink the last pair in its chain*/
            if (previous != INVALID_ID)
            {
                assert(_next[previous] == last_pair_idx);
                _next[previous] = _next[last_pair_idx];
            }
            /* The last pair was first in its chain, just update the hash */
            else
            { 
                _hash_table[last_hash] = _next[last_pair_idx];
            }
            /* Now we can reuse _next[last_pair_idx] */

            /* Move last pair to the empty slot */
            _pairs[pair_index]      = _pairs[last_pair_idx];
            _next[pair_index]       = _hash_table[last_hash];
            _hash_table[last_hash]  = pair_index;
            --_number_of_pairs;

            return pair;
        }

        // Access functions
        unsigned int size() const { return _number_of_pairs; }

        contents* begin()   const { return &_pairs[0];                  }
        contents* end()     const { return &_pairs[_number_of_pairs];   }

    private :
        void rehash(const unsigned int size)
        {
            /* Make more space for the hash table */
            _size = next_power_of_two(size / _load_factor);
            _mask = _size - 1;

            delete [] _hash_table;
            _hash_table = new unsigned int[_size];
            std::fill_n(_hash_table, _size, INVALID_ID);

            /* Get some more space for data */
            contents* new_pairs     = new contents [_size];
            unsigned int* new_next  = new unsigned int [_size];

            /* Copy the old data */
            if(_number_of_pairs)
            {
                memcpy(new_pairs, _pairs, _number_of_pairs*sizeof(contents));
            }

            /* Rehash everything */
            for (unsigned int i = 0; i < _number_of_pairs; ++i)
            {
                const std::size_t hash = _hash_func(_pairs[i]) & _mask;
                new_next[i] = _hash_table[hash];
                _hash_table[hash] = i;
            }

            /* Clean up */
            delete [] _next;
            delete [] _pairs;

            _pairs = new_pairs;
            _next = new_next;
        }

        unsigned int get_index(const contents* pair)  const
        {
            return ((unsigned int)((std::size_t(pair) - std::size_t(_pairs))) / sizeof(contents));
        }

        contents* find(const physics_object *id0, const physics_object *id1, const std::size_t hash) const
        {
            if (_hash_table == nullptr)
            {
                return nullptr;
            }

            /* Look for it in the table */
            unsigned int offset = _hash_table[hash];
            while ((offset != INVALID_ID) && is_different_pair(_pairs[offset], id0, id1))
            {
                assert(_pairs[offset].first != nullptr);
                offset = _next[offset];
            }

            if (offset == INVALID_ID)
            {
                return nullptr;
            }

            assert(offset < _number_of_pairs);
            return &_pairs[offset];
        }

        HashFn          _hash_func;
        contents*       _pairs;
        unsigned int*   _hash_table;
        unsigned int*   _next;
        float           _load_factor;
        unsigned int    _size;
        unsigned int    _mask;
        unsigned int    _number_of_pairs;
};
}; /* namespace raptor_physics */
