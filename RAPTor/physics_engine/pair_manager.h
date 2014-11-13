#ifndef __PAIR_MANAGER_H__
#define __PAIR_MANAGER_H__

/* Standard headers */
#include <cassert>

/* Boost headers */
#include "boost/functional/hash.hpp"
#include "boost/iterator/iterator_facade.hpp"

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


template<class HashFn = boost::hash<contents>>
class pair_manager
{
    public :
        /* Iterators */
        class pair_manager_iterator : public boost::iterator_facade<pair_manager_iterator, contents, boost::forward_traversal_tag>
        {
            public :
                pair_manager_iterator(const pair_manager *const container, const unsigned int bin, const unsigned int idx)
                : _container(container), _bin(bin), _idx(idx) {  };

                constexpr pair_manager_iterator(const pair_manager_iterator &rhs)
                : _container(rhs._container), _bin(rhs._bin), _idx(rhs._idx) {  }

                pair_manager_iterator& operator=(pair_manager_iterator&& rhs)
                {
                    _container = rhs._container;
                    _bin = rhs._bin;
                    _idx = rhs._idx;

                    return *this;
                }

                operator contents*() { return &_container->_pairs[(_bin * _container->_load_factor) + _idx]; }

            private :
                friend class boost::iterator_core_access;

                void increment()
                {
                    ++_idx;
                    if (_idx == _container->_pairs_in_bin[_bin])
                    {
                        ++_bin;
                        while ((_bin < _container->_size) && (_container->_pairs_in_bin[_bin] == 0))
                        {
                            ++_bin;
                        }

                        assert(_container->_pairs_in_bin[_bin] < _container->_load_factor);
                        _idx = 0;
                    }
                }

                bool equal(pair_manager_iterator const& iter) const
                {
                    return (_container == iter._container) && (_bin == iter._bin) && (_idx == iter._idx);
                }

                contents& dereference() const
                {
                    return _container->_pairs[(_bin * _container->_load_factor) + _idx];
                }

                const pair_manager *    _container;
                unsigned int            _bin;
                unsigned int            _idx;
        };

        /* CTOR */
        pair_manager()
        : _hash_func(), _pairs(nullptr), _pairs_in_bin(nullptr), _load_factor(8), _size(128), _mask(0x7f), _number_of_pairs(0) { }

        unsigned int capacity()     const { return _size * _load_factor;    }
        unsigned int load_factor()  const { return _load_factor;            }

        pair_manager& reserve(const unsigned int size)
        {
            rehash(size);
            return *this;
        }

        pair_manager& load_factor(const unsigned int load_factor)
        {
            assert(load_factor > 1);
            _load_factor = next_power_of_two(load_factor);
            rehash(_size - 1);

            return *this;
        }

        pair_manager& clear()
        {
            if (_pairs != nullptr)
            {
                _number_of_pairs = 0;
                std::fill_n(_pairs_in_bin.get(), _size, 0);
            }

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
            /* Compute hash value for this pair */
            const std::size_t hash = _hash_func(p) & _mask;

            /* Find */
            return find(p.first, p.second, hash);
        }

        const contents* insert(const physics_object *id0, const physics_object *id1)
        {
            /* Order the ids */
            sort(id0, id1);

            return insert(std::make_pair(id0, id1));
        }

        const contents* insert(const contents &p)
        {
            /* Check the table exists */
            if (_pairs == nullptr)
            {
                _pairs_in_bin.reset(new unsigned int [_size] ());
                _pairs.reset(new contents [_size * _load_factor]);
            }

            /* Check for existing pair */
            std::size_t hash = _hash_func(p) & _mask;
            assert(_pairs_in_bin[hash] < _load_factor);

            contents* pair = find(p.first, p.second, hash);
            if (pair != nullptr)
            {
                return pair;
            }
            
            /* Possibly rehash */
            while (_pairs_in_bin[hash] == (_load_factor - 1))
            {
                rehash(_size + 1);

                /* Rehash the new entry */
                hash = _hash_func(p) & _mask;
            }

            /* Update */
            contents* new_p = &_pairs[(hash * _load_factor) + _pairs_in_bin[hash]];
            (*new_p) = p;

            ++_pairs_in_bin[hash];
            ++_number_of_pairs;

            assert(_pairs_in_bin[hash] < _load_factor);
            return new_p;
        }

        void check_bin_size(const std::size_t hash)
        {
            assert(_pairs_in_bin[hash] < _load_factor);
        }


        pair_manager_iterator erase(const physics_object *id0, const physics_object *id1)
        {
            /* Order the ids */
            sort(id0, id1);

            return erase(std::make_pair(id0, id1));
        }

        pair_manager_iterator erase(const contents &p)
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

        pair_manager_iterator erase(const pair_manager_iterator &p)
        {
            contents &pair = *p;
            const std::size_t hash = _hash_func(pair) & _mask;
            return erase(&pair, hash);
        }

        pair_manager_iterator erase(contents *const pair, const std::size_t hash)
        {
            /* Convert to indices */
            const unsigned int idx = static_cast<unsigned int>((((std::size_t)pair - (std::size_t)_pairs.get())) / sizeof(contents));
            const unsigned int bin = idx / _load_factor;
            assert(_pairs_in_bin[bin] < _load_factor);

            /* Decrement size of bin */
            --_pairs_in_bin[bin];
            --_number_of_pairs;
            assert(_pairs_in_bin[hash] < _load_factor);

            /* Move the element now out the end of the bin into the erased element */
            const unsigned int bin_offset = bin * _load_factor;
            const unsigned int last_pair = bin_offset + _pairs_in_bin[bin];
            if (idx != last_pair)
            {
                _pairs[idx] = _pairs[last_pair];
                return pair_manager_iterator(this, bin, idx - bin_offset);
            }
            /* Search for the next pair in other bins */
            else
            {
                for (unsigned int i = bin + 1; i < _size; ++i)
                {
                    if (_pairs_in_bin[i] > 0)
                    {
                        return pair_manager_iterator(this, i, 0);
                    }
                }

                for (unsigned int i = 0; i < bin; ++i)
                {
                    if (_pairs_in_bin[i] > 0)
                    {
                        return pair_manager_iterator(this, i, 0);
                    }
                }

                return end();
            }
        }

        /* Access functions */
        unsigned int size() const { return _number_of_pairs; }

        pair_manager_iterator begin() const
        {
            /* Check we have data */
            if (_pairs == nullptr)
            {
                return pair_manager_iterator(this, 0, 0);
            }

            /* Find first populated bin */
            unsigned int bin = 0;
            while ((bin < _size) && (_pairs_in_bin[bin] == 0))
            {
                ++bin;
            }
            assert(_pairs_in_bin[bin] < _load_factor);

            return pair_manager_iterator(this, bin, 0);
        }

        pair_manager_iterator end() const
        {
            /* Check we have data */
            if (_pairs == nullptr)
            {
                return pair_manager_iterator(this, 0, 0);
            }

            return pair_manager_iterator(this, _size, 0);
        }

    private :
        void rehash(const unsigned int size)
        {
            /* Check the hash exists */
            if (_pairs == nullptr)
            {
                _size = next_power_of_two(size);
                _mask = size - 1;
                return;
            }

            /* Make more space for the hash table */
            const unsigned int new_size = next_power_of_two(size);
            _mask = new_size - 1;

            /* Get some more space for data */
            std::unique_ptr<contents []> new_pairs(new contents [new_size * _load_factor]);
            std::unique_ptr<unsigned int []> new_pairs_in_bin(new unsigned int [new_size] ());

            /* Rehash everything */
            unsigned int bin_offset = 0;
            for (unsigned int i = 0; i < _size; ++i)
            {
                for (unsigned int j = 0; j < _pairs_in_bin[i]; ++j)
                {
                    /* TODO - Could save the call to hash_func by keeping unmasked hash */
                    /*        This would speed up rehash, but slow down everything else, lets just keep rehashing rare and everything else fast */
                    const std::size_t hash = _hash_func(_pairs[bin_offset + j]) & _mask;
                    new_pairs[(hash * _load_factor) + new_pairs_in_bin[hash]] = _pairs[bin_offset + j];
                    ++new_pairs_in_bin[hash];
                    assert(new_pairs_in_bin[hash] < _load_factor);
                }

                bin_offset += _load_factor;
            }

            /* Swap in the new table  */
            _size = new_size;
            _pairs.swap(new_pairs);
            _pairs_in_bin.swap(new_pairs_in_bin);
        }

        contents* find(const physics_object *id0, const physics_object *id1, const std::size_t hash) const
        {
            assert(_pairs_in_bin[hash] < _load_factor);

            /* Check we have data */
            if (_pairs == nullptr)
            {
                return nullptr;
            }

            /* Look for it in the table */
            const unsigned int bin_offset = hash * _load_factor;
            for (unsigned int i = bin_offset; i < (bin_offset + _pairs_in_bin[hash]); ++i)
            {
                assert(_pairs_in_bin[hash] < _load_factor);
                if (!is_different_pair(_pairs[i], id0, id1))
                {
                    return &_pairs[i];
                }
            }

            /* Not found */
            return nullptr;
        }

        HashFn                              _hash_func;
        std::unique_ptr<contents []>        _pairs;
        std::unique_ptr<unsigned int []>    _pairs_in_bin;
        unsigned int                        _load_factor;
        unsigned int                        _size;
        unsigned int                        _mask;
        unsigned int                        _number_of_pairs;
};
}; /* namespace raptor_physics */

#endif /* #ifndef __PAIR_MANAGER_H__ */
