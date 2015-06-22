#pragma once

/* Standard headers */
#include <cstdint>
#include <string>

/* Boost headers */
#include "boost/uuid/uuid.hpp"
#include "boost/functional/hash.hpp"

/* Tbb headers */
#include "tbb/concurrent_hash_map.h"

/* Networking headers */
#include "stack_map_element.h"


namespace raptor_networking
{
/* Forward declarations */
class stack_component;
class message_delivery;

using boost::uuids::uuid;


/* Hash compare for using in tbb concurrent_hash_map */
template<typename K> 
struct hash_compare
{
    /* Hash a key */
    static size_t hash(const K& key)
    {
        return boost::hash_value<K>(key);
    }

    /* Compare two keys for equality */
    static bool equal(const K& key1, const K& key2)
    {
        return (key1 == key2);
    }
}; 


/* Class to wrap an accessor and stack element to provide synchonised access to a protocol stack */
class stack_accessor
{
    private :
        typedef tbb::concurrent_hash_map<uuid, stack_map_element*, hash_compare<uuid>> stack_map;

    public :
        /* Implicit conversion to a stack accessor */
        operator stack_map::accessor&() { return _acc; }

        /* Put a stack element into the map */
        stack_accessor& create_element(stack_map_element *const element)
        {
            _acc->second = element;
            return *this;
        }

        /* Get the logical address that this stack serves */
        const uuid& address() const
        {
            return _acc->first;
        }

        /* Forward-ers to the stack element */
        std::uint32_t           next_sequence() const { return _acc->second->next_sequence();   }
        message_delivery *const stack_top()     const { return _acc->second->stack_top();       }
        stack_component *const  stack_bottom()  const { return _acc->second->stack_bottom();    }

        /* Release the lock on the underlying map */
        stack_accessor& release()
        {
            _acc.release();
            return *this;
        }

    private :
        stack_map::accessor _acc;   /* Accessor to the stack components */
};
}; /* namespace raptor_networking */
