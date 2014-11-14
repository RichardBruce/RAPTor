#ifdef STAND_ALONE
#define BOOST_TEST_MODULE pair_manager test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <cstdint>

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Raytracer headers */

/* Physics headers */
#include "pair_manager.h"


using namespace raptor_physics;

/* Mock hash so I can cause collisions */
class mock_hash_fn
{
    public :
        std::size_t operator()(const contents &p) const
        {
            return reinterpret_cast<std::uint64_t>(p.second);
        }

    private :
};

/* Test data */
struct pair_manager_fixture : private boost::noncopyable
{
    pair_manager_fixture()
    : uut() { }

    pair_manager<mock_hash_fn> uut;
};

BOOST_FIXTURE_TEST_SUITE( pair_manager_tests, pair_manager_fixture );


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(uut.size() == 0);
    BOOST_CHECK(uut.load_factor() == 1.0f);
    BOOST_CHECK(uut.capacity() == 0);
    BOOST_CHECK(uut.begin() == uut.end());
}

/* Test capacity change */
BOOST_AUTO_TEST_CASE( capacity_test )
{
    BOOST_CHECK(uut.size() == 0);
    BOOST_CHECK(uut.load_factor() == 1.0f);
    BOOST_CHECK(uut.capacity() == 0);

    /* We lazily rehash when setting load factor so see a capacity of 32 here and 64 below */
    uut.reserve(27);
    uut.load_factor(0.7f);
    BOOST_CHECK(uut.load_factor() == 0.7f);
    BOOST_CHECK(uut.capacity() == 32);

    uut.load_factor(0.7f);
    uut.reserve(27);
    BOOST_CHECK(uut.load_factor() == 0.7f);
    BOOST_CHECK(uut.capacity() == 64);
}

/* Insert test */
BOOST_AUTO_TEST_CASE( insert_test )
{
    /* Insert */
    const auto *const ret0 = uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.size() == 1);
    BOOST_CHECK(ret0->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret0->second == reinterpret_cast<physics_object *>(0x02));

    /* Insert the same pair again, but in reversed order */
    const auto *const ret1 = uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(uut.size() == 1);
    BOOST_CHECK(ret1 == ret0);

    /* Insert some new pairs */
    const auto *const ret2 = uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 2);
    BOOST_CHECK(ret2->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret2->second == reinterpret_cast<physics_object *>(0x03));

    const auto *const ret3 = uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.size() == 3);
    BOOST_CHECK(ret3->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret3->second == reinterpret_cast<physics_object *>(0x04));

    /* Insert exactly the same pair again */
    const auto *const ret4 = uut.insert(*ret3);
    BOOST_CHECK(uut.size() == 3);
    BOOST_CHECK(ret4 == ret3);

    /* Insert with collision */
    const auto *const ret5 = uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 4);
    BOOST_CHECK(ret5->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret5->second == reinterpret_cast<physics_object *>(0x03));

    /* Insert causing (multiple) rehash */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x05), reinterpret_cast<physics_object *>(0x900));
    BOOST_CHECK(uut.size() == 13);
}

/* Clear test */
BOOST_AUTO_TEST_CASE( clear_test )
{
    /* Insert some data */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.size() == 3);

    /* Clear */
    uut.clear();
    BOOST_CHECK(uut.size() == 0);
    BOOST_CHECK(uut.begin() == uut.end());
}

/* Find test */
BOOST_AUTO_TEST_CASE( find_test )
{
    /* Insert some data */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 4);

    /* Find */
    const auto *const ret0 = uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret0->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret0->second == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.size() == 4);

    /* Find the same pair again, but in reversed order */
    const auto *const ret1 = uut.find(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret1 == ret0);
    BOOST_CHECK(uut.size() == 4);

    /* Find some new pairs */
    const auto *const ret2 = uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(ret2->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret2->second == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 4);

    const auto *const ret3 = uut.find(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret3->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret3->second == reinterpret_cast<physics_object *>(0x04));
    BOOST_CHECK(uut.size() == 4);

    /* Find exactly the same pair again */
    const auto *const ret4 = uut.find(*ret3);
    BOOST_CHECK(ret4 == ret3);
    BOOST_CHECK(uut.size() == 4);

    /* Look for something that isnt there */
    const auto *const ret5 = uut.find(reinterpret_cast<physics_object *>(0x10), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret5 == uut.end());
    BOOST_CHECK(uut.size() == 4);

    /* Find after collision */
    const auto *const ret6 = uut.find(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(ret6->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret6->second == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 4);

    /* Cause (multiple) rehash */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x100));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x900));
    uut.insert(reinterpret_cast<physics_object *>(0x05), reinterpret_cast<physics_object *>(0x900));
    BOOST_CHECK(uut.size() == 13);

    /* Find after causing rehash */
    const auto *const ret7 = uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x100));
    BOOST_CHECK(ret7->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret7->second == reinterpret_cast<physics_object *>(0x100));

    const auto *const ret8 = uut.find(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x100));
    BOOST_CHECK(ret8->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret8->second == reinterpret_cast<physics_object *>(0x100));

    const auto *const ret9 = uut.find(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x100));
    BOOST_CHECK(ret9->first == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(ret9->second == reinterpret_cast<physics_object *>(0x100));

    const auto *const ret10 = uut.find(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x100));
    BOOST_CHECK(ret10->first == reinterpret_cast<physics_object *>(0x04));
    BOOST_CHECK(ret10->second == reinterpret_cast<physics_object *>(0x100));

    const auto *const ret11 = uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x900));
    BOOST_CHECK(ret11->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret11->second == reinterpret_cast<physics_object *>(0x900));

    const auto *const ret12 = uut.find(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x900));
    BOOST_CHECK(ret12->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret12->second == reinterpret_cast<physics_object *>(0x900));

    const auto *const ret13 = uut.find(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x900));
    BOOST_CHECK(ret13->first == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(ret13->second == reinterpret_cast<physics_object *>(0x900));

    const auto *const ret14 = uut.find(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x900));
    BOOST_CHECK(ret14->first == reinterpret_cast<physics_object *>(0x04));
    BOOST_CHECK(ret14->second == reinterpret_cast<physics_object *>(0x900));

    const auto *const ret15 = uut.find(reinterpret_cast<physics_object *>(0x05), reinterpret_cast<physics_object *>(0x900));
    BOOST_CHECK(ret15->first == reinterpret_cast<physics_object *>(0x05));
    BOOST_CHECK(ret15->second == reinterpret_cast<physics_object *>(0x900));

    BOOST_CHECK(uut.size() == 13);
}

/* Erase test */
BOOST_AUTO_TEST_CASE( erase_test )
{
    /* Insert some data */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.size() == 3);

    /* Erase */
    const auto ret0 = uut.erase(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02)) == uut.end());
    BOOST_CHECK(ret0 != uut.end());
    BOOST_CHECK(uut.size() == 2);

    /* Erase again */
    const auto ret1 = uut.erase(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02)) == uut.end());
    BOOST_CHECK(ret1 == uut.end());
    BOOST_CHECK(uut.size() == 2);

    /* Erase the same pair again, but in reversed order */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    const auto ret2 = uut.erase(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x01)) == uut.end());
    BOOST_CHECK(ret2 == uut.end());
    BOOST_CHECK(uut.size() == 2);

    /* Erase some new pairs */
    /* Note: When 1,2 was erased the first time 4,2 was moved into its spot so no longer at the end */
    const auto ret3 = uut.erase(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02)) == uut.end());
    BOOST_CHECK(ret3 != uut.end());
    BOOST_CHECK(uut.size() == 1);

    const auto ret4 = uut.erase(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03)) == uut.end());
    BOOST_CHECK(ret4 == uut.end());
    BOOST_CHECK(uut.size() == 0);

    /* Erase something that is no longer there */
    const auto ret5 = uut.erase(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02)) == uut.end());
    BOOST_CHECK(ret5 == uut.end());
    BOOST_CHECK(uut.size() == 0);
}

BOOST_AUTO_TEST_CASE( erase_after_collision_test )
{
    /* Insert some data */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x03));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 3);
 
     /* Erase after collision */
    const auto ret0 = uut.erase(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(ret0->first == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(ret0->second == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 2);

    const auto ret1 = uut.erase(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(ret1 == uut.end());
    BOOST_CHECK(uut.size() == 1);

    const auto ret2 = uut.erase(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret2 == uut.end());
    BOOST_CHECK(uut.size() == 0);
}

BOOST_AUTO_TEST_CASE( erase_after_rehash_test )
{
    /* Cause rehash */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x05), reinterpret_cast<physics_object *>(0x90));
    BOOST_CHECK(uut.size() == 9);

    /* Erase after causing rehash */
    const auto ret0 = uut.erase(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x10));
    BOOST_CHECK(ret0 != uut.end());
    BOOST_CHECK(uut.size() == 8);

    const auto ret1 = uut.erase(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x10));
    BOOST_CHECK(ret1 != uut.end());
    BOOST_CHECK(uut.size() == 7);

    const auto ret2 = uut.erase(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x10));
    BOOST_CHECK(ret2 != uut.end());
    BOOST_CHECK(uut.size() == 6);

    const auto ret3 = uut.erase(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x10));
    BOOST_CHECK(ret3 != uut.end());
    BOOST_CHECK(uut.size() == 5);

    /* From here on down the order was reversed as above were erase */
    const auto ret4 = uut.erase(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x90));
    BOOST_CHECK(ret4 == uut.end());
    BOOST_CHECK(uut.size() == 4);

    const auto ret5 = uut.erase(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x90));
    BOOST_CHECK(ret5 == uut.end());
    BOOST_CHECK(uut.size() == 3);

    const auto ret6 = uut.erase(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x90));
    BOOST_CHECK(ret6 == uut.end());
    BOOST_CHECK(uut.size() == 2);

    const auto ret7 = uut.erase(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x90));
    BOOST_CHECK(ret7 == uut.end());
    BOOST_CHECK(uut.size() == 1);

    const auto ret8 = uut.erase(reinterpret_cast<physics_object *>(0x05), reinterpret_cast<physics_object *>(0x90));
    BOOST_CHECK(ret8 == uut.end());
    BOOST_CHECK(uut.size() == 0);
}

BOOST_AUTO_TEST_CASE( erase_sequence_test )
{
    /* Insert some data */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.size() == 3);

    const auto ret0 = uut.erase(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x02)) == uut.end());
    BOOST_CHECK(ret0->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(ret0->second == reinterpret_cast<physics_object *>(0x04));
    BOOST_CHECK(uut.size() == 2);
    
    const auto ret1 = uut.erase(ret0);
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x02)) == uut.end());
    BOOST_CHECK(ret1->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(ret1->second == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(uut.size() == 1);
    
    const auto ret2 = uut.erase(ret1);
    BOOST_CHECK(uut.find(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x03)) == uut.end());
    BOOST_CHECK(ret2 == uut.end());
    BOOST_CHECK(uut.size() == 0);
}

BOOST_AUTO_TEST_CASE( iterate_test )
{
    /* Fill up the hash */
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x10));
    uut.insert(reinterpret_cast<physics_object *>(0x01), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x02), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x03), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x04), reinterpret_cast<physics_object *>(0x90));
    uut.insert(reinterpret_cast<physics_object *>(0x05), reinterpret_cast<physics_object *>(0x90));
    BOOST_CHECK(uut.size() == 9);

    /* Erase after causing rehash */
    auto iter = uut.begin();
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x10));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x10));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x10));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x04));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x10));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x01));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x90));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x02));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x90));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x03));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x90));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x04));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x90));

    ++iter;
    BOOST_CHECK(iter->first == reinterpret_cast<physics_object *>(0x05));
    BOOST_CHECK(iter->second == reinterpret_cast<physics_object *>(0x90));
}

BOOST_AUTO_TEST_SUITE_END()
