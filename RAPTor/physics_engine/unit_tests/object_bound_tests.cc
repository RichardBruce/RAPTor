#ifdef STAND_ALONE
#define BOOST_TEST_MODULE object_bound test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "object_bound.h"


using namespace raptor_physics;

BOOST_AUTO_TEST_SUITE( object_bound_tests )


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    /* Construct */
    object_bound uut0(reinterpret_cast<physics_object *>(0x5), 0.5, true);

    /* Checks */
    BOOST_CHECK(uut0.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut0.bound()     == 0.5);
    BOOST_CHECK(uut0.index()     == -1);
    BOOST_CHECK(uut0.min()       == true);

    /* Construct */
    object_bound uut1(reinterpret_cast<physics_object *>(0x15), 1.5, false);

    /* Checks */
    BOOST_CHECK(uut1.object()    == reinterpret_cast<physics_object *>(0x15));
    BOOST_CHECK(uut1.bound()     == 1.5);
    BOOST_CHECK(uut1.index()     == -1);
    BOOST_CHECK(uut1.min()       == false);
}

BOOST_AUTO_TEST_CASE( bound_test )
{
    object_bound uut(reinterpret_cast<physics_object *>(0x5), 0.5, true);

    /* Checks */
    BOOST_CHECK(uut.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut.bound()     == 0.5);
    BOOST_CHECK(uut.index()     == -1);
    BOOST_CHECK(uut.min()       == true);

    /* Set bound */
    uut.bound(1.5);

    /* Checks */
    BOOST_CHECK(uut.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut.bound()     == 1.5);
    BOOST_CHECK(uut.index()     == -1);
    BOOST_CHECK(uut.min()       == true);

    /* Set bound */
    uut.bound(-1.5);

    /* Checks */
    BOOST_CHECK(uut.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut.bound()     == -1.5);
    BOOST_CHECK(uut.index()     == -1);
    BOOST_CHECK(uut.min()       == true);
}

BOOST_AUTO_TEST_CASE( index_test )
{
    object_bound uut(reinterpret_cast<physics_object *>(0x5), 0.5, true);

    /* Checks */
    BOOST_CHECK(uut.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut.bound()     == 0.5);
    BOOST_CHECK(uut.index()     == -1);
    BOOST_CHECK(uut.min()       == true);

    /* Set index */
    uut.index(5);

    /* Checks */
    BOOST_CHECK(uut.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut.bound()     == 0.5);
    BOOST_CHECK(uut.index()     == 5);
    BOOST_CHECK(uut.min()       == true);

    /* Set index */
    uut.index(1);

    /* Checks */
    BOOST_CHECK(uut.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut.bound()     == 0.5);
    BOOST_CHECK(uut.index()     == 1);
    BOOST_CHECK(uut.min()       == true);
}

BOOST_AUTO_TEST_CASE( compare_and_swap_test )
{
    object_bound uut0(reinterpret_cast<physics_object *>(0x5), 0.5, true);
    object_bound uut1(reinterpret_cast<physics_object *>(0x15), 1.5, false);
    uut0.index(1);
    uut1.index(0);

    /* Checks */
    BOOST_CHECK(uut0.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut0.bound()     == 0.5);
    BOOST_CHECK(uut0.index()     == 1);
    BOOST_CHECK(uut0.min()       == true);

    BOOST_CHECK(uut1.object()    == reinterpret_cast<physics_object *>(0x15));
    BOOST_CHECK(uut1.bound()     == 1.5);
    BOOST_CHECK(uut1.index()     == 0);
    BOOST_CHECK(uut1.min()       == false);

    /* Compare and swap */
    BOOST_CHECK(uut0.compare_and_swap(&uut1));

    /* Checks */
    BOOST_CHECK(uut0.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut0.bound()     == 0.5);
    BOOST_CHECK(uut0.index()     == 0);
    BOOST_CHECK(uut0.min()       == true);

    BOOST_CHECK(uut1.object()    == reinterpret_cast<physics_object *>(0x15));
    BOOST_CHECK(uut1.bound()     == 1.5);
    BOOST_CHECK(uut1.index()     == 1);
    BOOST_CHECK(uut1.min()       == false);

    /* Compare and swap, opposite direction means no swap */
    BOOST_CHECK(!uut1.compare_and_swap(&uut0));

    /* Checks */
    BOOST_CHECK(uut0.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut0.bound()     == 0.5);
    BOOST_CHECK(uut0.index()     == 0);
    BOOST_CHECK(uut0.min()       == true);

    BOOST_CHECK(uut1.object()    == reinterpret_cast<physics_object *>(0x15));
    BOOST_CHECK(uut1.bound()     == 1.5);
    BOOST_CHECK(uut1.index()     == 1);
    BOOST_CHECK(uut1.min()       == false);

    /* Compare and swap, repeat same direction means keep swapping */
    BOOST_CHECK(uut0.compare_and_swap(&uut1));

    /* Checks */
    BOOST_CHECK(uut0.object()    == reinterpret_cast<physics_object *>(0x5));
    BOOST_CHECK(uut0.bound()     == 0.5);
    BOOST_CHECK(uut0.index()     == 1);
    BOOST_CHECK(uut0.min()       == true);

    BOOST_CHECK(uut1.object()    == reinterpret_cast<physics_object *>(0x15));
    BOOST_CHECK(uut1.bound()     == 1.5);
    BOOST_CHECK(uut1.index()     == 0);
    BOOST_CHECK(uut1.min()       == false);
}

BOOST_AUTO_TEST_SUITE_END()
