#ifdef STAND_ALONE
#define BOOST_TEST_MODULE texture_mapper test

/* Common headers */
#include "common.h"
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

// Standard headers

#include "texture_mapper.h"

#include "boost/test/unit_test.hpp"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001;

BOOST_AUTO_TEST_SUITE( texture_mapper_tests );


BOOST_AUTO_TEST_CASE( texture_mapper_flat_bump_map_test )
{
    /* Texture in xy */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f)) == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f)) == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f)) == point_t(0.0f, 0.0f, 1.0f));

    /* Texture in yz */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f)) == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f)) == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f)) == point_t(1.0f, 0.0f, 0.0f));

    /* Texture in zx */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f)) == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f)) == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f)) == point_t(0.0f, 1.0f, 0.0f));

    /* Texture in yx */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f)) == point_t(-1.0f,  0.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f)) == point_t( 0.0f, -1.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f)) == point_t( 0.0f,  0.0f, -1.0f));

    /* Texture in zy */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f)) == point_t( 0.0f, -1.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f)) == point_t( 0.0f,  0.0f, -1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f)) == point_t(-1.0f,  0.0f,  0.0f));

    /* Texture in xz */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f)) == point_t( 0.0f,  0.0f, -1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f)) == point_t(-1.0f,  0.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f)) == point_t( 0.0f, -1.0f,  0.0f));
}

BOOST_AUTO_TEST_CASE( texture_mapper_flat_double_dist_bump_map_test )
{
    /* Texture in xy */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f)) == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f)) == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f)) == point_t(0.0f, 0.0f, 1.0f));

    /* Texture in yz */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f)) == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f)) == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f)) == point_t(1.0f, 0.0f, 0.0f));

    /* Texture in zx */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f)) == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f)) == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f)) == point_t(0.0f, 1.0f, 0.0f));

    /* Texture in yx */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f)) == point_t(-1.0f,  0.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f)) == point_t( 0.0f, -1.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f)) == point_t( 0.0f,  0.0f, -1.0f));

    /* Texture in zy */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f)) == point_t( 0.0f, -1.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f)) == point_t( 0.0f,  0.0f, -1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f)) == point_t(-1.0f,  0.0f,  0.0f));

    /* Texture in xz */
    BOOST_CHECK(bump_map(point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f)) == point_t( 0.0f,  0.0f, -1.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f)) == point_t(-1.0f,  0.0f,  0.0f));
    BOOST_CHECK(bump_map(point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 0.0f, 0.0f), point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f)) == point_t( 0.0f, -1.0f,  0.0f));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
