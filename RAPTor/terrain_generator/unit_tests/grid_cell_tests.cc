#ifdef STAND_ALONE
#define BOOST_TEST_MODULE grid_cell test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "grid_cell.h"


namespace raptor_terrain
{
namespace test
{
/* Test data */
struct grid_cell_fixture : private boost::noncopyable
{
    grid_cell_fixture() :
        verts(new point_t<> [16] {
            point_t<>(0.0, 0.0, 0.0),
            point_t<>(1.0, 0.0, 0.0),
            point_t<>(1.0, 0.0, 1.0),
            point_t<>(0.0, 0.0, 1.0),
            
            point_t<>(0.0, 0.0, 0.0),
            point_t<>(1.0, 2.0, 0.0),
            point_t<>(1.0, 2.0, 1.0),
            point_t<>(0.0, 0.0, 1.0),
            
            point_t<>(0.0, 0.0, 0.0),
            point_t<>(1.0, 0.5, 0.0),
            point_t<>(1.0, 0.5, 1.0),
            point_t<>(0.0, 0.0, 1.0),

            point_t<>(0.0, 0.0, 0.0),
            point_t<>(1.0, 0.1, 0.0),
            point_t<>(1.0, 0.1, 1.0),
            point_t<>(0.0, 0.0, 1.0)
        }),
        gc(new grid_cell(nullptr, -1, 1, 2, 5, 101, 102, 107, 67)),
        merge_gc0(new grid_cell(verts.get(), -1, 1, 2, 5,  0,  1,  2,  3)),
        merge_gc1(new grid_cell(verts.get(), -1, 1, 2, 5,  4,  5,  6,  7)),
        merge_gc2(new grid_cell(verts.get(), -1, 1, 2, 5,  8,  9, 10, 11)),
        merge_gc3(new grid_cell(verts.get(), -1, 1, 2, 5, 12, 13, 14, 15)) {  };
    
    std::unique_ptr<point_t<> []> verts;
    std::unique_ptr<grid_cell> gc;
    std::unique_ptr<grid_cell> merge_gc0;
    std::unique_ptr<grid_cell> merge_gc1;
    std::unique_ptr<grid_cell> merge_gc2;
    std::unique_ptr<grid_cell> merge_gc3;
};

BOOST_FIXTURE_TEST_SUITE( grid_cell_tests, grid_cell_fixture )

const float result_tolerance = 0.0005;


/* Neighbour tests */
BOOST_AUTO_TEST_CASE( turn_left_test )
{
    BOOST_CHECK(turn_left(neighbour_t::NORTH)   == neighbour_t::WEST);
    BOOST_CHECK(turn_left(neighbour_t::SOUTH)   == neighbour_t::EAST);
    BOOST_CHECK(turn_left(neighbour_t::EAST)    == neighbour_t::NORTH);
    BOOST_CHECK(turn_left(neighbour_t::WEST)    == neighbour_t::SOUTH);
}

BOOST_AUTO_TEST_CASE( turn_right_test )
{
    BOOST_CHECK(turn_right(neighbour_t::NORTH)  == neighbour_t::EAST);
    BOOST_CHECK(turn_right(neighbour_t::SOUTH)  == neighbour_t::WEST);
    BOOST_CHECK(turn_right(neighbour_t::EAST)   == neighbour_t::SOUTH);
    BOOST_CHECK(turn_right(neighbour_t::WEST)   == neighbour_t::NORTH);
}

BOOST_AUTO_TEST_CASE( turn_around_test )
{
    BOOST_CHECK(turn_around(neighbour_t::NORTH) == neighbour_t::SOUTH);
    BOOST_CHECK(turn_around(neighbour_t::SOUTH) == neighbour_t::NORTH);
    BOOST_CHECK(turn_around(neighbour_t::EAST)  == neighbour_t::WEST);
    BOOST_CHECK(turn_around(neighbour_t::WEST)  == neighbour_t::EAST);
}

/* Test ctor for shapes with known tensor */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(gc->north()         ==  -1);
    BOOST_CHECK(gc->south()         ==   1);
    BOOST_CHECK(gc->east()          ==   2);
    BOOST_CHECK(gc->west()          ==   5);
    BOOST_CHECK(gc->top_left()      == 101);
    BOOST_CHECK(gc->bottom_left()   == 102);
    BOOST_CHECK(gc->bottom_right()  == 107);
    BOOST_CHECK(gc->top_right()     ==  67);
}

/* Getter tests */
BOOST_AUTO_TEST_CASE( neighbour_test )
{
    BOOST_CHECK(gc->neighbour(neighbour_t::NORTH)   == -1);
    BOOST_CHECK(gc->neighbour(neighbour_t::SOUTH)   ==  1);
    BOOST_CHECK(gc->neighbour(neighbour_t::EAST)    ==  2);
    BOOST_CHECK(gc->neighbour(neighbour_t::WEST)    ==  5);
}

BOOST_AUTO_TEST_CASE( corner_entering_test )
{
    /* Test moving north */
    BOOST_CHECK(gc->corner_entering(neighbour_t::NORTH, neighbour_t::SOUTH) ==  67);
    BOOST_CHECK(gc->corner_entering(neighbour_t::NORTH, neighbour_t::EAST)  == 107);
    BOOST_CHECK(gc->corner_entering(neighbour_t::NORTH, neighbour_t::WEST)  ==  67);

    /* Test moving south */
    BOOST_CHECK(gc->corner_entering(neighbour_t::SOUTH, neighbour_t::NORTH) == 102);
    BOOST_CHECK(gc->corner_entering(neighbour_t::SOUTH, neighbour_t::EAST)  == 102);
    BOOST_CHECK(gc->corner_entering(neighbour_t::SOUTH, neighbour_t::WEST)  == 101);

    /* Test moving east */
    BOOST_CHECK(gc->corner_entering(neighbour_t::EAST, neighbour_t::NORTH)  == 107);
    BOOST_CHECK(gc->corner_entering(neighbour_t::EAST, neighbour_t::SOUTH)  == 102);
    BOOST_CHECK(gc->corner_entering(neighbour_t::EAST, neighbour_t::WEST)   == 107);

    /* Test moving west */
    BOOST_CHECK(gc->corner_entering(neighbour_t::WEST, neighbour_t::NORTH)  ==  67);
    BOOST_CHECK(gc->corner_entering(neighbour_t::WEST, neighbour_t::SOUTH)  == 101);
    BOOST_CHECK(gc->corner_entering(neighbour_t::WEST, neighbour_t::EAST)   == 101);
}

BOOST_AUTO_TEST_CASE( corner_leaving_test )
{
    /* Test moving north */
    BOOST_CHECK(gc->corner_leaving(neighbour_t::NORTH, neighbour_t::SOUTH)  == 101);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::NORTH, neighbour_t::EAST)   == 107);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::NORTH, neighbour_t::WEST)   ==  67);

    /* Test moving south */
    BOOST_CHECK(gc->corner_leaving(neighbour_t::SOUTH, neighbour_t::NORTH)  == 107);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::SOUTH, neighbour_t::EAST)   == 102);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::SOUTH, neighbour_t::WEST)   == 101);

    /* Test moving east */
    BOOST_CHECK(gc->corner_leaving(neighbour_t::EAST, neighbour_t::NORTH)   == 107);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::EAST, neighbour_t::SOUTH)   == 102);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::EAST, neighbour_t::WEST)    ==  67);

    /* Test moving west */
    BOOST_CHECK(gc->corner_leaving(neighbour_t::WEST, neighbour_t::NORTH)   ==  67);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::WEST, neighbour_t::SOUTH)   == 101);
    BOOST_CHECK(gc->corner_leaving(neighbour_t::WEST, neighbour_t::EAST)    == 102);
}

BOOST_AUTO_TEST_CASE( furthest_index_test )
{
    BOOST_CHECK(gc->furthest_index(neighbour_t::NORTH)  ==  67);
    BOOST_CHECK(gc->furthest_index(neighbour_t::SOUTH)  == 102);
    BOOST_CHECK(gc->furthest_index(neighbour_t::EAST)   == 107);
    BOOST_CHECK(gc->furthest_index(neighbour_t::WEST)   == 101);
}


/* Merging tests */
BOOST_AUTO_TEST_CASE( can_merge_test )
{
    BOOST_CHECK(!merge_gc0->can_merge(*merge_gc1, 0.5));
    BOOST_CHECK(!merge_gc1->can_merge(*merge_gc0, 0.5));
    BOOST_CHECK( merge_gc0->can_merge(*merge_gc2, 0.5));
    BOOST_CHECK( merge_gc2->can_merge(*merge_gc0, 0.5));

    BOOST_CHECK(!merge_gc0->can_merge(*merge_gc2, 0.95));
    BOOST_CHECK(!merge_gc2->can_merge(*merge_gc0, 0.95));
    BOOST_CHECK( merge_gc0->can_merge(*merge_gc3, 0.95));
    BOOST_CHECK( merge_gc3->can_merge(*merge_gc0, 0.95));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_terrain */
