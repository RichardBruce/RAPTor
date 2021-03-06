#ifdef STAND_ALONE
#define BOOST_TEST_MODULE tracking_info test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "simplex.h"
#include "vertex_group.h"
#include "physics_object.h"
#include "tracking_info.h"

/* Test headers */
#include "mock_physics_object.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct tracking_info_fixture
{
    tracking_info_fixture() :
        data0(physics_object_for_simplex_testing(new std::vector<point_t<>>(
            {
                point_t<>( 1.5f,  1.5f, 1.0f),
                point_t<>( 1.5f, -0.5f, 1.0f),
                point_t<>(-0.5f, -0.5f, 1.0f),
                point_t<>(-0.5f,  1.5f, 1.0f)
            }), { 0, 1, 2, 3 }, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f))),
        data1(physics_object_for_simplex_testing(new std::vector<point_t<>>(
            {
                point_t<>( 1.5f,  1.6f,  1.0f),
                point_t<>(-1.5f,  0.5f,  5.0f),
                point_t<>(-3.5f, -0.8f,  1.0f),
                point_t<>(-0.5f,  1.5f, -2.0f)
            }), { 0, 1, 2, 3 }, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f)))
    {  };

    std::unique_ptr<physics_object> data0;
    std::unique_ptr<physics_object> data1;
};


BOOST_FIXTURE_TEST_SUITE( tracking_info_tests, tracking_info_fixture )

const float result_tolerance = 0.0005f;


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    std::unique_ptr<simplex> s1(new simplex(*data1, true));

    /* Test certain collision */
    /* This is a bit naughty, but dont actaully need a class here */
    physics_object *vg = reinterpret_cast<physics_object *>(0x5);
    tracking_info<physics_object> uut(vg, s0, *s1, 0.5f, collision_t::COLLISION);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);
    BOOST_CHECK(uut.get_first_collision()       == vg);

    BOOST_CHECK(uut[vg]->get_simplex().get()        == s0);
    BOOST_CHECK(uut[vg]->get_normal_of_collision()  == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg]->get_point_of_collision()   == point_t<>(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut[vg]->get_type()                 == collision_t::COLLISION);
    BOOST_CHECK(uut[vg]->get_time()                 == 0.5f);
}

/* Get collision info test */
BOOST_AUTO_TEST_CASE( get_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    std::unique_ptr<simplex> s1(new simplex(*data1, true));

    /* Test certain collision */
    /* This is a bit naughty, but dont actaully need a class here */
    physics_object *vg = reinterpret_cast<physics_object *>(0x5);
    tracking_info<physics_object> uut(vg, s0, *s1, 0.5f, collision_t::COLLISION);

    /* Check unknown object give nullptrs, otherwise this is well tested by other tests */
    BOOST_CHECK(uut[nullptr] == nullptr);
    BOOST_CHECK(uut[reinterpret_cast<physics_object *>(0x6)] == nullptr);
}

/* Update collisions test */
BOOST_AUTO_TEST_CASE( update_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s1 = new simplex(*data1, true);
    simplex *s2 = new simplex(*data0, true);

    /* Construct and test */
    /* This is a bit naughty, but dont actaully need a class here */
    physics_object *vg0 = reinterpret_cast<physics_object *>(0x5);
    tracking_info<physics_object> uut(vg0, s0, *s1, 0.5f, collision_t::COLLISION);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg0]->get_simplex().get()       == s0);
    BOOST_CHECK(uut[vg0]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg0]->get_point_of_collision()  == point_t<>(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut[vg0]->get_type()                == collision_t::COLLISION);
    BOOST_CHECK(uut[vg0]->get_time()                == 0.5f);

    /* Update existing and check */
    uut.update(vg0, s1, *s2, 1.5f, collision_t::POSSIBLE_COLLISION);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 1.5f);

    BOOST_CHECK(uut[vg0]->get_simplex().get()       == s1);
    BOOST_CHECK(uut[vg0]->get_normal_of_collision() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut[vg0]->get_point_of_collision()  == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut[vg0]->get_type()                == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut[vg0]->get_time()                == 1.5f);

    /* Add a new collision and test */
    physics_object *vg1 = reinterpret_cast<physics_object *>(0x10);
    uut.update(vg1, s2, *s1, 0.7f, collision_t::SLIDING_COLLISION);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg1);
    BOOST_CHECK(fabs(uut.get_first_collision_time() - 0.7f) < result_tolerance);

    BOOST_CHECK(uut[vg1]->get_simplex().get()       == s2);
    BOOST_CHECK(uut[vg1]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg1]->get_point_of_collision()  == point_t<>(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut[vg1]->get_type()                == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(fabs(uut[vg1]->get_time() - 0.7f) < result_tolerance);
}

/* Void collision test */
BOOST_AUTO_TEST_CASE( void_collsion_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s1 = new simplex(*data1, true);
    simplex *s2 = new simplex(*data0, true);
    std::unique_ptr<simplex> s3(new simplex(*data1, true));

    /* Construct and test */
    physics_object *vg0 = reinterpret_cast<physics_object *>(0x5);
    tracking_info<physics_object> uut(vg0, s0, *s1, 0.5f, collision_t::COLLISION);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg0]->get_simplex().get()       == s0);
    BOOST_CHECK(uut[vg0]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg0]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg0]->get_type()                == collision_t::COLLISION);
    BOOST_CHECK(uut[vg0]->get_time()                == 0.5f);

    /* Add a new collision and test */
    physics_object *vg1 = reinterpret_cast<physics_object *>(0x10);
    uut.update(vg1, s1, *s2, 0.7f, collision_t::SLIDING_COLLISION);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg1]->get_simplex().get()       == s1);
    BOOST_CHECK(uut[vg1]->get_normal_of_collision() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut[vg1]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg1]->get_type()                == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(fabs(uut[vg1]->get_time() - 0.7f) < result_tolerance);

    /* Remove non-existant and check */
    uut.void_collision(reinterpret_cast<physics_object *>(0x15));

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg0]->get_simplex().get()       == s0);
    BOOST_CHECK(uut[vg0]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg0]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg0]->get_time()                == 0.5f);
    BOOST_CHECK(uut[vg0]->get_type()                == collision_t::COLLISION);

    BOOST_CHECK(uut[vg1]->get_simplex().get()       == s1);
    BOOST_CHECK(uut[vg1]->get_normal_of_collision() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut[vg1]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg1]->get_type()                == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(fabs(uut[vg1]->get_time() - 0.7f) < result_tolerance);

    /* Remove nearest and check */
    uut.void_collision(vg1);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg0]->get_simplex().get()       == s0);
    BOOST_CHECK(uut[vg0]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg0]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg0]->get_time()                == 0.5f);
    BOOST_CHECK(uut[vg0]->get_type()                == collision_t::COLLISION);

    /* Add a new collision and test */
    uut.update(vg1, s2, *s3, 0.6f, collision_t::SLIDING_COLLISION);

    /* Remove furthest and check */
    uut.void_collision(vg0);
    
    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg1);
    BOOST_CHECK(fabs(uut.get_first_collision_time() - 0.6f) < result_tolerance);

    BOOST_CHECK(uut[vg1]->get_simplex().get()       == s2);
    BOOST_CHECK(uut[vg1]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg1]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg1]->get_type()                == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(fabs(uut[vg1]->get_time() - 0.6f) < result_tolerance);
}

/* Void all and update */
BOOST_AUTO_TEST_CASE( void_all_and_update_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s2 = new simplex(*data0, true);
    std::unique_ptr<simplex> s1(new simplex(*data1, true));

    /* Construct and test */
    physics_object *vg0 = reinterpret_cast<physics_object *>(0x5);
    tracking_info<physics_object> uut(vg0, s0, *s1, 0.5f, collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg0]->get_simplex().get()       == s0);
    BOOST_CHECK(uut[vg0]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg0]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg0]->get_time()                == 0.5f);
    BOOST_CHECK(uut[vg0]->get_type()                == collision_t::COLLISION);

    /* Void and check */
    uut.void_collision(vg0);
    BOOST_CHECK(uut.get_first_collision() == nullptr);

    /* Update and check */
    physics_object *vg1 = reinterpret_cast<physics_object *>(0x10);
    uut.update(vg1, s2, *s1, 0.6f, collision_t::SLIDING_COLLISION);
    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg1);
    BOOST_CHECK(fabs(uut.get_first_collision_time() - 0.6f) < result_tolerance);

    BOOST_CHECK(uut[vg1]->get_simplex().get()       == s2);
    BOOST_CHECK(uut[vg1]->get_normal_of_collision() == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg1]->get_point_of_collision()  == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg1]->get_type()                == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(fabs(uut[vg1]->get_time() - 0.6f) < result_tolerance);
}

/* Test successful retest update */
BOOST_AUTO_TEST_CASE( successful_retest_update_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s2 = new simplex(*data0, true);
    std::unique_ptr<simplex> s1(new simplex(*data1, true));

    /* Construct and check */
    physics_object *vg = reinterpret_cast<physics_object *>(0x5);
    tracking_info<physics_object> uut(vg, s0, *s1, 0.5f, collision_t::POSSIBLE_COLLISION);

    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg]->get_simplex().get()        == s0);
    BOOST_CHECK(uut[vg]->get_normal_of_collision()  == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut[vg]->get_point_of_collision()   == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut[vg]->get_type()                 == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut[vg]->get_time()                 == 0.5f);

    /* Update and test */
    uut.successful_retest_update(vg, s2, *s1);
    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    BOOST_CHECK(uut[vg]->get_simplex().get()        == s2);
    BOOST_CHECK(uut[vg]->get_normal_of_collision()  == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut[vg]->get_point_of_collision()   == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut[vg]->get_type()                 == collision_t::COLLISION);
    BOOST_CHECK(uut[vg]->get_time()                 == 0.5f);
}

/* Test iteration */
BOOST_AUTO_TEST_CASE( iter_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s1 = new simplex(*data1, true);
    simplex *s2 = new simplex(*data0, true);
    simplex *s3 = new simplex(*data1, true);

    /* Construct and check a little */
    physics_object *vg0 = reinterpret_cast<physics_object *>(0x5);
    tracking_info<physics_object> uut(vg0, s0, *s1, 0.5f, collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg0);
    BOOST_CHECK(uut.get_first_collision_time()  == 0.5f);

    /* Update a bit and check */
    physics_object *vg1 = reinterpret_cast<physics_object *>(0x15);
    uut.update(vg1, s2, *s1, 0.6f, collision_t::POSSIBLE_COLLISION);

    physics_object *vg2 = reinterpret_cast<physics_object *>(0x20);
    uut.update(vg2, s1, *s0, 0.7f, collision_t::SLIDING_COLLISION);

    physics_object *vg3 = reinterpret_cast<physics_object *>(0x10);
    uut.update(vg3, s3, *s2, 0.3f, collision_t::POSSIBLE_SLIDING_COLLISION);
    BOOST_CHECK(uut.get_first_collision_type()  == collision_t::POSSIBLE_SLIDING_COLLISION);
    BOOST_CHECK(uut.get_first_collision()       == vg3);
    BOOST_CHECK(fabs(uut.get_first_collision_time() - 0.3f) < result_tolerance);

    /* Iterate checking */
    auto coll_iter = uut.begin();
    BOOST_CHECK(coll_iter->first == vg0);
    BOOST_CHECK(coll_iter->second->get_simplex().get()          == s0);
    BOOST_CHECK(coll_iter->second->get_normal_of_collision()    == point_t<>(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(coll_iter->second->get_point_of_collision()     == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(coll_iter->second->get_type()                   == collision_t::COLLISION);
    BOOST_CHECK(coll_iter->second->get_time()                   == 0.5f);

    ++coll_iter;
    BOOST_CHECK(coll_iter->first == vg3);
    BOOST_CHECK(coll_iter->second->get_simplex().get()          == s3);
    BOOST_CHECK(coll_iter->second->get_normal_of_collision()    == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(coll_iter->second->get_point_of_collision()     == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(coll_iter->second->get_type()                   == collision_t::POSSIBLE_SLIDING_COLLISION);
    BOOST_CHECK(fabs(coll_iter->second->get_time() - 0.3f) < result_tolerance);

    ++coll_iter;
    BOOST_CHECK(coll_iter->first == vg1);
    BOOST_CHECK(coll_iter->second->get_simplex().get()          == s2);
    BOOST_CHECK(coll_iter->second->get_normal_of_collision()    == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(coll_iter->second->get_point_of_collision()     == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(coll_iter->second->get_type()                   == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(fabs(coll_iter->second->get_time() - 0.6f) < result_tolerance);

    ++coll_iter;
    BOOST_CHECK(coll_iter->first == vg2);
    BOOST_CHECK(coll_iter->second->get_simplex().get()          == s1);
    BOOST_CHECK(coll_iter->second->get_normal_of_collision()    == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(coll_iter->second->get_point_of_collision()     == point_t<>(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(coll_iter->second->get_type()                   == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(fabs(coll_iter->second->get_time() - 0.7f) < result_tolerance);

    ++coll_iter;
    BOOST_CHECK(coll_iter == uut.end());
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
