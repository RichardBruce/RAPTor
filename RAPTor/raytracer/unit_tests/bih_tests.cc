#ifdef STAND_ALONE
#define BOOST_TEST_MODULE bih test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */

/* Boost headerrs */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "phong_shader.h"
#include "bih.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001f;

struct bih_fixture
{
    bih_fixture() :
    mat(new phong_shader(ext_colour_t(255.0f, 255.0f, 255.0f))), x_tris(6), y_tris(6), z_tris(6)
    {
        /* Set up for x direction tests */
        x_tris[0] = new triangle(mat.get(), point_t( 0.0f, 0.0f, 0.0f), point_t( 0.0f, 9.0f, 0.0f), point_t( 0.0f, 9.0f, 9.0f), false);
        x_tris[1] = new triangle(mat.get(), point_t( 0.0f, 0.0f, 0.0f), point_t( 0.0f, 9.0f, 9.0f), point_t( 0.0f, 0.0f, 9.0f), false);
        
        x_tris[2] = new triangle(mat.get(), point_t( 1.0f, 0.0f, 0.0f), point_t( 1.0f, 2.0f, 0.0f), point_t( 1.0f, 2.0f, 2.0f), false);
        x_tris[3] = new triangle(mat.get(), point_t( 1.0f, 0.0f, 0.0f), point_t( 1.0f, 2.0f, 2.0f), point_t( 1.0f, 0.0f, 2.0f), false);

        x_tris[4] = new triangle(mat.get(), point_t(10.0f, 3.0f, 0.0f), point_t(10.0f, 5.0f, 0.0f), point_t(10.0f, 5.0f, 2.0f), false);
        x_tris[5] = new triangle(mat.get(), point_t(10.0f, 3.0f, 0.0f), point_t(10.0f, 5.0f, 2.0f), point_t(10.0f, 3.0f, 2.0f), false);
        x_tris_orig = x_tris;

        /* Set up for y direction tests */
        y_tris[0] = new triangle(mat.get(), point_t(0.0f,  0.0f, 0.0f), point_t(9.0f,  0.0f, 0.0f), point_t(9.0f,  0.0f, 9.0f), false);
        y_tris[1] = new triangle(mat.get(), point_t(0.0f,  0.0f, 0.0f), point_t(9.0f,  0.0f, 9.0f), point_t(0.0f,  0.0f, 9.0f), false);
        
        y_tris[2] = new triangle(mat.get(), point_t(0.0f,  1.0f, 0.0f), point_t(2.0f,  1.0f, 0.0f), point_t(2.0f,  1.0f, 2.0f), false);
        y_tris[3] = new triangle(mat.get(), point_t(0.0f,  1.0f, 0.0f), point_t(2.0f,  1.0f, 2.0f), point_t(0.0f,  1.0f, 2.0f), false);

        y_tris[4] = new triangle(mat.get(), point_t(3.0f, 10.0f, 0.0f), point_t(5.0f, 10.0f, 0.0f), point_t(5.0f, 10.0f, 2.0f), false);
        y_tris[5] = new triangle(mat.get(), point_t(3.0f, 10.0f, 0.0f), point_t(5.0f, 10.0f, 2.0f), point_t(3.0f, 10.0f, 2.0f), false);
        y_tris_orig = y_tris;

        /* Set up for z direction tests */
        z_tris[0] = new triangle(mat.get(), point_t(0.0f, 0.0f,  0.0f), point_t(9.0f, 0.0f,  0.0f), point_t(9.0f, 9.0f,  0.0f), false);
        z_tris[1] = new triangle(mat.get(), point_t(0.0f, 0.0f,  0.0f), point_t(9.0f, 9.0f,  0.0f), point_t(0.0f, 9.0f,  0.0f), false);
        
        z_tris[2] = new triangle(mat.get(), point_t(0.0f, 0.0f,  1.0f), point_t(2.0f, 0.0f,  1.0f), point_t(2.0f, 2.0f,  1.0f), false);
        z_tris[3] = new triangle(mat.get(), point_t(0.0f, 0.0f,  1.0f), point_t(2.0f, 2.0f,  1.0f), point_t(0.0f, 2.0f,  1.0f), false);

        z_tris[4] = new triangle(mat.get(), point_t(3.0f, 0.0f, 10.0f), point_t(5.0f, 0.0f, 10.0f), point_t(5.0f, 2.0f, 10.0f), false);
        z_tris[5] = new triangle(mat.get(), point_t(3.0f, 0.0f, 10.0f), point_t(5.0f, 2.0f, 10.0f), point_t(3.0f, 2.0f, 10.0f), false);
        z_tris_orig = z_tris;
    }

    ~bih_fixture()
    {
        for (auto *t : x_tris)
        {
            delete t;
        }

        for (auto *t : y_tris)
        {
            delete t;
        }

        for (auto *t : z_tris)
        {
            delete t;
        }
    }

    std::unique_ptr<material>   mat;
    std::vector<triangle *>     x_tris;
    std::vector<triangle *>     x_tris_orig;
    std::vector<triangle *>     y_tris;
    std::vector<triangle *>     y_tris_orig;
    std::vector<triangle *>     z_tris;
    std::vector<triangle *>     z_tris_orig;
};

BOOST_FIXTURE_TEST_SUITE( bih_tests, bih_fixture );

BOOST_AUTO_TEST_CASE( x_ray_miss_world_test )
{
    /* Build BIH */
    bih uut(x_tris, 4);

    /* Completely miss the world */
    hit_description h;
    ray r(point_t(15.0f, 9.1f, 0.0f), -1.0f, 0.0f, 0.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( x_ray_away_from_world_test )
{
    /* Build BIH */
    bih uut(x_tris, 4);

    /* Completely miss the world */
    hit_description h;
    ray r(point_t(20.0f, 5.0f, 0.0f), 1.0f, 0.0f, 0.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( x_ray_pass_between_test )
{
    /* Build BIH */
    bih uut(x_tris, 4);

    /* Check we dont hit anything in the middle of the partition */
    hit_description h;
    ray r(point_t(5.0f, 5.0f, -10.0f), 0.0f, 0.0f, 1.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 100.0f));
}

BOOST_AUTO_TEST_CASE( x_ray_stuck_in_middle_test )
{
    /* Build BIH */
    bih uut(x_tris, 4);

    /* Check the ray doesnt reach a node */
    hit_description h;
    ray r(point_t(5.0f, 5.0f, 0.0f), 1.0f, 0.0f, 0.0f);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( x_ray_hit_in_near_test )
{
    /* Build BIH */
    bih uut(x_tris, 4);

    /* Check we hit the small near triangle */
    hit_description h;
    ray r(point_t(15.0f, 4.0f, 0.1f), -1.0f, 0.0f, 0.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == x_tris_orig[4]);
    BOOST_CHECK_CLOSE(h.d, 5.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 5.1f));
}

BOOST_AUTO_TEST_CASE( x_ray_hit_in_near_reverse_test )
{
    /* Build BIH */
    bih uut(x_tris, 4);

    /* Check we hit the small near triangle */
    hit_description h;
    ray r(point_t(-15.0f, 4.0f, 0.1f), 1.0f, 0.0f, 0.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == x_tris_orig[0]);
    BOOST_CHECK_CLOSE(h.d, 15.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 15.1f));
}

BOOST_AUTO_TEST_CASE( x_ray_hit_in_far_test )
{
    /* Build BIH */
    bih uut(x_tris, 4);

    /* Check we hit the far large triangle  */
    hit_description h;
    ray r(point_t(15.0f, 8.0f, 0.1f), -1.0f, 0.0f, 0.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == x_tris_orig[0]);
    BOOST_CHECK_CLOSE(h.d, 15.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 15.1f));
}

BOOST_AUTO_TEST_CASE( y_ray_miss_world_test )
{
    /* Build BIH */
    bih uut(y_tris, 4);

    /* Completely miss the world */
    hit_description h;
    ray r(point_t(9.1f, 15.0f, 0.0f), 0.0f, -1.0f, 0.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( y_ray_away_from_world_test )
{
    /* Build BIH */
    bih uut(y_tris, 4);

    /* Completely miss the world */
    hit_description h;
    ray r(point_t(5.0f, 20.0f, 0.0f), 0.0f, 1.0f, 0.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( y_ray_pass_between_test )
{
    /* Build BIH */
    bih uut(y_tris, 4);

    /* Check we dont hit anything in the middle of the partition */
    hit_description h;
    ray r(point_t(5.0f, 5.0f, -10.0f), 0.0f, 0.0f, 1.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 100.0f));
}

BOOST_AUTO_TEST_CASE( y_ray_stuck_in_middle_test )
{
    /* Build BIH */
    bih uut(y_tris, 4);

    /* Check the ray doesnt reach a node */
    hit_description h;
    ray r(point_t(5.0f, 5.0f, 0.0f), 0.0f, 1.0f, 0.0f);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( y_ray_hit_in_near_test )
{
    /* Build BIH */
    bih uut(y_tris, 4);

    /* Check we hit the small near triangle */
    hit_description h;
    ray r(point_t(4.0f, 15.0f, 0.1f), 0.0f, -1.0f, 0.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == y_tris_orig[4]);
    BOOST_CHECK_CLOSE(h.d, 5.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 5.1f));
}

BOOST_AUTO_TEST_CASE( y_ray_hit_in_near_reverse_test )
{
    /* Build BIH */
    bih uut(y_tris, 4);

    /* Check we hit the small near triangle */
    hit_description h;
    ray r(point_t(4.0f, -15.0f, 0.1f), 0.0f, 1.0f, 0.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == y_tris_orig[0]);
    BOOST_CHECK_CLOSE(h.d, 15.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 15.1f));
}

BOOST_AUTO_TEST_CASE( y_ray_hit_in_far_test )
{
    /* Build BIH */
    bih uut(y_tris, 4);

    /* Check we hit the far large triangle  */
    hit_description h;
    ray r(point_t(8.0f, 15.0f, 0.1f), 0.0f, -1.0f, 0.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == y_tris_orig[0]);
    BOOST_CHECK_CLOSE(h.d, 15.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 15.1f));
}

BOOST_AUTO_TEST_CASE( z_ray_miss_world_test )
{
    /* Build BIH */
    bih uut(z_tris, 4);

    /* Completely miss the world */
    hit_description h;
    ray r(point_t(9.1f, 0.0f, 15.0f), 0.0f, 0.0f, -1.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( z_ray_away_from_world_test )
{
    /* Build BIH */
    bih uut(z_tris, 4);

    /* Completely miss the world */
    hit_description h;
    ray r(point_t(5.0f, 0.0f, 20.0f), 0.0f, 0.0f, 1.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( z_ray_pass_between_test )
{
    /* Build BIH */
    bih uut(z_tris, 4);

    /* Check we dont hit anything in the middle of the partition */
    hit_description h;
    ray r(point_t(5.0f, -10.0f, 5.0f), 0.0f, 1.0f, 0.0f);
    BOOST_CHECK( uut.find_nearest_object(&r, &h) == nullptr);
    BOOST_CHECK(!uut.found_nearer_object(&r, 100.0f));
}

BOOST_AUTO_TEST_CASE( z_ray_stuck_in_middle_test )
{
    /* Build BIH */
    bih uut(z_tris, 4);

    /* Check the ray doesnt reach a node */
    hit_description h;
    ray r(point_t(5.0f, 0.0f, 5.0f), 0.0f, 0.0f, 1.0f);
    BOOST_CHECK(!uut.found_nearer_object(&r, 1.0f));
}

BOOST_AUTO_TEST_CASE( z_ray_hit_in_near_test )
{
    /* Build BIH */
    bih uut(z_tris, 4);

    /* Check we hit the small near triangle */
    hit_description h;
    ray r(point_t(4.0f, 0.1f, 15.0f), 0.0f, 0.0f, -1.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == z_tris_orig[4]);
    BOOST_CHECK_CLOSE(h.d, 5.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 5.1f));
}

BOOST_AUTO_TEST_CASE( z_ray_hit_in_near_reverse_test )
{
    /* Build BIH */
    bih uut(z_tris, 4);

    /* Check we hit the small near triangle */
    hit_description h;
    ray r(point_t(4.0f, 0.1f, -15.0f), 0.0f, 0.0f, 1.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == z_tris_orig[0]);
    BOOST_CHECK_CLOSE(h.d, 15.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 15.1f));
}

BOOST_AUTO_TEST_CASE( z_ray_hit_in_far_test )
{
    /* Build BIH */
    bih uut(z_tris, 4);

    /* Check we hit the far large triangle  */
    hit_description h;
    ray r(point_t(8.0f, 0.1f, 15.0f), 0.0f, 0.0f, -1.0f);
    BOOST_CHECK(uut.find_nearest_object(&r, &h) == z_tris_orig[0]);
    BOOST_CHECK_CLOSE(h.d, 15.0f, result_tolerance);

    /* Check we hit this for found neearer as well */
    BOOST_CHECK(uut.found_nearer_object(&r, 15.1f));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
