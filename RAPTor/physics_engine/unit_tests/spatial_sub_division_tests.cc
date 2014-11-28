#ifdef STAND_ALONE
#define BOOST_TEST_MODULE spatial_sub_division test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <chrono>
#include <cmath>
#include <random>
#include <unordered_map>

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Raytracer headers */
#include "phong_shader.h"

/* Physics headers */
#include "spatial_sub_division.h"


using namespace raptor_physics;

/* Test data */
struct spatial_sub_division_fixture : private boost::noncopyable
{
    spatial_sub_division_fixture()
    : mat( new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0)),
      po0( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t( 1.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po1( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 2.0, 0.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po2( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 4.0, 0.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po3( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 6.0, 0.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po4( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 8.0, 0.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po5( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t(10.0, 0.0, 0.0), point_t(-1.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po6( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 0.0, 1.0, 0.0), point_t( 0.0, 1.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po7( new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 1.0, 1.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po8( new physics_object(make_cube(mat, point_t(-1.5, -0.5f, -0.5f), point_t(1.5,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 2.0, 1.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po9( new physics_object(make_cube(mat, point_t(-4.5, -0.5f, -0.5f), point_t(4.5,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 5.0, 1.0, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po10(new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 0.0, 1.5, 0.0), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po11(new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 0.0, 0.0, 1.5), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po12(new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 0.0, 1.0, 1.5), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po13(new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 1.0, 1.0, 1.5), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      po14(new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 1.0, 0.0, 1.5), point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0)),
      objects0(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po5.get() }
        }),
      objects1(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po6.get() }
        }),
      objects2(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po7.get() }
        }),
      objects3(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po8.get() }
        }),
      objects4(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po6.get() }, { 6, po7.get() }, { 7, po8.get() }
        }),
      objects5(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po10.get() }
        }),
      objects6(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po11.get() }
        }),
      objects7(
        {
            { 0, po0.get() }, { 1, po1.get() }, { 2, po2.get() }, { 3, po3.get() }, { 4, po4.get() }, { 5, po11.get() }, { 6, po12.get() }, { 7, po13.get() }, { 8, po14.get() }
        }),
      generator(),
      normal_dist(1.0f, 0.1f)
      {  };

    ~spatial_sub_division_fixture()
    {
        delete mat;
    }

    raptor_raytracer::material *const           mat;
    std::unique_ptr<physics_object>             po0;
    std::unique_ptr<physics_object>             po1;
    std::unique_ptr<physics_object>             po2;
    std::unique_ptr<physics_object>             po3;
    std::unique_ptr<physics_object>             po4;
    std::unique_ptr<physics_object>             po5;
    std::unique_ptr<physics_object>             po6;
    std::unique_ptr<physics_object>             po7;
    std::unique_ptr<physics_object>             po8;
    std::unique_ptr<physics_object>             po9;
    std::unique_ptr<physics_object>             po10;
    std::unique_ptr<physics_object>             po11;
    std::unique_ptr<physics_object>             po12;
    std::unique_ptr<physics_object>             po13;
    std::unique_ptr<physics_object>             po14;
    std::unordered_map<int, physics_object*>    objects0;
    std::unordered_map<int, physics_object*>    objects1;
    std::unordered_map<int, physics_object*>    objects2;
    std::unordered_map<int, physics_object*>    objects3;
    std::unordered_map<int, physics_object*>    objects4;
    std::unordered_map<int, physics_object*>    objects5;
    std::unordered_map<int, physics_object*>    objects6;
    std::unordered_map<int, physics_object*>    objects7;
    std::default_random_engine                  generator;
    std::normal_distribution<float>             normal_dist;
#ifndef VALGRIND_TESTS
    const int number_of_objects                 = 10000;
    const int number_of_updates                 = 100000;
#else
    const int number_of_objects                 = 1000;
    const int number_of_updates                 = 10000;
#endif
};

BOOST_FIXTURE_TEST_SUITE( spatial_sub_division_tests, spatial_sub_division_fixture );


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_no_collisions_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);
}

BOOST_AUTO_TEST_CASE( ctor_one_collision_test )
{
    /* Construct, one collision exact overlap */
    spatial_sub_division uut(objects1);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po6.get()), std::max(po0.get(), po6.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( ctor_two_collisions_test )
{
    /* Construct, in the middle just touching */
    spatial_sub_division uut(objects2);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 2);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po7.get()), std::max(po0.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po7.get()), std::max(po1.get(), po7.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( ctor_three_collisions_test )
{
    /* Construct, complete overlap */
    spatial_sub_division uut(objects3);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 3);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po8.get()), std::max(po0.get(), po8.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po8.get()), std::max(po1.get(), po8.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po2.get(), po8.get()), std::max(po2.get(), po8.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( ctor_nine_collisions_test )
{
    /* Construct, all of the above in one */
    spatial_sub_division uut(objects4);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 9);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po6.get()), std::max(po0.get(), po6.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po7.get()), std::max(po0.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po7.get()), std::max(po1.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po8.get()), std::max(po0.get(), po8.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po8.get()), std::max(po1.get(), po8.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po2.get(), po8.get()), std::max(po2.get(), po8.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po7.get()), std::max(po6.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po8.get()), std::max(po6.get(), po8.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po7.get(), po8.get()), std::max(po7.get(), po8.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( ctor_no_collisions_offset_in_y_test )
{
    /* Construct */
    spatial_sub_division uut(objects5);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);
}

BOOST_AUTO_TEST_CASE( ctor_no_collisions_offset_in_x_test )
{
    /* Construct */
    spatial_sub_division uut(objects6);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);
}


BOOST_AUTO_TEST_CASE( ctor_z_collisions_test )
{
    /* Construct, offset in z and just touching eachother */
    spatial_sub_division uut(objects7);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 6);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po11.get(), po12.get()), std::max(po11.get(), po12.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po11.get(), po13.get()), std::max(po11.get(), po13.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po11.get(), po14.get()), std::max(po11.get(), po14.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po12.get(), po13.get()), std::max(po12.get(), po13.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po12.get(), po14.get()), std::max(po12.get(), po14.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po13.get(), po14.get()), std::max(po13.get(), po14.get()))) != uut.possible_collisions().end());
}

/* Test add_object */
BOOST_AUTO_TEST_CASE( add_object_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);

    /* Add object */
    uut.add_object(*po6);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po0.get()), std::max(po6.get(), po0.get()))) != uut.possible_collisions().end());

    /* Add object */
    uut.add_object(*po7);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 4);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po0.get()), std::max(po6.get(), po0.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po7.get()), std::max(po0.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po7.get()), std::max(po1.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po7.get()), std::max(po6.get(), po7.get()))) != uut.possible_collisions().end());

    /* Add object */
    uut.add_object(*po9);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 12);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po0.get()), std::max(po6.get(), po0.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po7.get()), std::max(po0.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po7.get()), std::max(po1.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po7.get()), std::max(po6.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po9.get()), std::max(po0.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po9.get()), std::max(po1.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po2.get(), po9.get()), std::max(po2.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po3.get(), po9.get()), std::max(po3.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po4.get(), po9.get()), std::max(po4.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po5.get(), po9.get()), std::max(po5.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po9.get()), std::max(po6.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po7.get(), po9.get()), std::max(po7.get(), po9.get()))) != uut.possible_collisions().end());
}

/* Test remove_object */
BOOST_AUTO_TEST_CASE( remove_object_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);
    uut.add_object(*po6);
    uut.add_object(*po7);
    uut.add_object(*po9);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 12);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po0.get()), std::max(po6.get(), po0.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po7.get()), std::max(po0.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po7.get()), std::max(po1.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po7.get()), std::max(po6.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po9.get()), std::max(po0.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po9.get()), std::max(po1.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po2.get(), po9.get()), std::max(po2.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po3.get(), po9.get()), std::max(po3.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po4.get(), po9.get()), std::max(po4.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po5.get(), po9.get()), std::max(po5.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po9.get()), std::max(po6.get(), po9.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po7.get(), po9.get()), std::max(po7.get(), po9.get()))) != uut.possible_collisions().end());

    /* Add object */
    uut.remove_object(po9.get());

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 4);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po0.get()), std::max(po6.get(), po0.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po7.get()), std::max(po0.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po1.get(), po7.get()), std::max(po1.get(), po7.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po7.get()), std::max(po6.get(), po7.get()))) != uut.possible_collisions().end());

    /* Add object */
    uut.remove_object(po7.get());

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po6.get(), po0.get()), std::max(po6.get(), po0.get()))) != uut.possible_collisions().end());

    /* Add object */
    uut.remove_object(po6.get());

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);
}


/* Test update_object */
BOOST_AUTO_TEST_CASE( update_object_move_just_touching_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);

    /* Update */
    po0->begin_time_step(1.5);
    po0->commit_movement(1.5);
    uut.update_object(*po0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po1.get()), std::max(po0.get(), po1.get()))) != uut.possible_collisions().end());

    /* Update */
    po5->begin_time_step(1.5);
    po5->commit_movement(1.5);
    uut.update_object(*po5);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 2);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po1.get()), std::max(po0.get(), po1.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po4.get(), po5.get()), std::max(po4.get(), po5.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( update_object_move_just_touching_far_side_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);

    /* Update */
    po0->begin_time_step(2.5);
    po0->commit_movement(2.5);
    uut.update_object(*po0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po1.get()), std::max(po0.get(), po1.get()))) != uut.possible_collisions().end());

    /* Update */
    po5->begin_time_step(2.5);
    po5->commit_movement(2.5);
    uut.update_object(*po5);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 2);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po1.get()), std::max(po0.get(), po1.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po4.get(), po5.get()), std::max(po4.get(), po5.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( update_object_move_overlap_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);

    /* Update */
    po0->begin_time_step(2.0);
    po0->commit_movement(2.0);
    uut.update_object(*po0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po1.get()), std::max(po0.get(), po1.get()))) != uut.possible_collisions().end());

    /* Update */
    po5->begin_time_step(2.0);
    po5->commit_movement(2.0);
    uut.update_object(*po5);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 2);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po1.get()), std::max(po0.get(), po1.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po4.get(), po5.get()), std::max(po4.get(), po5.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( update_object_move_center_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);

    /* Update */
    po0->begin_time_step(4.0);
    po0->commit_movement(4.0);
    uut.update_object(*po0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po2.get()), std::max(po0.get(), po2.get()))) != uut.possible_collisions().end());

    /* Update */
    po5->begin_time_step(6.0);
    po5->commit_movement(6.0);
    uut.update_object(*po5);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 3);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po2.get()), std::max(po0.get(), po2.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po5.get()), std::max(po0.get(), po5.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po2.get(), po5.get()), std::max(po2.get(), po5.get()))) != uut.possible_collisions().end());
}

BOOST_AUTO_TEST_CASE( update_object_move_center_and_away_test )
{
    /* Construct */
    spatial_sub_division uut(objects0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);

    /* Update */
    po0->begin_time_step(4.0);
    po5->begin_time_step(6.0);
    po0->commit_movement(4.0);
    po5->commit_movement(6.0);
    uut.update_object(*po0);
    uut.update_object(*po5);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 3);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po2.get()), std::max(po0.get(), po2.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po5.get()), std::max(po0.get(), po5.get()))) != uut.possible_collisions().end());
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po2.get(), po5.get()), std::max(po2.get(), po5.get()))) != uut.possible_collisions().end());

    /* Update */
    po0->begin_time_step(10.0);
    po0->commit_movement(10.0);
    uut.update_object(*po0);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po2.get(), po5.get()), std::max(po2.get(), po5.get()))) != uut.possible_collisions().end());

    /* Update */
    po5->begin_time_step(10.0);
    po5->commit_movement(10.0);
    uut.update_object(*po5);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);
}

BOOST_AUTO_TEST_CASE( update_object_move_away_y_test )
{
    /* Construct, one collision exact overlap */
    spatial_sub_division uut(objects1);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 1);
    BOOST_CHECK(uut.possible_collisions().find(std::make_pair(std::min(po0.get(), po6.get()), std::max(po0.get(), po6.get()))) != uut.possible_collisions().end());

    /* Update */
    po6->begin_time_step(1.0);
    po6->commit_movement(1.0);
    uut.update_object(*po6);

    /* Checks */
    BOOST_CHECK(uut.number_of_possible_collisions() == 0);
}

BOOST_AUTO_TEST_CASE( contruct_performance_test )
{
    std::uniform_real_distribution<float> real_uniform_dist(0.0f, std::pow(number_of_objects, 1.0f / 3.0f) * 5.0f);

    /* Create some random objects on a grid moving in random directions */
    std::unordered_map<int, physics_object*> objects;
    for (int i = 0; i < number_of_objects; ++i)
    {
        const point_t pos(real_uniform_dist(generator), real_uniform_dist(generator), real_uniform_dist(generator));
        auto po = new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), pos, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), 1.0f);
        objects.emplace(i, po);
    }

    /* Construct */
    const auto t0(std::chrono::system_clock::now());
    spatial_sub_division uut(objects);
    const auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: Sort and Sweep Construct";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    /* Clean up */
    for (auto &p : objects)
    {
        delete p.second;
    }
}

BOOST_AUTO_TEST_CASE( update_object_performance_test )
{
    std::uniform_real_distribution<float> real_uniform_dist(0.0f, std::pow(number_of_objects, 1.0f / 3.0f) * 5.0f);
    std::uniform_int_distribution<int> int_uniform_dist(0, number_of_objects - 1);

    /* Create some random objects on a grid moving in random directions */
    std::unordered_map<int, physics_object*> objects;
    for (int i = 0; i < number_of_objects; ++i)
    {
        const point_t pos(real_uniform_dist(generator), real_uniform_dist(generator), real_uniform_dist(generator));
        const point_t vel(normalise(-pos) * point_t(normal_dist(generator), normal_dist(generator), normal_dist(generator)));
        auto po = new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), pos, vel, point_t(0.0f, 0.0f, 0.0f), 1.0f);
        objects.emplace(i, po);
    }

    /* Do some updates */
    spatial_sub_division uut(objects);
    const auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < number_of_updates; ++i)
    {
        const int obj       = int_uniform_dist(generator);
        const float t_step  = normal_dist(generator);
        objects[obj]->begin_time_step(t_step);
        uut.update_object(*objects[obj]);

        objects[obj]->commit_movement(t_step);
        uut.update_object(*objects[obj]);
    }
    const auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: Sort and Sweep Update Object";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    /* Clean up */
    for (auto &p : objects)
    {
        delete p.second;
    }
}

BOOST_AUTO_TEST_CASE( remove_object_performance_test )
{
    std::uniform_real_distribution<float> real_uniform_dist(0.0, std::pow(number_of_objects, 1.0f / 3.0f) * 5.0f);
    std::uniform_int_distribution<int> int_uniform_dist(0, number_of_objects - 1);

    /* Create some random objects on a grid moving in random directions */
    std::unordered_map<int, physics_object*> objects;
    for (int i = 0; i < number_of_objects; ++i)
    {
        const point_t pos(real_uniform_dist(generator), real_uniform_dist(generator), real_uniform_dist(generator));
        auto po = new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), pos, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), 1.0f);
        objects.emplace(i, po);
    }

    /* Remove 10%-ish of the objects */
    spatial_sub_division uut(objects);
    const auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < number_of_objects * 0.1f; ++i)
    {
        const int obj = int_uniform_dist(generator);
        auto obj_iter = objects.find(obj);
        if (obj_iter != objects.end())
        {
            uut.remove_object(obj_iter->second);
            delete obj_iter->second;
            objects.erase(obj_iter);
        }
    }
    const auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: Sort and Sweep Remove Object";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    /* Clean up */
    for (auto &p : objects)
    {
        delete p.second;
    }
}

BOOST_AUTO_TEST_CASE( add_object_performance_test )
{
    std::uniform_real_distribution<float> real_uniform_dist(0.0, std::pow(number_of_objects, 1.0f / 3.0f) * 5.0f);
    std::uniform_int_distribution<int> int_uniform_dist(0, number_of_objects - 1);

    /* Create 90% of some random objects on a grid moving in random directions */
    std::unordered_map<int, physics_object*> objects;
    int i = 0;
    for ( ; i < static_cast<int>(number_of_objects * 0.9f); ++i)
    {
        const point_t pos(real_uniform_dist(generator), real_uniform_dist(generator), real_uniform_dist(generator));
        auto po = new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), pos, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), 1.0f);
        objects.emplace(i, po);
    }

    /* Add the final 10% of the objects */
    spatial_sub_division uut(objects);
    const auto t0(std::chrono::system_clock::now());
    for ( ; i < number_of_objects; ++i)
    {
        const point_t pos(real_uniform_dist(generator), real_uniform_dist(generator), real_uniform_dist(generator));
        auto po = new physics_object(make_cube(mat, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f,  0.5f,  0.5f)), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), pos, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), 1.0f);
        objects.emplace(i, po);
        uut.add_object(*po);
    }
    const auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: Sort and Sweep Add Object";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    /* Clean up */
    for (auto &p : objects)
    {
        delete p.second;
    }
}

BOOST_AUTO_TEST_SUITE_END()
