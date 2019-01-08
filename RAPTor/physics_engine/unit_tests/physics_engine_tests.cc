#ifdef STAND_ALONE
#define BOOST_TEST_MODULE physics_engine test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Raytracer headers */
#include "phong_shader.h"

/* Physics headers */
#include "rigid_body_collider.h"
#include "physics_object.h"
#include "physics_engine.h"

/* Test headers */
#include "mock_force.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct physics_engine_fixture : private boost::noncopyable
{
    physics_engine_fixture()
    :  collider(new rigid_body_collider(0.9f, 0.0f, 0.75f)),
       uut(collider),
       m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), 1.0f)),
       po0_check(new physics_object(make_cube(m, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f,  9.5f, 0.0f), 10.0f)),
       po1_check(new physics_object(make_cube(m, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, -9.5f, 0.0f), 10.0f)),
       po2_check(new physics_object(make_cube(m, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, -8.4999975f, 0.0f), 10.0f)),
       po3_check(new physics_object(make_cube(m, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, -7.4999925f, 0.0f), 10.0f)),
       po4_check(new physics_object(make_cube(m, point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, 11.0f, 0.0f), 10.0f, 1)),
       po0(po0_check),
       po1(po1_check),
       po2(po2_check),
       po3(po3_check),
       po4(po4_check)
    {  };

    ~physics_engine_fixture()
    {
        delete m;
    }

    rigid_body_collider *collider;
    physics_engine uut;
    raptor_raytracer::material *m;

    physics_object *po0_check;
    physics_object *po1_check;
    physics_object *po2_check;
    physics_object *po3_check;
    physics_object *po4_check;
    std::unique_ptr<physics_object> po0;
    std::unique_ptr<physics_object> po1;
    std::unique_ptr<physics_object> po2;
    std::unique_ptr<physics_object> po3;
    std::unique_ptr<physics_object> po4;
};


BOOST_FIXTURE_TEST_SUITE( physics_engine_tests, physics_engine_fixture )

const float result_tolerance = 0.0005f;


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
}


/* Add object tests */
BOOST_AUTO_TEST_CASE( add_object_test )
{
    uut.add_object(po0.release());
    BOOST_CHECK(uut.number_of_objects() == 1);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 1);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.add_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
}

BOOST_AUTO_TEST_CASE( add_moving_object_test )
{
    uut.add_moving_object(po0.release());
    BOOST_CHECK(uut.number_of_objects() == 1);
    BOOST_CHECK(uut.number_of_moving_objects() == 1);
    BOOST_CHECK(uut.next_object_id() == 1);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.add_moving_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
}

BOOST_AUTO_TEST_CASE( add_no_cleaning_test )
{
    rigid_body_collider *const pe_collider = new rigid_body_collider(0.9f, 0.0f, 0.75f);
    physics_engine pe(pe_collider, false);
    BOOST_CHECK(pe.number_of_objects() == 0);
    BOOST_CHECK(pe.number_of_moving_objects() == 0);
    BOOST_CHECK(pe.next_object_id() == 0);
    BOOST_CHECK(pe.get_object(0) == nullptr);
    BOOST_CHECK(pe.number_of_collisions() == 0);
    BOOST_CHECK(pe.default_collider() == pe_collider);

    pe.add_object(po0.get());
    BOOST_CHECK(pe.number_of_objects() == 1);
    BOOST_CHECK(pe.number_of_moving_objects() == 0);
    BOOST_CHECK(pe.next_object_id() == 1);
    BOOST_CHECK(pe.get_object(0) == po0_check);
    BOOST_CHECK(pe.number_of_collisions() == 0);
    BOOST_CHECK(pe.default_collider() == pe_collider);

    pe.add_moving_object(po1.get());
    BOOST_CHECK(pe.number_of_objects() == 2);
    BOOST_CHECK(pe.number_of_moving_objects() == 1);
    BOOST_CHECK(pe.next_object_id() == 2);
    BOOST_CHECK(pe.get_object(0) == po0_check);
    BOOST_CHECK(pe.number_of_collisions() == 0);
    BOOST_CHECK(pe.default_collider() == pe_collider);
}


/* Remove object tests */
BOOST_AUTO_TEST_CASE( remove_object_test )
{
    uut.add_object(po0.release());
    uut.add_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.remove_object(0);
    BOOST_CHECK(uut.number_of_objects() == 1);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.remove_object(1);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.get_object(1) == nullptr);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
}

BOOST_AUTO_TEST_CASE( remove_moving_object_test )
{
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.remove_object(0);
    BOOST_CHECK(uut.number_of_objects() == 1);
    BOOST_CHECK(uut.number_of_moving_objects() == 1);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.remove_object(1);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.get_object(1) == nullptr);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
}

BOOST_AUTO_TEST_CASE( remove_mixed_object_test )
{
    uut.add_object(po0.release());
    uut.add_moving_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 1);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.remove_object(0);
    BOOST_CHECK(uut.number_of_objects() == 1);
    BOOST_CHECK(uut.number_of_moving_objects() == 1);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.remove_object(1);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.get_object(1) == nullptr);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
}


/* To triangles tests */
BOOST_AUTO_TEST_CASE( scene_to_triangles_test )
{
    uut.add_object(po0.release());
    uut.add_moving_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 1);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    std::unique_ptr<raptor_raytracer::primitive_store> tris(uut.scene_to_triangles());
    BOOST_CHECK(tris->size() == 24);

    BOOST_CHECK(tris->primitive( 0)->get_vertex_a() == point_t(-0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 0)->get_vertex_b() == point_t( 0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 0)->get_vertex_c() == point_t( 0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 1)->get_vertex_a() == point_t(-0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 1)->get_vertex_b() == point_t(-0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 1)->get_vertex_c() == point_t( 0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 2)->get_vertex_a() == point_t(-0.5f, -10.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 2)->get_vertex_b() == point_t( 0.5f, -10.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 2)->get_vertex_c() == point_t( 0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 3)->get_vertex_a() == point_t(-0.5f, -10.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 3)->get_vertex_b() == point_t( 0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 3)->get_vertex_c() == point_t(-0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 4)->get_vertex_a() == point_t(-0.5f, -10.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 4)->get_vertex_b() == point_t(-0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 4)->get_vertex_c() == point_t(-0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 5)->get_vertex_a() == point_t(-0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 5)->get_vertex_b() == point_t(-0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 5)->get_vertex_c() == point_t(-0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 6)->get_vertex_a() == point_t( 0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 6)->get_vertex_b() == point_t( 0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 6)->get_vertex_c() == point_t( 0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 7)->get_vertex_a() == point_t( 0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 7)->get_vertex_b() == point_t( 0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 7)->get_vertex_c() == point_t( 0.5f, -10.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 8)->get_vertex_a() == point_t(-0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 8)->get_vertex_b() == point_t(-0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 8)->get_vertex_c() == point_t( 0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 9)->get_vertex_a() == point_t(-0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive( 9)->get_vertex_b() == point_t( 0.5f,  -9.0f,  0.5f));
    BOOST_CHECK(tris->primitive( 9)->get_vertex_c() == point_t( 0.5f,  -9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(10)->get_vertex_a() == point_t(-0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(10)->get_vertex_b() == point_t( 0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(10)->get_vertex_c() == point_t(-0.5f, -10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(11)->get_vertex_a() == point_t( 0.5f, -10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(11)->get_vertex_b() == point_t( 0.5f, -10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(11)->get_vertex_c() == point_t(-0.5f, -10.0f,  0.5f));

    BOOST_CHECK(tris->primitive(12)->get_vertex_a() == point_t(-0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(12)->get_vertex_b() == point_t( 0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(12)->get_vertex_c() == point_t( 0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(13)->get_vertex_a() == point_t(-0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(13)->get_vertex_b() == point_t(-0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(13)->get_vertex_c() == point_t( 0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(14)->get_vertex_a() == point_t(-0.5f,  9.0f,  0.5f));
    BOOST_CHECK(tris->primitive(14)->get_vertex_b() == point_t( 0.5f,  9.0f,  0.5f));
    BOOST_CHECK(tris->primitive(14)->get_vertex_c() == point_t( 0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(15)->get_vertex_a() == point_t(-0.5f,  9.0f,  0.5f));
    BOOST_CHECK(tris->primitive(15)->get_vertex_b() == point_t( 0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(15)->get_vertex_c() == point_t(-0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(16)->get_vertex_a() == point_t(-0.5f,  9.0f,  0.5f));
    BOOST_CHECK(tris->primitive(16)->get_vertex_b() == point_t(-0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(16)->get_vertex_c() == point_t(-0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(17)->get_vertex_a() == point_t(-0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(17)->get_vertex_b() == point_t(-0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(17)->get_vertex_c() == point_t(-0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(18)->get_vertex_a() == point_t( 0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(18)->get_vertex_b() == point_t( 0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(18)->get_vertex_c() == point_t( 0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(19)->get_vertex_a() == point_t( 0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(19)->get_vertex_b() == point_t( 0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(19)->get_vertex_c() == point_t( 0.5f,  9.0f,  0.5f));
    BOOST_CHECK(tris->primitive(20)->get_vertex_a() == point_t(-0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(20)->get_vertex_b() == point_t(-0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(20)->get_vertex_c() == point_t( 0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(21)->get_vertex_a() == point_t(-0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(21)->get_vertex_b() == point_t( 0.5f, 10.0f,  0.5f));
    BOOST_CHECK(tris->primitive(21)->get_vertex_c() == point_t( 0.5f, 10.0f, -0.5f));
    BOOST_CHECK(tris->primitive(22)->get_vertex_a() == point_t(-0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(22)->get_vertex_b() == point_t( 0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(22)->get_vertex_c() == point_t(-0.5f,  9.0f,  0.5f));
    BOOST_CHECK(tris->primitive(23)->get_vertex_a() == point_t( 0.5f,  9.0f, -0.5f));
    BOOST_CHECK(tris->primitive(23)->get_vertex_b() == point_t( 0.5f,  9.0f,  0.5f));
    BOOST_CHECK(tris->primitive(23)->get_vertex_c() == point_t(-0.5f,  9.0f,  0.5f));
}


/* Apply force tests */
BOOST_AUTO_TEST_CASE( apply_force_test )
{
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    BOOST_CHECK(uut.get_object(0)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(0)->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_torque() == point_t(0.0f, 0.0f, 0.0f));

    uut.apply_force(new mock_force(point_t( 0.65f, -2.25f, 1.05f), point_t(0.0f,  0.0f, 0.0f), 1.0f), 0);
    uut.apply_force(new mock_force(point_t(-3.3f,   4.5f,  0.1f ), point_t(0.0f, -0.2f, 9.0f), 1.0f), 1);

    BOOST_CHECK(uut.get_object(0)->get_force()  == point_t( 0.65f, -2.25f, 1.05f));
    BOOST_CHECK(uut.get_object(1)->get_force()  == point_t(-3.3f,   4.5f,  0.1f ));
    BOOST_CHECK(uut.get_object(0)->get_torque() == point_t( 0.0f,   0.0f,  0.0f ));
    BOOST_CHECK(uut.get_object(1)->get_torque() == point_t( 0.0f,  -0.2f,  9.0f ));
}

BOOST_AUTO_TEST_CASE( apply_force_not_moving_test )
{
    uut.add_object(po0.release());
    uut.add_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    BOOST_CHECK(uut.get_object(0)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(0)->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_torque() == point_t(0.0f, 0.0f, 0.0f));

    uut.apply_force(new mock_force(point_t( 0.65f, -2.25f, 1.05f), point_t(0.0f,  0.0f, 0.0f), 1.0f), 0);
    uut.apply_force(new mock_force(point_t(-3.3f,   4.5f,  0.1f ), point_t(0.0f, -0.2f, 9.0f), 1.0f), 1);

    BOOST_CHECK(uut.get_object(0)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(0)->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_torque() == point_t(0.0f, 0.0f, 0.0f));
}


/* Apply force tests */
BOOST_AUTO_TEST_CASE( apply_acceleration_test )
{
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());
    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    BOOST_CHECK(uut.get_object(0)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(0)->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_object(1)->get_torque() == point_t(0.0f, 0.0f, 0.0f));

    uut.apply_force(new mock_force(point_t( 0.65f, -2.25f, 1.05f), point_t(0.0f,  0.0f, 0.0f), 1.0f), 0);
    uut.apply_force(new mock_force(point_t(-3.3f,   4.5f,  0.1f ), point_t(0.0f, -0.2f, 9.0f), 1.0f), 1);

    uut.advance_time(0.5f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_force()    - point_t( 0.65f,   -2.25f,   1.05f  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_force()    - point_t(-3.3f,     4.5f,    0.1f   ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_torque()   - point_t( 0.0f,     0.0f,    0.0f   ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_torque()   - point_t( 0.0f,    -0.2f,    9.0f   ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t( 0.0325f, -0.1125f, 0.0525f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(-0.165f,   0.225f,  0.005f ))) < result_tolerance);
}


/* Collision detect versus tests */
BOOST_AUTO_TEST_CASE( get_collision_test )
{
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());
    uut.add_moving_object(po2.release());
    uut.add_moving_object(po3.release());
    BOOST_CHECK(uut.number_of_objects() == 4);
    BOOST_CHECK(uut.number_of_moving_objects() == 4);
    BOOST_CHECK(uut.next_object_id() == 4);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == po2_check);
    BOOST_CHECK(uut.get_object(3) == po3_check);
    BOOST_CHECK(uut.get_object(4) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 3);
    BOOST_CHECK(uut.default_collider() == collider);

    const collision_info<> *const c1_2 = uut.get_collision(1, 2);
    BOOST_CHECK(fabs(magnitude(c1_2->get_normal_of_collision() - point_t(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c1_2->get_point_of_collision()  - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c1_2->get_time() == 0.0f);
    BOOST_CHECK(c1_2->get_type() == collision_t::SLIDING_COLLISION);

    const collision_info<> *const c2_1 = uut.get_collision(2, 1);
    BOOST_CHECK(fabs(magnitude(c2_1->get_normal_of_collision() - point_t(0.0f,  1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c2_1->get_point_of_collision()  - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c2_1->get_time() == 0.0f);
    BOOST_CHECK(c2_1->get_type() == collision_t::SLIDING_COLLISION);

    const collision_info<> *const c2_3 = uut.get_collision(2, 3);
    BOOST_CHECK(fabs(magnitude(c2_3->get_normal_of_collision() - point_t(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c2_3->get_point_of_collision()  - point_t(0.0f, -8.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c2_3->get_time() == 0.0f);
    BOOST_CHECK(c2_3->get_type() == collision_t::SLIDING_COLLISION);

    const collision_info<> *const c3_2 = uut.get_collision(3, 2);
    BOOST_CHECK(fabs(magnitude(c3_2->get_normal_of_collision() - point_t(0.0f,  1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c3_2->get_point_of_collision()  - point_t(0.0f, -8.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c3_2->get_time() == 0.0f);
    BOOST_CHECK(c3_2->get_type() == collision_t::SLIDING_COLLISION);

    BOOST_CHECK(uut.get_collision(0, 1) == nullptr);
    BOOST_CHECK(uut.get_collision(0, 2) == nullptr);
    BOOST_CHECK(uut.get_collision(0, 3) == nullptr);
    BOOST_CHECK(uut.get_collision(0, 4) == nullptr);
    BOOST_CHECK(uut.get_collision(1, 0) == nullptr);
    BOOST_CHECK(uut.get_collision(2, 0) == nullptr);
    BOOST_CHECK(uut.get_collision(3, 0) == nullptr);
    BOOST_CHECK(uut.get_collision(4, 0) == nullptr);
}


/* Collision detect versus tests */
BOOST_AUTO_TEST_CASE( void_all_collisions_with_in_steps_test )
{
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());
    uut.add_moving_object(po2.release());
    uut.add_moving_object(po3.release());
    BOOST_CHECK(uut.number_of_objects() == 4);
    BOOST_CHECK(uut.number_of_moving_objects() == 4);
    BOOST_CHECK(uut.next_object_id() == 4);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == po2_check);
    BOOST_CHECK(uut.get_object(3) == po3_check);
    BOOST_CHECK(uut.get_object(4) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 3);
    BOOST_CHECK(uut.default_collider() == collider);

    /* Check voiding an unknown object has no effect */
    uut.void_all_collisions_with(4);
    BOOST_CHECK(uut.number_of_collisions() == 3);
    BOOST_CHECK(uut.get_collision(1, 2) != nullptr);
    BOOST_CHECK(uut.get_collision(2, 1) != nullptr);
    BOOST_CHECK(uut.get_collision(2, 3) != nullptr);
    BOOST_CHECK(uut.get_collision(3, 2) != nullptr);

    /* Check voiding an object that isnt colliding has no effect */
    uut.void_all_collisions_with(0);
    BOOST_CHECK(uut.number_of_collisions() == 3);
    BOOST_CHECK(uut.get_collision(1, 2) != nullptr);
    BOOST_CHECK(uut.get_collision(2, 1) != nullptr);
    BOOST_CHECK(uut.get_collision(2, 3) != nullptr);
    BOOST_CHECK(uut.get_collision(3, 2) != nullptr);

    /* Void and check */
    uut.void_all_collisions_with(1);
    BOOST_CHECK(uut.number_of_collisions() == 3);

    const collision_info<> *const c1_2_0 = uut.get_collision(1, 2);
    BOOST_CHECK(fabs(magnitude(c1_2_0->get_normal_of_collision() - point_t(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c1_2_0->get_point_of_collision()  - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c1_2_0->get_time() == 0.0f);
    BOOST_CHECK(c1_2_0->get_type() == collision_t::SLIDING_COLLISION);

    const collision_info<> *const c2_1_0 = uut.get_collision(2, 1);
    BOOST_CHECK(c2_1_0->get_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(c2_1_0->get_type() == collision_t::NO_COLLISION);
    
    const collision_info<> *const c2_3_0 = uut.get_collision(2, 3);
    BOOST_CHECK(fabs(magnitude(c2_3_0->get_normal_of_collision() - point_t(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c2_3_0->get_point_of_collision()  - point_t(0.0f, -8.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c2_3_0->get_time() == 0.0f);
    BOOST_CHECK(c2_3_0->get_type() == collision_t::SLIDING_COLLISION);

    const collision_info<> *const c3_2_0 = uut.get_collision(3, 2);
    BOOST_CHECK(fabs(magnitude(c3_2_0->get_normal_of_collision() - point_t(0.0f,  1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c3_2_0->get_point_of_collision()  - point_t(0.0f, -8.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c3_2_0->get_time() == 0.0f);
    BOOST_CHECK(c3_2_0->get_type() == collision_t::SLIDING_COLLISION);

    /* Void and check */
    uut.void_all_collisions_with(3);
    BOOST_CHECK(uut.number_of_collisions() == 3);

    const collision_info<> *const c1_2_1 = uut.get_collision(1, 2);
    BOOST_CHECK(fabs(magnitude(c1_2_1->get_normal_of_collision() - point_t(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c1_2_1->get_point_of_collision()  - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c1_2_1->get_time() == 0.0f);
    BOOST_CHECK(c1_2_1->get_type() == collision_t::SLIDING_COLLISION);

    const collision_info<> *const c2_1_1 = uut.get_collision(2, 1);
    BOOST_CHECK(c2_1_1->get_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(c2_1_1->get_type() == collision_t::NO_COLLISION);
    
    const collision_info<> *const c2_3_1 = uut.get_collision(2, 3);
    BOOST_CHECK(c2_3_1->get_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(c2_3_1->get_type() == collision_t::NO_COLLISION);

    const collision_info<> *const c3_2_1 = uut.get_collision(3, 2);
    BOOST_CHECK(fabs(magnitude(c3_2_1->get_normal_of_collision() - point_t(0.0f,  1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c3_2_1->get_point_of_collision()  - point_t(0.0f, -8.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c3_2_1->get_time() == 0.0f);
    BOOST_CHECK(c3_2_1->get_type() == collision_t::SLIDING_COLLISION);

    /* Void and check */
    uut.void_all_collisions_with(2);
    BOOST_CHECK(uut.number_of_collisions() == 3);

    const collision_info<> *const c1_2_2 = uut.get_collision(1, 2);
    BOOST_CHECK(c1_2_2->get_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(c1_2_2->get_type() == collision_t::NO_COLLISION);

    const collision_info<> *const c2_1_2 = uut.get_collision(2, 1);
    BOOST_CHECK(c2_1_2->get_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(c2_1_2->get_type() == collision_t::NO_COLLISION);
    
    const collision_info<> *const c2_3_2 = uut.get_collision(2, 3);
    BOOST_CHECK(c2_3_2->get_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(c2_3_2->get_type() == collision_t::NO_COLLISION);

    const collision_info<> *const c3_2_2 = uut.get_collision(3, 2);
    BOOST_CHECK(c3_2_2->get_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(c3_2_2->get_type() == collision_t::NO_COLLISION);
}


/* Default Collider tests */
BOOST_AUTO_TEST_CASE( default_collider_test )
{
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    auto collider1 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.default_collider(collider1);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider1);
}


BOOST_AUTO_TEST_CASE( add_pairwise_collider_test )
{
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    auto collider00 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.pair_collider(collider00, 0, 0);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
    BOOST_CHECK(uut.pair_collider(0, 0) == collider00);

    auto collider01 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.pair_collider(collider01, 0, 1);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
    BOOST_CHECK(uut.pair_collider(0, 1) == collider01);
    BOOST_CHECK(uut.pair_collider(1, 0) == collider01);

    auto collider10 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.pair_collider(collider10, 1, 0);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
    BOOST_CHECK(uut.pair_collider(0, 1) == collider10);
    BOOST_CHECK(uut.pair_collider(1, 0) == collider10);

    auto collider11 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.pair_collider(collider11, 1, 1);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
    BOOST_CHECK(uut.pair_collider(1, 1) == collider11);
}


BOOST_AUTO_TEST_CASE( update_pairwise_collider_test )
{
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    auto collider00 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.pair_collider(collider00, 0, 0);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
    BOOST_CHECK(uut.pair_collider(0, 0) == collider00);

    auto collider01 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.pair_collider(collider01, 0, 0);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
    BOOST_CHECK(uut.pair_collider(0, 0) == collider01);
}


BOOST_AUTO_TEST_CASE( get_pairwise_collider_test )
{
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    BOOST_CHECK(uut.pair_collider( 0,  0) == collider);
    BOOST_CHECK(uut.pair_collider( 1,  0) == collider);
    BOOST_CHECK(uut.pair_collider( 0,  1) == collider);
    BOOST_CHECK(uut.pair_collider(-7, 18) == collider);

    auto collider00 = new rigid_body_collider(0.5f, 0.0f, 0.5f);
    uut.pair_collider(collider00, 0, 0);
    BOOST_CHECK(uut.number_of_objects() == 0);
    BOOST_CHECK(uut.number_of_moving_objects() == 0);
    BOOST_CHECK(uut.next_object_id() == 0);
    BOOST_CHECK(uut.get_object(0) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);
    
    BOOST_CHECK(uut.pair_collider( 0,  0) == collider00);
    BOOST_CHECK(uut.pair_collider( 0,  1) == collider);
    BOOST_CHECK(uut.pair_collider( 1,  0) == collider);
    BOOST_CHECK(uut.pair_collider(-7, 18) == collider);
}


BOOST_AUTO_TEST_CASE( advance_time_collide_test )
{
    po0->set_velocity(point_t(0.0f, -9.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  9.0f, 0.0f));
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());

    rigid_body_collider *const c = new rigid_body_collider(0.5f, 0.0f, 0.0f);
    uut.default_collider(c);

    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == c);

    uut.advance_time(0.5f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f,  9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  5.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -5.0f, 0.0f))) < result_tolerance);

    uut.advance_time(0.6f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f,  4.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f, -4.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  0.95f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -0.95f, 0.0f))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( advance_time_three_way_collide_test )
{
    po0->set_velocity(point_t(0.0f, -9.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  9.0f, 0.0f));
    po4->set_velocity(point_t(0.0f, -9.0f, 0.0f));
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());
    uut.add_moving_object(po4.release());

    rigid_body_collider *const dc = new rigid_body_collider(0.5f, 0.0f, 0.0f);
    uut.default_collider(dc);

    rigid_body_collider *const pc = new rigid_body_collider(0.0f, 0.0f, 0.0f);
    uut.pair_collider(pc, 0, 1);

    BOOST_CHECK(uut.number_of_objects() == 3);
    BOOST_CHECK(uut.number_of_moving_objects() == 3);
    BOOST_CHECK(uut.next_object_id() == 3);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == po4_check);
    BOOST_CHECK(uut.get_object(3) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == dc);
    BOOST_CHECK(uut.pair_collider(0, 1) == pc);

    uut.advance_time(0.5f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f,  9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(2)->get_velocity() - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  5.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -5.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(2)->get_center_of_mass() - point_t(0.0f,  6.5f, 0.0f))) < result_tolerance);

    uut.advance_time(0.6f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f, -2.25f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f, -4.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(2)->get_velocity() - point_t(0.0f, -2.25f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  0.524812f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -0.95f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(2)->get_center_of_mass() - point_t(0.0f,  1.52556f, 0.0f))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( advance_time_rotating_past_test )
{
    po0->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
    po4->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
    po0->set_angular_velocity(point_t(1.0f, 0.0f, 10.0f));
    po4->set_angular_velocity(point_t(1.0f, 0.0f, 10.0f));
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po4.release());

    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po4_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == collider);

    uut.advance_time(0.5f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t( 0.5f,  9.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(-0.5f, 11.0f, 0.0f))) < result_tolerance);

    uut.advance_time(0.5f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t( 1.0f,  9.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(-1.0f, 11.0f, 0.0f))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( advance_time_rotating_in_plane_collide_test )
{
    po0->set_velocity(point_t(0.0f, -9.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  9.0f, 0.0f));
    po0->set_angular_velocity(point_t(0.0f, -9.0f, 0.0f));
    po1->set_angular_velocity(point_t(0.0f,  9.0f, 0.0f));
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());

    rigid_body_collider *const c = new rigid_body_collider(0.5f, 0.0f, 0.0f);
    uut.default_collider(c);

    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == c);

    uut.advance_time(0.5f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f,  9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  5.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -5.0f, 0.0f))) < result_tolerance);

    uut.advance_time(0.6f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f,  4.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f, -4.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  0.95f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -0.95f, 0.0f))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( advance_time_rotating_in_plane_collide_and_slide_test )
{
    po0->set_velocity(point_t(0.0f, -9.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  9.0f, 0.0f));
    po0->set_angular_velocity(point_t(0.0f, -9.0f, 0.0f));
    po1->set_angular_velocity(point_t(0.0f,  9.0f, 0.0f));
    uut.add_moving_object(po0.release());
    uut.add_moving_object(po1.release());

    rigid_body_collider *const c = new rigid_body_collider(0.0f, 0.0f, 0.0f);
    uut.default_collider(c);

    BOOST_CHECK(uut.number_of_objects() == 2);
    BOOST_CHECK(uut.number_of_moving_objects() == 2);
    BOOST_CHECK(uut.next_object_id() == 2);
    BOOST_CHECK(uut.get_object(0) == po0_check);
    BOOST_CHECK(uut.get_object(1) == po1_check);
    BOOST_CHECK(uut.get_object(2) == nullptr);
    BOOST_CHECK(uut.number_of_collisions() == 0);
    BOOST_CHECK(uut.default_collider() == c);

    uut.advance_time(0.5f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f, -9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f,  9.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  5.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -5.0f, 0.0f))) < result_tolerance);

    uut.advance_time(0.6f);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(0)->get_center_of_mass() - point_t(0.0f,  0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_object(1)->get_center_of_mass() - point_t(0.0f, -0.5f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
