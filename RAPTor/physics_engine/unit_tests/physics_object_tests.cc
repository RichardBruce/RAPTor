#ifdef STAND_ALONE
#define BOOST_TEST_MODULE physics_object test

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
#include "point_t.h"
#include "matrix_3d.h"
#include "simplex.h"
#include "physics_object.h"

/* Test headers */
#include "mock_force.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct physics_object_fixture : private boost::noncopyable
{
    physics_object_fixture()
    : point_outside(100.0, 110.0, 90.0),
      mat(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0)),
      vg0(make_plane(mat, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0))),
      vg1(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5,  0.5,  0.5))),
      vg2(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5,  0.5,  0.5))),
      vg3(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5,  0.5,  0.5))),
      vg4(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5,  0.5,  0.5))),
      vg5(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5,  0.5,  0.5))),
      vg6(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5,  0.5,  0.5))),
      vg7(make_cube(mat, point_t( 5.5, -1.5, -3.5), point_t(6.5, -0.5, -2.5))),
      plane_po(new physics_object(vg0, point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity(), 7)),
      cube_po0(new physics_object(vg1, point_t(0.5,  -9.5, 0.0), 1.0)),
      cube_po1(new physics_object(vg2, point_t(0.0,   9.5, 0.0), 1.0)),
      centered_cube(new physics_object(vg7, point_t(-5.0, 10.5, 3.0), 1.0)),
      far_po(new physics_object(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, 9.5, 0.0), 1.0)),
      near_po(new physics_object(make_cube(mat, point_t(-0.25, -0.5, -0.5), point_t(0.25, 0.5, 0.5)), point_t(1.25, -8.5, 1.0) + (0.5 * raptor_physics::WELD_DISTANCE), 1.0)),
      nearer_po(new physics_object(make_cube(mat, point_t(-0.25, -0.5, -0.5), point_t(0.25, 0.5, 0.5)), point_t(1.25, -8.5, 1.0) + (0.25 * raptor_physics::WELD_DISTANCE), 1.0)),
      touching_po(new physics_object(make_cube(mat, point_t(-0.25, -0.5, -0.5), point_t(0.25, 0.5, 0.5)), point_t(1.25, -8.5, 1.0) + (0.25 * raptor_physics::EPSILON), 1.0)),
      hit_except_x_po(new physics_object(make_cube(mat, point_t(-0.25, -0.5, -0.5), point_t(0.25, 0.5, 0.5)), point_t(1.25 + (0.5 * raptor_physics::WELD_DISTANCE), -8.5, 1.0), 1.0)),
      orientated_po(new physics_object(vg3, quaternion_t(0.924, 0.383, 0.0, 0.0), point_t(1.0, -8.5, 1.0), 1.0)),
      orientated_po_type(new physics_object(vg4, quaternion_t(0.924, 0.383, 0.0, 0.0), point_t(1.0, -8.5, 1.0), 1.0, 4)),
      moving_po(new physics_object(vg5, quaternion_t(0.924, 0.383, 0.0, 0.0), point_t(1.0, -8.5, 1.0), point_t(1.3, 5.4, 6.5), point_t(7.5, 3.7, 3.1), 1.0)),
      moving_po_type(new physics_object(vg6, quaternion_t(0.924, 0.383, 0.0, 0.0), point_t(1.0, -8.5, 1.0), point_t(1.3, 5.4, 6.5), point_t(7.5, 3.7, 3.1), 1.0, 17))
      {  };

    ~physics_object_fixture()
    {
        delete mat;
        delete plane_po;
        delete cube_po0;
        delete cube_po1;
        delete centered_cube;
        delete far_po;
        delete near_po;
        delete nearer_po;
        delete touching_po;
        delete hit_except_x_po;
        delete orientated_po;
        delete orientated_po_type;
        delete moving_po;
        delete moving_po_type;
    }

    const point_t                   point_outside;
    raptor_raytracer::material  *   mat;
    vertex_group                *   vg0;
    vertex_group                *   vg1;
    vertex_group                *   vg2;
    vertex_group                *   vg3;
    vertex_group                *   vg4;
    vertex_group                *   vg5;
    vertex_group                *   vg6;
    vertex_group                *   vg7;
    physics_object              *   plane_po;
    physics_object              *   cube_po0;
    physics_object              *   cube_po1;
    physics_object              *   centered_cube;
    physics_object              *   far_po;
    physics_object              *   near_po;
    physics_object              *   nearer_po;
    physics_object              *   touching_po;
    physics_object              *   hit_except_x_po;
    physics_object              *   orientated_po;
    physics_object              *   orientated_po_type;
    physics_object              *   moving_po;
    physics_object              *   moving_po_type;
};


BOOST_FIXTURE_TEST_SUITE( physics_object_tests, physics_object_fixture )

const float result_tolerance = 0.0005;

/* Standalone function tests */
BOOST_AUTO_TEST_CASE( is_uncertain_test )
{
    BOOST_CHECK(is_uncertain(NO_COLLISION) == false);
    BOOST_CHECK(is_uncertain(SLIDING_COLLISION) == false);
    BOOST_CHECK(is_uncertain(COLLISION) == false);
    BOOST_CHECK(is_uncertain(POSSIBLE_SLIDING_COLLISION) == true);
    BOOST_CHECK(is_uncertain(POSSIBLE_COLLISION) == true);
}


BOOST_AUTO_TEST_CASE( to_uncertain_test )
{
    BOOST_CHECK(to_certain(NO_COLLISION) == NO_COLLISION);
    BOOST_CHECK(to_certain(SLIDING_COLLISION) == SLIDING_COLLISION);
    BOOST_CHECK(to_certain(COLLISION) == COLLISION);
    BOOST_CHECK(to_certain(POSSIBLE_SLIDING_COLLISION) == SLIDING_COLLISION);
    BOOST_CHECK(to_certain(POSSIBLE_COLLISION) == COLLISION);
}


/* ctor tests */
BOOST_AUTO_TEST_CASE( basic_ctor_test )
{
    /* Check the plane */
    BOOST_CHECK(plane_po->get_physical_type() == 7);
    BOOST_CHECK(plane_po->number_of_registered_forces() == 0);
    BOOST_CHECK(plane_po->get_vertex_group().get() == vg0);
    
    BOOST_CHECK(plane_po->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_angular_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_velocity(point_outside) == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(plane_po->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(plane_po->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_speed() == 0.0);

    const inertia_tensor &it0 = plane_po->get_inertia_tenor();
    BOOST_CHECK(plane_po->get_mass() == it0.mass());
    BOOST_CHECK(plane_po->get_center_of_mass() == it0.center_of_mass());
    BOOST_CHECK(plane_po->get_orientation() == quaternion_t(1.0, 0.0, 0.0, 0.0));


    /* Check the cube 0 */
    BOOST_CHECK(cube_po0->get_physical_type() == 0);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
    BOOST_CHECK(cube_po0->get_vertex_group().get() == vg1);

    BOOST_CHECK(cube_po0->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->get_angular_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->get_velocity(point_outside) == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(cube_po0->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(cube_po0->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->get_speed() == 0.0);

    const inertia_tensor &it1 = cube_po0->get_inertia_tenor();
    BOOST_CHECK(cube_po0->get_mass() == it1.mass());
    BOOST_CHECK(cube_po0->get_center_of_mass() == it1.center_of_mass());
    BOOST_CHECK(cube_po0->get_orientation() == quaternion_t(1.0, 0.0, 0.0, 0.0));


    /* Check the cube 1 */
    BOOST_CHECK(cube_po1->get_physical_type() == 0);
    BOOST_CHECK(cube_po1->number_of_registered_forces() == 0);
    BOOST_CHECK(cube_po1->get_vertex_group().get() == vg2);

    BOOST_CHECK(cube_po1->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po1->get_angular_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po1->get_velocity(point_outside) == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(cube_po1->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po1->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(cube_po1->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po1->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po1->get_speed() == 0.0);

    const inertia_tensor &it2 = cube_po1->get_inertia_tenor();
    BOOST_CHECK(cube_po1->get_mass() == it2.mass());
    BOOST_CHECK(cube_po1->get_center_of_mass() == it2.center_of_mass());
    BOOST_CHECK(cube_po1->get_orientation() == quaternion_t(1.0, 0.0, 0.0, 0.0));


    /* Check for cube with non-center com */
    BOOST_CHECK(centered_cube->get_physical_type() == 0);
    BOOST_CHECK(centered_cube->number_of_registered_forces() == 0);
    BOOST_CHECK(centered_cube->get_vertex_group().get() == vg7);

    BOOST_CHECK(centered_cube->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(centered_cube->get_angular_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(centered_cube->get_velocity(point_outside) == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(centered_cube->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(centered_cube->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(centered_cube->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(centered_cube->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(centered_cube->get_speed() == 0.0);

    const inertia_tensor &it3 = centered_cube->get_inertia_tenor();
    BOOST_CHECK(centered_cube->get_mass() == it3.mass());
    BOOST_CHECK(centered_cube->get_center_of_mass() == it3.center_of_mass());
    BOOST_CHECK(centered_cube->get_orientation() == quaternion_t(1.0, 0.0, 0.0, 0.0));
}


BOOST_AUTO_TEST_CASE( orientated_ctor_test )
{
    /* Check without type */
    BOOST_CHECK(orientated_po->get_physical_type() == 0);
    BOOST_CHECK(orientated_po->number_of_registered_forces() == 0);
    BOOST_CHECK(orientated_po->get_vertex_group().get() == vg3);
    
    BOOST_CHECK(orientated_po->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po->get_angular_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po->get_velocity(point_outside) == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(orientated_po->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(orientated_po->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po->get_speed() == 0.0);

    const inertia_tensor &it0 = orientated_po->get_inertia_tenor();
    BOOST_CHECK(orientated_po->get_mass() == it0.mass());
    BOOST_CHECK(orientated_po->get_center_of_mass() == it0.center_of_mass());
    BOOST_CHECK(orientated_po->get_orientation() == quaternion_t(0.924, 0.383, 0.0, 0.0));

    /* Check with type */
    BOOST_CHECK(orientated_po_type->get_physical_type() == 4);
    BOOST_CHECK(orientated_po_type->number_of_registered_forces() == 0);
    BOOST_CHECK(orientated_po_type->get_vertex_group().get() == vg4);
    
    BOOST_CHECK(orientated_po_type->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po_type->get_angular_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po_type->get_velocity(point_outside) == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(orientated_po_type->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po_type->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    BOOST_CHECK(orientated_po_type->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po_type->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(orientated_po_type->get_speed() == 0.0);

    const inertia_tensor &it1 = orientated_po_type->get_inertia_tenor();
    BOOST_CHECK(orientated_po_type->get_mass() == it1.mass());
    BOOST_CHECK(orientated_po_type->get_center_of_mass() == it1.center_of_mass());
    BOOST_CHECK(orientated_po_type->get_orientation() == quaternion_t(0.924, 0.383, 0.0, 0.0));
}


BOOST_AUTO_TEST_CASE( moving_ctor_test )
{
    /* Check without type */
    BOOST_CHECK(moving_po->get_physical_type() == 0);
    BOOST_CHECK(moving_po->number_of_registered_forces() == 0);
    BOOST_CHECK(moving_po->get_vertex_group().get() == vg5);
    
    BOOST_CHECK(moving_po->get_velocity() == point_t(1.3, 5.4, 6.5));
    BOOST_CHECK(moving_po->get_angular_velocity() == point_t(7.5, 3.7, 3.1));
    BOOST_CHECK(fabs(magnitude(moving_po->get_velocity(point_outside) - point_t(-36.75, -355.2, 528.95))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(moving_po->get_momentum() - point_t(1.3, 5.4, 6.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(moving_po->get_angular_momentum() - point_t(1.25, 0.616835, 0.516808))) < result_tolerance);

    BOOST_CHECK(moving_po->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(moving_po->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(moving_po->get_speed() - 8.54985) < result_tolerance);

    const inertia_tensor &it0 = moving_po->get_inertia_tenor();
    BOOST_CHECK(moving_po->get_mass() == it0.mass());
    BOOST_CHECK(moving_po->get_center_of_mass() == it0.center_of_mass());
    BOOST_CHECK(moving_po->get_orientation() == quaternion_t(0.924, 0.383, 0.0, 0.0));

    /* Check with type */
    BOOST_CHECK(moving_po_type->get_physical_type() == 17);
    BOOST_CHECK(moving_po_type->number_of_registered_forces() == 0);
    BOOST_CHECK(moving_po_type->get_vertex_group().get() == vg6);
    
    BOOST_CHECK(moving_po_type->get_velocity() == point_t(1.3, 5.4, 6.5));
    BOOST_CHECK(moving_po_type->get_angular_velocity() == point_t(7.5, 3.7, 3.1));
    BOOST_CHECK(fabs(magnitude(moving_po_type->get_velocity(point_outside) - point_t(-36.75, -355.2, 528.95))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(moving_po_type->get_momentum() - point_t(1.3, 5.4, 6.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(moving_po_type->get_angular_momentum() - point_t(1.25, 0.616835, 0.516808))) < result_tolerance);

    BOOST_CHECK(moving_po_type->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(moving_po_type->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(moving_po_type->get_speed() - 8.54985) < result_tolerance);

    const inertia_tensor &it1 = moving_po_type->get_inertia_tenor();
    BOOST_CHECK(moving_po_type->get_mass() == it1.mass());
    BOOST_CHECK(moving_po_type->get_center_of_mass() == it1.center_of_mass());
    BOOST_CHECK(moving_po_type->get_orientation() == quaternion_t(0.924, 0.383, 0.0, 0.0));
}


/* Test get vertex */
BOOST_AUTO_TEST_CASE( get_vertex_test )
{
    BOOST_CHECK(orientated_po->get_vertex_group()->get_number_of_vertices() == 8);
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(0) == point_t(-0.5, -0.5, -0.5));
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(1) == point_t( 0.5, -0.5, -0.5));
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(2) == point_t( 0.5,  0.5, -0.5));
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(3) == point_t(-0.5,  0.5, -0.5));
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(4) == point_t(-0.5, -0.5,  0.5));
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(5) == point_t( 0.5, -0.5,  0.5));
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(6) == point_t( 0.5,  0.5,  0.5));
    BOOST_CHECK(orientated_po->get_vertex_group()->get_vertex(7) == point_t(-0.5,  0.5,  0.5));
    
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(0) - point_t(-0.5,  0.000581, -0.707203))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(1) - point_t( 0.5,  0.000581, -0.707203))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(2) - point_t( 0.5,  0.707203,  0.000581))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(3) - point_t(-0.5,  0.707203,  0.000581))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(4) - point_t(-0.5, -0.707203, -0.000581))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(5) - point_t( 0.5, -0.707203, -0.000581))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(6) - point_t( 0.5, -0.000581,  0.707203))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_orientated_vertex(7) - point_t(-0.5, -0.000581,  0.707203))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(0) - point_t(0.5, -8.499419, 0.292797))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(1) - point_t(1.5, -8.499419, 0.292797))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(2) - point_t(1.5, -7.792797, 1.000581))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(3) - point_t(0.5, -7.792797, 1.000581))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(4) - point_t(0.5, -9.2072,   0.999419))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(5) - point_t(1.5, -9.2072,   0.999419))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(6) - point_t(1.5, -8.50058,  1.707203))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(orientated_po->get_global_vertex(7) - point_t(0.5, -8.50058,  1.707203))) < result_tolerance);
}


/* Test register force */
BOOST_AUTO_TEST_CASE( register_force_test )
{
    /* Check the plane */
    BOOST_CHECK(plane_po->number_of_registered_forces() == 0);

    /* Register force and check */
    const_force *const cf0 = new const_force(point_t(0.0, 1.0, 0.0), point_t(0.5, -7.2, 5.5), 0.5);
    plane_po->register_force(cf0);
    BOOST_CHECK(plane_po->number_of_registered_forces() == 1);
    BOOST_CHECK(plane_po->get_registered_force(0) == cf0);

    /* Check the cube 0 */
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
    
    /* Register force and check */
    const_force *const cf1 = new const_force(point_t(1.0,  1.0, -0.5), point_t(-0.5,  7.2, 3.5), 0.25);
    const_force *const cf2 = new const_force(point_t(0.0, -1.0,  2.0), point_t( 1.5, -7.2, 4.5), 2.0);
    cube_po0->register_force(cf1);
    cube_po0->register_force(cf2);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 2);
    BOOST_CHECK(cube_po0->get_registered_force(0) == cf1);
    BOOST_CHECK(cube_po0->get_registered_force(1) == cf2);
}


/* Test get registered force */
BOOST_AUTO_TEST_CASE( get_registered_force_test )
{
    /* Check the cube 0 */
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
    BOOST_CHECK(cube_po0->get_registered_force(0)       == nullptr);   

    /* Register force and check */
    const_force *const cf = new const_force(point_t(0.0, 1.0, 0.0), point_t(0.5, -7.2, 5.5), 0.5);
    cube_po0->register_force(cf);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 1);
    BOOST_CHECK(cube_po0->get_registered_force(0)       == cf);
    BOOST_CHECK(cube_po0->get_registered_force(1)       == nullptr);
}


/* Test begin time step */
BOOST_AUTO_TEST_CASE( infinite_mass_begin_time_step_test )
{
    /* Check the plane */
    BOOST_CHECK(plane_po->number_of_registered_forces() == 0);

    /* Register force and check */
    const_force *const cf = new const_force(point_t(0.0, 1.0, 0.0), point_t(0.5, -7.2, 5.5), 0.5);
    plane_po->register_force(cf);
    BOOST_CHECK(plane_po->number_of_registered_forces() == 1);
    BOOST_CHECK(plane_po->get_registered_force(0) == cf);

    /* Time step and check */
    plane_po->begin_time_step(1.0);
    BOOST_CHECK(plane_po->get_force() == point_t(0.5, -7.2, 5.5));
    BOOST_CHECK(plane_po->get_torque() == point_t(5.5, 0.0, -0.5));

    /* Commit and check */
    plane_po->commit_movement(1.0);
    BOOST_CHECK(plane_po->number_of_registered_forces() == 0);
}


BOOST_AUTO_TEST_CASE( half_step_begin_time_step_test )
{
    /* Check the cube 0 */
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
    
    /* Register force and check */
    linear_force *const lf = new linear_force(point_t(0.0, 1.0, 0.0), point_t(0.5, -7.2, 5.5), point_t(5.5, 0.0, -0.5), 1.0);
    cube_po0->register_force(lf);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 1);
    BOOST_CHECK(cube_po0->get_registered_force(0) == lf);

    /* Time step and check */
    cube_po0->begin_time_step(0.5);
    BOOST_CHECK(cube_po0->get_force() == point_t(5.5, 0.0, -0.5));
    BOOST_CHECK(cube_po0->get_torque() == point_t(-0.5, 0.0, -5.5));
    
    /* Commit and check */
    cube_po0->commit_movement(0.5);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 1);
    BOOST_CHECK(cube_po0->get_registered_force(0) == lf);

    BOOST_CHECK(cube_po0->get_force() == point_t(5.75, -3.6, 2.25));
    BOOST_CHECK(cube_po0->get_torque() == point_t(2.25, 0.0, -5.75));
}


BOOST_AUTO_TEST_CASE( expended_forces_begin_time_step_test )
{
    /* Check the cube 0 */
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
    
    /* Register force and check */
    linear_force *const lf = new linear_force(point_t(0.0, 1.0, 0.0), point_t(0.5, -7.2, 5.5), point_t(5.5, 0.0, -0.5), 1.0);
    cube_po0->register_force(lf);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 1);
    BOOST_CHECK(cube_po0->get_registered_force(0) == lf);

    /* Time step and check */
    cube_po0->begin_time_step(0.5);
    BOOST_CHECK(cube_po0->get_force() == point_t(5.5, 0.0, -0.5));
    BOOST_CHECK(cube_po0->get_torque() == point_t(-0.5, 0.0, -5.5));
    
    /* Commit and check */
    cube_po0->commit_movement(0.5);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 1);
    BOOST_CHECK(cube_po0->get_registered_force(0) == lf);

    /* Time step and check */
    cube_po0->begin_time_step(0.5);
    BOOST_CHECK(cube_po0->get_force() == point_t(5.75, -3.6, 2.25));
    BOOST_CHECK(cube_po0->get_torque() == point_t(2.25, 0.0, -5.75));
    
    /* Commit and check */
    cube_po0->commit_movement(0.5);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);

    /* Time step and check */
    cube_po0->begin_time_step(1.0);
    BOOST_CHECK(cube_po0->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->get_torque() == point_t(0.0, 0.0, 0.0));
    
    /* Commit and check */
    cube_po0->commit_movement(1.0);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
}


BOOST_AUTO_TEST_CASE( staggered_forces_begin_time_step_test )
{
    /* Check the cube 0 */
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
    
    /* Register force and check */
    const_force *const cf0 = new const_force(point_t(0.0, 0.0, 0.0), point_t(0.5, -7.2, 5.5), 0.5);
    const_force *const cf1 = new const_force(point_t(0.0, 0.0, 0.0), point_t(1.5, -8.2, 6.5), 1.5);
    const_force *const cf2 = new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -8.2, 6.5), 1.5);
    const_force *const cf3 = new const_force(point_t(0.0, 0.0, 0.0), point_t(2.5, -9.2, 7.5), 2.5);
    cube_po0->register_force(cf0);
    cube_po0->register_force(cf1);
    cube_po0->register_force(cf2);
    cube_po0->register_force(cf3);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 4);
    BOOST_CHECK(cube_po0->get_registered_force(0) == cf0);
    BOOST_CHECK(cube_po0->get_registered_force(1) == cf1);
    BOOST_CHECK(cube_po0->get_registered_force(2) == cf2);
    BOOST_CHECK(cube_po0->get_registered_force(3) == cf3);

    /* Time step and check */
    cube_po0->begin_time_step(1.0);
    BOOST_CHECK(cube_po0->get_force() == point_t(4.5, -32.8, 26.0));
    BOOST_CHECK(cube_po0->get_torque() == point_t(0.0, 0.0, 0.0));
    
    /* Commit and check */
    cube_po0->commit_movement(1.0);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 3);

    /* Time step and check */
    cube_po0->begin_time_step(1.0);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_force() - point_t(4.0, -25.6, 20.5))) < result_tolerance);
    BOOST_CHECK(cube_po0->get_torque() == point_t(0.0, 0.0, 0.0));
    
    /* Commit and check */
    cube_po0->commit_movement(1.0);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 1);

    /* Time step and check */
    cube_po0->begin_time_step(1.0);
    BOOST_CHECK(cube_po0->get_force() == point_t(2.5, -9.2, 7.5));
    BOOST_CHECK(cube_po0->get_torque() == point_t(0.0, 0.0, 0.0));
    
    /* Commit and check */
    cube_po0->commit_movement(1.0);
    BOOST_CHECK(cube_po0->number_of_registered_forces() == 0);
}


/* Test setters */
BOOST_FIXTURE_TEST_CASE( setter_test, physics_object_fixture )
{
    const point_t v(9.4, 8.3, 7.9);
    cube_po0->set_velocity(v);
    BOOST_CHECK(cube_po0->get_velocity() == v);
    BOOST_CHECK(cube_po0->get_velocity(point_outside) == v);
    BOOST_CHECK(cube_po0->get_momentum() == v);
    
    const point_t w(1.5, 2.1, 5.4);
    cube_po0->set_angular_velocity(w);
    BOOST_CHECK(cube_po0->get_angular_velocity() == w);
    BOOST_CHECK(cube_po0->get_velocity(point_outside) == (v + cross_product(w, point_outside - cube_po0->get_inertia_tenor().center_of_mass())));
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (w * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
}


/* Test apply force */
BOOST_FIXTURE_TEST_CASE( apply_force_whole_frame_at_com_test, physics_object_fixture )
{
    const float t0 = 1.2;
    const point_t f0(1.0, 2.5, 3.9);
    const point_t at0(0.0, 0.0, 0.0);
    plane_po->apply_force(at0, f0, t0);

    BOOST_CHECK(plane_po->get_force() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_torque() == point_t(0.0, 0.0, 0.0));

    const float t1 = 1.2;
    const point_t f1(0.0, 1.5, 2.9);
    const point_t at1(0.0, 0.0, 0.0);
    cube_po0->apply_force(at1, f1, t1);

    BOOST_CHECK(cube_po0->get_force() == f1);
    BOOST_CHECK(cube_po0->get_torque() == point_t(0.0, 0.0, 0.0));
}


BOOST_FIXTURE_TEST_CASE( apply_force_whole_frame_off_com_test, physics_object_fixture )
{
    const float t0 = 1.2;
    const point_t f0(1.0, 12.5, -3.9);
    const point_t at0(0.0, 0.1, 10.0);
    plane_po->apply_force(at0, f0, t0);

    BOOST_CHECK(fabs(magnitude(plane_po->get_force() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    const float t1 = 1.2;
    const point_t f1(0.1, 0.0, -0.1);
    const point_t at1(0.5, -1.3, 0.9);
    cube_po0->apply_force(at1, f1, t1);

    BOOST_CHECK(fabs(magnitude(cube_po0->get_force() - point_t(0.1, 0.0, -0.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_torque() - point_t(0.13, 0.14, 0.13))) < result_tolerance);
}


/* Test apply impluse */
BOOST_FIXTURE_TEST_CASE( apply_impulse_test, physics_object_fixture )
{
    const point_t p0(10.0, 10.0, 10.0);
    const point_t im0(45.0, 100.0, -70.0);
    plane_po->apply_impulse(im0, p0);
    BOOST_CHECK(plane_po->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_angular_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    const point_t p1(0.5, -9.5, 0.0);
    const point_t im1(-5.0, 10.0, -7.0);
    cube_po0->set_velocity(point_t(1.9, 0.0, 0.0));
    cube_po0->set_angular_velocity(point_t(0.0, 0.0, -2.0));
    cube_po0->apply_impulse(im1, p1);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity() - point_t(-3.1, 10.0, -7.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - point_t(0.0, 0.0, -2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum() - cube_po0->get_velocity())) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (cube_po0->get_angular_velocity() * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
      
    const point_t p2(0.0, 8.0, 0.0);
    const point_t im2(4.5, 2.0, -10.0);
    cube_po1->set_velocity(point_t(1.9, 0.0, -10.1));
    cube_po1->set_angular_velocity(point_t(-4.3, 17.9, 11.0));
    cube_po1->apply_impulse(im2, p2);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_velocity() - point_t(6.4, 2.0, -20.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_angular_velocity() - point_t(85.7, 17.9, 51.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_momentum() - cube_po1->get_velocity())) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_angular_momentum() - (cube_po1->get_angular_velocity() * point_t(0.166667, 0.166667, 0.166667)))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( apply_impulse_with_pre_computes_test, physics_object_fixture )
{
    const point_t n0(0.0, 1.0, 0.0);
    const point_t aw0(0.3905667329, -0.1301889110, 0.9113223769);
    const float i0 = 1.0;
    plane_po->apply_impulse(n0, aw0, i0);
    BOOST_CHECK(plane_po->get_velocity() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(plane_po->get_angular_velocity() - point_t(0.3905667329, -0.1301889110, 0.9113223769))) < result_tolerance);
    BOOST_CHECK(plane_po->get_momentum() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(plane_po->get_angular_momentum() == point_t(0.0, 0.0, 0.0));

    const point_t n1(0.7071067812, 0.0, 0.7071067812);
    const point_t aw1(0.0, -0.7071067812, -0.7071067812);
    const float i1 = 12.7;
    cube_po0->set_velocity(point_t(1.9, 0.0, 0.0));
    cube_po0->set_angular_velocity(point_t(0.0, 0.0, -2.0));
    cube_po0->apply_impulse(n1, aw1, i1);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity() - point_t(10.8803, 0.0, 8.98026))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - point_t(0.0, -8.98026, -10.9803))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum() - cube_po0->get_velocity())) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (cube_po0->get_angular_velocity() * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
      
    const point_t n2(0.1301889110, 0.3905667329, 0.9113223769);
    const point_t aw2(0.0, 0.0, 1.0);
    const float i2 = 4.9;
    cube_po1->set_velocity(point_t(1.9, 0.0, -10.1));
    cube_po1->set_angular_velocity(point_t(-4.3, 17.9, 11.0));
    cube_po1->apply_impulse(n2, aw2, i2);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_velocity() - point_t(2.53793, 1.91378, -5.63452))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_angular_velocity() - point_t(-4.3, 17.9, 15.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_momentum() - cube_po1->get_velocity())) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po1->get_angular_momentum() - (cube_po1->get_angular_velocity() * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
}


/* Test build triangles */
BOOST_FIXTURE_TEST_CASE( build_triangles_test, physics_object_fixture )
{
    /* Test plane */
    raptor_raytracer::primitive_list p0;
    plane_po->triangles(&p0);
    BOOST_CHECK(p0.size() == 2);

    BOOST_CHECK(p0[0]->get_vertex_a() == point_t(-10.0, -10.0, -10.0));
    BOOST_CHECK(p0[0]->get_vertex_b() == point_t( 10.0, -10.0,  10.0));
    BOOST_CHECK(p0[0]->get_vertex_c() == point_t(-10.0, -10.0,  10.0));

    BOOST_CHECK(p0[1]->get_vertex_a() == point_t(-10.0, -10.0, -10.0));
    BOOST_CHECK(p0[1]->get_vertex_b() == point_t( 10.0, -10.0, -10.0));
    BOOST_CHECK(p0[1]->get_vertex_c() == point_t( 10.0, -10.0,  10.0));

    for (auto *tri : p0)
    {
        delete tri;
    }

    /* Test cube */
    raptor_raytracer::primitive_list p1;
    cube_po0->triangles(&p1);
    BOOST_CHECK(p1.size() == 12);

    BOOST_CHECK(p1[0]->get_vertex_a() == point_t( 0.0, -10.0, -0.5));
    BOOST_CHECK(p1[0]->get_vertex_b() == point_t( 1.0,  -9.0, -0.5));
    BOOST_CHECK(p1[0]->get_vertex_c() == point_t( 1.0, -10.0, -0.5));

    BOOST_CHECK(p1[1]->get_vertex_a() == point_t( 0.0, -10.0, -0.5));
    BOOST_CHECK(p1[1]->get_vertex_b() == point_t( 0.0,  -9.0, -0.5));
    BOOST_CHECK(p1[1]->get_vertex_c() == point_t( 1.0,  -9.0, -0.5));

    BOOST_CHECK(p1[2]->get_vertex_a() == point_t( 0.0, -10.0,  0.5));
    BOOST_CHECK(p1[2]->get_vertex_b() == point_t( 1.0, -10.0,  0.5));
    BOOST_CHECK(p1[2]->get_vertex_c() == point_t( 1.0,  -9.0,  0.5));

    BOOST_CHECK(p1[3]->get_vertex_a() == point_t( 0.0, -10.0,  0.5));
    BOOST_CHECK(p1[3]->get_vertex_b() == point_t( 1.0,  -9.0,  0.5));
    BOOST_CHECK(p1[3]->get_vertex_c() == point_t( 0.0,  -9.0,  0.5));

    BOOST_CHECK(p1[4]->get_vertex_a() == point_t( 0.0, -10.0,  0.5));
    BOOST_CHECK(p1[4]->get_vertex_b() == point_t( 0.0,  -9.0,  0.5));
    BOOST_CHECK(p1[4]->get_vertex_c() == point_t( 0.0, -10.0, -0.5));

    BOOST_CHECK(p1[5]->get_vertex_a() == point_t( 0.0,  -9.0,  0.5));
    BOOST_CHECK(p1[5]->get_vertex_b() == point_t( 0.0,  -9.0, -0.5));
    BOOST_CHECK(p1[5]->get_vertex_c() == point_t( 0.0, -10.0, -0.5));

    BOOST_CHECK(p1[6]->get_vertex_a() == point_t( 1.0, -10.0, -0.5));
    BOOST_CHECK(p1[6]->get_vertex_b() == point_t( 1.0,  -9.0, -0.5));
    BOOST_CHECK(p1[6]->get_vertex_c() == point_t( 1.0,  -9.0,  0.5));

    BOOST_CHECK(p1[7]->get_vertex_a() == point_t( 1.0, -10.0, -0.5));
    BOOST_CHECK(p1[7]->get_vertex_b() == point_t( 1.0,  -9.0,  0.5));
    BOOST_CHECK(p1[7]->get_vertex_c() == point_t( 1.0, -10.0,  0.5));

    BOOST_CHECK(p1[8]->get_vertex_a() == point_t( 0.0,  -9.0, -0.5));
    BOOST_CHECK(p1[8]->get_vertex_b() == point_t( 0.0,  -9.0,  0.5));
    BOOST_CHECK(p1[8]->get_vertex_c() == point_t( 1.0,  -9.0,  0.5));

    BOOST_CHECK(p1[9]->get_vertex_a() == point_t( 0.0,  -9.0, -0.5));
    BOOST_CHECK(p1[9]->get_vertex_b() == point_t( 1.0,  -9.0,  0.5));
    BOOST_CHECK(p1[9]->get_vertex_c() == point_t( 1.0,  -9.0, -0.5));

    BOOST_CHECK(p1[10]->get_vertex_a() == point_t( 0.0, -10.0, -0.5));
    BOOST_CHECK(p1[10]->get_vertex_b() == point_t( 1.0, -10.0, -0.5));
    BOOST_CHECK(p1[10]->get_vertex_c() == point_t( 0.0, -10.0,  0.5));

    BOOST_CHECK(p1[11]->get_vertex_a() == point_t( 1.0, -10.0, -0.5));
    BOOST_CHECK(p1[11]->get_vertex_b() == point_t( 1.0, -10.0,  0.5));
    BOOST_CHECK(p1[11]->get_vertex_c() == point_t( 0.0, -10.0,  0.5));

    for (auto *tri : p1)
    {
        delete tri;
    }
}


/* Test commit movement */
BOOST_FIXTURE_TEST_CASE( commit_movement_no_applied_force_test, physics_object_fixture )
{
    /* Test plane */
    const float t0 = 0.2;
    const point_t v0(10.2, 4.5, -7.2);
    const point_t w0(0.2, -3.4, -1.9);
    plane_po->set_velocity(v0);
    plane_po->set_angular_velocity(w0);
    plane_po->begin_time_step(t0);
    plane_po->commit_movement(t0);
    
    /* Check kinematics */
    BOOST_CHECK(fabs(magnitude(plane_po->get_velocity()         - v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_angular_velocity() - w0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_momentum()         - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_angular_momentum() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_center_of_mass()   - point_t(2.04, -9.1, -1.44))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_orientation()      - quaternion_t(0.924936, 0.0194935, -0.331389, -0.185188))) < result_tolerance);

    /* Test cube */
    const float t1 = 1.3;
    const point_t v1(-1.7, 2.3, 1.1);
    const point_t w1(10.7, -20.6, 4.8);
    cube_po0->set_velocity(v1);
    cube_po0->set_angular_velocity(w1);
    cube_po0->begin_time_step(t1);
    cube_po0->commit_movement(t1);
    
    /* Check kinematics */
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - w1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (w1 * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-1.71, -6.51, 1.43))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(0.966299, -0.1162, 0.223712, -0.052127))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( commit_movement_test, physics_object_fixture )
{
    /* Test plane */
    const float t0 = 0.2;
    const point_t v0(10.2, 4.5, -7.2);
    const point_t w0(0.2, -3.4, -1.9);
    mock_force *const mf0 = new mock_force(point_t(-1.8, 8.7, 3.5), point_t(-7.5, 1.4, 4.6), t0);
    plane_po->register_force(mf0);
    plane_po->set_velocity(v0);
    plane_po->set_angular_velocity(w0);
    plane_po->begin_time_step(t0);
    plane_po->commit_movement(t0);
    
    /* Check kinematics */
    BOOST_CHECK(fabs(magnitude(plane_po->get_velocity()         - v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_angular_velocity() - w0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_momentum()         - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_angular_momentum() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_center_of_mass()   - point_t(2.04, -9.1, -1.44))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(plane_po->get_orientation()      - quaternion_t(0.924936, 0.0194935, -0.331389, -0.185188))) < result_tolerance);

    /* Test cube */
    const float t1 = 1.3;
    const point_t v1(-1.7, 2.3, 1.1);
    const point_t w1(10.7, -20.6, 4.8);
    mock_force *const mf1 = new mock_force(point_t(-1.8, 8.7, 3.5), point_t(-7.5, 1.4, 4.6), t1);
    cube_po0->register_force(mf1);
    cube_po0->set_velocity(v1);
    cube_po0->set_angular_velocity(w1);
    cube_po0->begin_time_step(t1);
    cube_po0->commit_movement(t1);
    
    /* Check kinematics */
    const point_t exp_v(-4.04, 13.61, 5.65);
    const point_t exp_w(10.7, -20.6, 4.8);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - exp_v)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - exp_w)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - exp_v)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (exp_w * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-3.231, 0.841499, 4.3875))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(0.967707, -0.11614, 0.218197, -0.0494436))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( commit_movement_two_steps_test, physics_object_fixture )
{
    const float t0 = 0.5;
    const float t1 = 1.3;
    const point_t v(-1.7, 2.3, 1.1);
    const point_t w(10.7, -20.6, 4.8);
    mock_force *const mf = new mock_force(point_t(-1.8, 8.7, 3.5), point_t(-7.5, 1.4, 4.6), 1.8);
    cube_po0->register_force(mf);
    cube_po0->set_velocity(v);
    cube_po0->set_angular_velocity(w);
    cube_po0->begin_time_step(t1);
    cube_po0->commit_movement(t0);
    
    /* Check kinematics */
    const point_t exp_v0(-2.6, 6.65, 2.85);
    const point_t exp_w0(5.62477, -11.0001, 2.53721);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - exp_v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - exp_w0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - exp_v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (exp_w0 * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-0.575, -7.2625, 0.9875))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(0.502279, -0.39563, 0.748378, -0.176415))) < result_tolerance);

    /* Step again */
    cube_po0->commit_movement(t1);
    
    /* Check kinematics */
    const point_t exp_v1(-4.04, 13.61, 5.65);
    const point_t exp_w1(2.5848, -20.2512, 4.9999);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - exp_v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - exp_w1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - exp_v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (exp_w1 * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-3.231, 0.841499, 4.3875))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(-0.611231, 0.239536, 0.728947, -0.19405))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( commit_movement_two_steps_not_backwards_test, physics_object_fixture )
{
    const float t0 = 1.3;
    const point_t v(-1.7, 2.3, 1.1);
    const point_t w(10.7, -20.6, 4.8);
    mock_force *const mf = new mock_force(point_t(-1.8, 8.7, 3.5), point_t(-7.5, 1.4, 4.6), 1.8);
    cube_po0->register_force(mf);
    cube_po0->set_velocity(v);
    cube_po0->set_angular_velocity(w);
    cube_po0->begin_time_step(t0);
    cube_po0->commit_movement(t0);
    
    /* Check kinematics */
    const point_t exp_v0(-4.04, 13.61, 5.65);
    const point_t exp_w0(10.7, -20.6, 4.8);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - exp_v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - exp_w0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - exp_v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (exp_w0 * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-3.231, 0.841499, 4.3875))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(0.967707, -0.11614, 0.218197, -0.0494436))) < result_tolerance);

    /* Step again */
    const float t1 = 0.5;
    cube_po0->commit_movement(t1);
    
    /* Check kinematics */
    const point_t exp_v1(-4.04, 13.61, 5.65);
    const point_t exp_w1(10.7, -20.6, 4.8);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - exp_v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - exp_w1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - exp_v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (exp_w1 * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-3.231, 0.841499, 4.3875))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(0.967707, -0.11614, 0.218197, -0.0494436))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( commit_movement_two_steps_with_reset_test, physics_object_fixture )
{
    const float t0 = 0.5;
    const point_t f(-1.8, 8.7, 3.5);
    const point_t a(-7.5, 1.4, 4.6);
    const point_t v(-1.7, 2.3, 1.1);
    const point_t w(10.7, -20.6, 4.8);
    mock_force *const mf = new mock_force(f, a, 1.3);
    cube_po0->register_force(mf);
    cube_po0->set_velocity(v);
    cube_po0->set_angular_velocity(w);
    cube_po0->begin_time_step(t0);
    cube_po0->commit_movement(t0);
    
    /* Check kinematics */
    const point_t exp_v0(-2.6, 6.65, 2.85);
    const point_t exp_w0(5.62477, -11.0001, 2.53721);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - exp_v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - exp_w0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - exp_v0)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (exp_w0 * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-0.575, -7.2625, 0.9875))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(0.502279, -0.39563, 0.748378, -0.176415))) < result_tolerance);

    /* Reset, reapply forces and step again */
    const float t1 = 0.8;
    mf->set_force(f);
    mf->set_torque(a);
    cube_po0->begin_time_step(t1);
    cube_po0->commit_movement(t1);
    
    /* Check kinematics */
    const point_t exp_v1(-4.04, 13.61, 5.65);
    const point_t exp_w1(2.49856, -20.3995, 5.00165);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_velocity()         - exp_v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_velocity() - exp_w1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_momentum()         - exp_v1)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_angular_momentum() - (exp_w1 * point_t(0.166672, 0.166667, 0.166672)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_center_of_mass()   - point_t(-3.231, 0.841499, 4.3875))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube_po0->get_orientation()      - quaternion_t(-0.611718, 0.242276, 0.728441, -0.190991))) < result_tolerance);
}


/* Test has collided */
BOOST_FIXTURE_TEST_CASE( self_collision_has_collided_test, physics_object_fixture )
{
    point_t d;
    simplex *manifold_a;
    simplex *manifold_b;
    const float t0 = 0.0;
    const float t1 = 1.0;

    /* Check not colliding */
    BOOST_CHECK(!cube_po0->has_collided(cube_po0, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(0.0, 0.0, 0.0)); 
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( no_movement_has_collided_test, physics_object_fixture )
{
    point_t d;
    simplex *manifold_a;
    simplex *manifold_b;
    const float t0 = 0.0;
    const float t1 = 1.0;

    /* Check not colliding */
    BOOST_CHECK(!cube_po0->has_collided(far_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(0.0, -18.0, 0.0));
    delete manifold_a;
    delete manifold_b;

    /* Check colliding */
    BOOST_CHECK(cube_po0->has_collided(touching_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(0.0, 0.0, 0.0));
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( one_translating_has_collided_test, physics_object_fixture )
{
    point_t d;
    simplex *manifold_a;
    simplex *manifold_b;
    const float t0 = 0.0;
    const float t1 = 1.0;

    /* Starting apart and heading away */
    mock_force *const mf = new mock_force(point_t(5.0, 1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf);
    cube_po0->set_velocity(point_t(10.0, -1.0, 2.0));
    BOOST_CHECK(!cube_po0->has_collided(far_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(0.0, -18.0, 0.0)); 
    delete manifold_a;
    delete manifold_b;

    /* Starting together and heading away */
    mf->set_force(point_t(-5.0, -1.0, -2.0));
    cube_po0->set_velocity(point_t(-10.0, -1.0, 2.0));
    BOOST_CHECK(cube_po0->has_collided(touching_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(0.0, 0.0, 0.0));
    delete manifold_a;
    delete manifold_b;

    /* Starting together and heading away, forcing displacement */
    mf->set_force(point_t(-5.0, -1.0, -2.0));
    cube_po0->set_velocity(point_t(-10.0, -1.0, 2.0));
    BOOST_CHECK(!cube_po0->has_collided(touching_po, &manifold_a, &manifold_b, &d, t1, t1));
    BOOST_CHECK(fabs(magnitude(d - point_t(-12.5, -1.5, 0.0))) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
    
    /* Starting apart and heading together */
    mf->set_force(point_t(0.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(0.0, 18.0, -0.5));
    BOOST_CHECK(cube_po0->has_collided(far_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(0.0, 0.0, 0.0)); 
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( two_translating_has_collided_test, physics_object_fixture )
{
    point_t d;
    simplex *manifold_a;
    simplex *manifold_b;
    const float t0 = 0.5;
    const float t1 = 1.5;

    /* Starting apart and heading away */
    mock_force *const mf0 = new mock_force(point_t( 5.0,  1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf1 = new mock_force(point_t( 5.0,  1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf2 = new mock_force(point_t(-5.0, -1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf0);
    far_po->register_force(mf1);
    cube_po0->set_velocity(point_t(10.0, -1.0, 2.0));
    far_po->set_velocity(point_t(-10.0, 1.0, 2.0));
    BOOST_CHECK(!cube_po0->has_collided(far_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(9.5, -19.0, 0.0));
    delete manifold_a;
    delete manifold_b;

    /* Starting together and heading away */
    mf0->set_force(point_t(-5.0, -1.0, -2.0));
    cube_po0->set_velocity(point_t(-10.0, -1.0, 2.0));
    nearer_po->register_force(mf2);
    nearer_po->set_velocity(point_t(10.0, 1.0, 2.0));
    BOOST_CHECK(!cube_po0->has_collided(nearer_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(magnitude(fabs(d - point_t(-9.375, -0.875, 0.0))) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together and heading away, forcing displacement */
    mf0->set_force(point_t(-5.0, -1.0, -2.0));
    cube_po0->set_velocity(point_t(0.0, 0.0, 0.0));
    mf2->set_force(point_t(5.0, 1.0, -2.0));
    nearer_po->set_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(!cube_po0->has_collided(nearer_po, &manifold_a, &manifold_b, &d, t1, t1));
    BOOST_CHECK(fabs(magnitude(d - point_t(-16.875, -3.375, 0.25))) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
    
    /* Starting apart and heading together */
    mf0->set_force(point_t(0.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(0.0, 8.0, -0.5));
    mf1->set_force(point_t(0.0, 0.0, 0.0));
    far_po->set_velocity(point_t(0.5, -10.0, -1.0));
    BOOST_CHECK(cube_po0->has_collided(far_po, &manifold_a, &manifold_b, &d, t0, t1));
    BOOST_CHECK(d == point_t(0.0, 0.0, 0.0)); 
    delete manifold_a;
    delete manifold_b;
}


/* Test resolve collision */
BOOST_FIXTURE_TEST_CASE( self_collision_exact_resolve_collisions_test, physics_object_fixture )
{
    float t = 10.0;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Check not colliding */
    BOOST_CHECK(cube_po0->resolve_collisions(cube_po0, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(t == 10.0); 
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( self_collision_conservative_resolve_collisions_test, physics_object_fixture )
{
    float t = 10.0;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Check not colliding */
    cube_po0->set_angular_velocity(point_t(0.1, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(cube_po0, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(t == 10.0); 
    delete manifold_a;
    delete manifold_b;

    mock_force *const mf = new mock_force(point_t(0.0, 0.0, 0.0), point_t(0.1, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf);
    cube_po0->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(cube_po0, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(t == 10.0); 
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( no_movement_resolve_collisions_no_collision_test, physics_object_fixture )
{
    float t = 10.0;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Check not colliding */
    BOOST_CHECK(cube_po0->resolve_collisions(far_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(t == 10.0);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( no_movement_resolve_collisions_sliding_collision_test, physics_object_fixture )
{
    float t = 10.0;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Check colliding */
    BOOST_CHECK(cube_po0->resolve_collisions(nearer_po, &manifold_a, &manifold_b, &t) == SLIDING_COLLISION);
    BOOST_CHECK(t == 10.0);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( one_translating_resolve_collisions_no_collision_test, physics_object_fixture )
{
    float t = 0.02;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Starting apart and heading away */
    mock_force *const mf = new mock_force(point_t(5.0, 1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf);
    cube_po0->set_velocity(point_t(10.0, -1.0, 2.0));
    BOOST_CHECK(cube_po0->resolve_collisions(far_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together and heading away */
    mf->set_force(point_t(-5.0, -1.0, -2.0));
    cube_po0->set_velocity(point_t(-10.0, -1.0, 2.0));
    BOOST_CHECK(cube_po0->resolve_collisions(nearer_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together, heading together, but turning back again */
    /* Technically there may have been a collision, but who could see it */
    mf->set_force(point_t(-1000.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(0.5, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( one_translating_resolve_collisions_sliding_collision_test, physics_object_fixture )
{
    float t = 0.02;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Starting together and stay near */
    mock_force *const mf = new mock_force(point_t(0.0, 5.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf);
    cube_po0->set_velocity(point_t(0.0, 10.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == SLIDING_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting close and getting a little bit closer, but never penetrating */
    mf->set_force(point_t(5.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(10.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(near_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(t < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together and getting closer */
    t = 0.02;
    mf->set_force(point_t(5.0, 6.0, 8.0));
    cube_po0->set_velocity(point_t(10.0, 3.0, 7.0));
    BOOST_CHECK(cube_po0->resolve_collisions(near_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(t < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together, heading apart, but coming back again */
    t = 0.02;
    mf->set_force(point_t(1000.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(-0.5, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == SLIDING_COLLISION);
    BOOST_CHECK(fabs(t - 0.001) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Attempting to pass through */
    t = 0.02;
    mf->set_force(point_t(0.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(1000.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(t < result_tolerance);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( one_translating_resolve_collisions_collision_test, physics_object_fixture )
{
    float t = 1.0;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Starting apart and heading together */
    cube_po0->set_velocity(point_t(0.0, 18.0, -0.5));
    BOOST_CHECK(cube_po0->resolve_collisions(far_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(fabs(t - 0.999951) < result_tolerance); 
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( two_translating_resolve_collisions_no_collision_test, physics_object_fixture )
{
    float t = 0.02;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Starting apart and heading away */
    mock_force *const mf0 = new mock_force(point_t( 5.0,  1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf1 = new mock_force(point_t( 5.0,  1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf2 = new mock_force(point_t(-5.0, -1.0, -2.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf3 = new mock_force(point_t(2000.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf0);
    far_po->register_force(mf1);
    cube_po0->set_velocity(point_t(10.0, -1.0, 2.0));
    far_po->set_velocity(point_t(-10.0, 1.0, 2.0));
    BOOST_CHECK(cube_po0->resolve_collisions(far_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together and heading away */
    mf0->set_force(point_t(-5.0, -1.0, -2.0));
    cube_po0->set_velocity(point_t(-10.0, -1.0, 2.0));
    nearer_po->register_force(mf2);
    nearer_po->set_velocity(point_t(10.0, 1.0, 2.0));
    BOOST_CHECK(cube_po0->resolve_collisions(nearer_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together, heading together, but turning back again */
    /* Technically there may have been a collision, but who could see it */
    mf0->set_force(point_t(-1000.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(0.5, 0.0, 0.0));
    hit_except_x_po->register_force(mf3);
    hit_except_x_po->set_velocity(point_t(-1.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( two_translating_resolve_collisions_sliding_collision_test, physics_object_fixture )
{
    float t = 0.02;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Starting together and stay near */
    mock_force *const mf0 = new mock_force(point_t( 0.0, -5.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf1 = new mock_force(point_t( 0.0,  0.0, 1.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf2 = new mock_force(point_t( 6.0, -5.1, 0.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf0);
    hit_except_x_po->register_force(mf1);
    cube_po0->set_velocity(point_t(0.0, 10.0, 0.0));
    hit_except_x_po->set_velocity(point_t(0.0, 0.0 -7.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting close and getting a little bit closer, but never penetrating */
    mf0->set_force(point_t(5.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(10.0, 0.0, 0.0));
    near_po->register_force(mf2);
    near_po->set_velocity(point_t(7.0, -10.7, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(near_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(t < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together and getting closer */
    t = 0.02;
    mf0->set_force(point_t(5.0, 6.0, 8.0));
    cube_po0->set_velocity(point_t(10.0, 3.0, 7.0));
    mf2->set_force(point_t(4.0, -1.0, -6.5));
    near_po->set_velocity(point_t(-10.0, 1.0, 2.0));
    BOOST_CHECK(cube_po0->resolve_collisions(near_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(t < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together, heading apart, but coming back again */
    t = 0.02;
    mf0->set_force(point_t(1000.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(-0.5, 0.0, 0.0));
    mf1->set_force(point_t(-2000.0, 0.0, 0.0));
    hit_except_x_po->set_velocity(point_t(1.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == SLIDING_COLLISION);
    BOOST_CHECK(fabs(t - 0.001) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Attempting to pass through */
    t = 0.02;
    mf0->set_force(point_t(0.0, 0.0, 0.0));
    cube_po0->set_velocity(point_t(1000.0, 0.0, 0.0));
    mf1->set_force(point_t(0.0, 0.0, 0.0));
    hit_except_x_po->set_velocity(point_t(-2000.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(hit_except_x_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(t < result_tolerance);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( two_translating_resolve_collisions_collision_test, physics_object_fixture )
{
    float t = 1.0;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Starting apart and heading together */
    mock_force *const mf0 = new mock_force(point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0);
    mock_force *const mf1 = new mock_force(point_t( 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po0->register_force(mf0);
    far_po->register_force(mf1);
    cube_po0->set_velocity(point_t(0.0, 8.0, -0.5));
    far_po->set_velocity(point_t(0.5, -10.0, -1.0));
    BOOST_CHECK(cube_po0->resolve_collisions(far_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(fabs(t - 0.999972) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( translating_solver_resolve_collisions_test, physics_object_fixture )
{
    float t = 5.0;
    simplex *manifold_a;
    simplex *manifold_b;

    /* General solver test */
    mock_force *const mf = new mock_force(point_t(0.0, -10.8, 0.0), point_t(0.0, 0.0, 0.0), 1.0);
    cube_po1->register_force(mf);
    cube_po1->set_velocity(point_t(0.0, -8.0, -0.5));
    BOOST_CHECK(cube_po1->resolve_collisions(plane_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(fabs(t - 1.27597) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
    
    /* Test solver with an over shooting predict */
    t = 5.0;
    mf->set_force(point_t(0.0, -10.7, 0.0));
    cube_po1->set_velocity(point_t(0.0, -8.0, -0.5));
    BOOST_CHECK(cube_po1->resolve_collisions(plane_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(fabs(t - 1.27972) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Test solver with a to close predict */
    t = 5.0;
    mf->set_force(point_t(0.0, -10.81, 0.0));
    cube_po1->set_velocity(point_t(0.0, -8.0, -0.5));
    BOOST_CHECK(cube_po1->resolve_collisions(plane_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(fabs(t - 1.27597) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* An extra one just for good measure */
    t = 5.0;
    mf->set_force(point_t(0.0, -9.82, 0.0));
    cube_po1->set_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po1->resolve_collisions(plane_po, &manifold_a, &manifold_b, &t) == COLLISION);
    BOOST_CHECK(fabs(t - 1.96712) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
}


BOOST_FIXTURE_TEST_CASE( one_rotating_resolve_collisions_test, physics_object_fixture )
{
    float t = 0.02;
    simplex *manifold_a;
    simplex *manifold_b;

    /* Starting apart and heading away */
    mock_force *const mf0 = new mock_force(point_t(0.0, 0.0, 0.0), point_t( 5.0,  1.0, -2.0), 1.0);
    mock_force *const mf1 = new mock_force(point_t(0.0, 0.0, 0.0), point_t(-5.0, -1.0, -2.0), 1.0);
    far_po->register_force(mf0);
    far_po->set_angular_velocity(point_t(10.0, -1.0, 2.0));
    BOOST_CHECK(cube_po0->resolve_collisions(far_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together and heading away */
    near_po->register_force(mf1);
    near_po->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(near_po, &manifold_a, &manifold_b, &t) == NO_COLLISION);
    BOOST_CHECK(fabs(t - 0.02) < result_tolerance);
    delete manifold_a;
    delete manifold_b;

    /* Starting together and stay near */
    // t = 0.02;
    // cube_po0->set_force(point_t(0.0, 0.0, 0.0));
    // cube_po0->set_velocity(point_t(0.0, 0.0, 0.0));
    // near_po->set_velocity(point_t(0.0, -10.0, -7.8));
    // near_po->set_torque(point_t(1.0, 0.0, 0.0));
    // near_po->set_angular_velocity(point_t(5.0, 0.0, 0.0));
    // BOOST_CHECK(cube_po0->resolve_collisions(near_po, &manifold_a, &manifold_b, &t) == SLIDING_COLLISION);
    // BOOST_CHECK(fabs(t - 0.02) < result_tolerance, t);
    // delete manifold_a;
    // delete manifold_b;

    /* Starting together and getting closer */
    mf1->set_torque(point_t(5.0, 0.0, 0.0));
    near_po->set_velocity(point_t(-1.0, 0.0, 0.0));
    near_po->set_angular_velocity(point_t(1.7, 0.0, 0.0));
    BOOST_CHECK(cube_po0->resolve_collisions(near_po, &manifold_a, &manifold_b, &t) == SLIDING_COLLISION);
    BOOST_CHECK(t == 0.0);
    delete manifold_a;
    delete manifold_b;
    
    /* Starting apart and heading together */
    t = 1.0;
    cube_po0->set_velocity(point_t(0.0, 18.0, -0.5));
    mf1->set_torque(point_t(1.7, 8.4, 7.2));
    near_po->set_angular_velocity(point_t(-5.0, -1.0, -2.0));
    BOOST_CHECK(cube_po0->resolve_collisions(far_po, &manifold_a, &manifold_b, &t) == POSSIBLE_COLLISION);
    BOOST_CHECK(fabs(t - 0.980449) < result_tolerance);
    delete manifold_a;
    delete manifold_b;
}


// /* Check for collisions and separate objects as needed */
// collision_t resolve_collisions(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, float *const t)
// {
//     /* If no rotation use exact */
//     if ((fabs(magnitude(    _w)) < raptor_physics::EPSILON) && (fabs(magnitude(    _tor)) < raptor_physics::EPSILON) &&
//         (fabs(magnitude(vg->_w)) < raptor_physics::EPSILON) && (fabs(magnitude(vg->_tor)) < raptor_physics::EPSILON))
//     {
//         return exactly_resolve_collisions(vg, manifold_a, manifold_b, t);
//     }
//     /* Else must be conservative */
//     else
//     {
//         return conservatively_resolve_collisions(vg, manifold_a, manifold_b, t);
//     }
// }

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
