/* Common headers */
#include "common.h"
#include "point_t.h"

/* Raytracer headers */
#include "phong_shader.h"

/* Physics headers */
#include "physics_options.h"
#include "physics_engine.h"
#include "physics_object.h"
#include "simulation_environment.h"
#include "rigid_body_collider.h"
#include "force.h"


// LCOV_EXCL_START
/* Initialise the logger */
const raptor_physics::init_logger init_logger;


/* Main routine */
int main(int argc, char **argv)
{
    /* Parse input arguements */
    raptor_physics::physics_options po(argv, argc);

    /* Set up the physics simulation */
    raptor_physics::physics_engine pe(new raptor_physics::rigid_body_collider(0.75f, 0.75f));
    raptor_physics::simulation_environment se(&pe, &po);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), point_t(10.0f,  10.0f, -100.0f));
    // se.add_light(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), point_t(10.0f, -20.0f, -100.0f));

    /* Add moving objects */
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), 1.0f));
    // auto po0 = new raptor_physics::physics_object(raptor_physics::make_cube(m.get(), point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, 5.0f, 0.0f), 10.0f);
    // po0->set_angular_velocity(point_t(0.0f, 0.0f, 5.0f));
    // se.add_moving_object(po0);

    //  Add static objects 
    // se.add_object(new raptor_physics::physics_object(raptor_physics::make_plane(m.get(), point_t(-10.0f, 0.0f, -10.0f), point_t(10.0f, 0.0f, -10.0f), point_t(-10.0f, 0.0f, 10.0f), point_t(10.0f, 0.0f, 10.0f)), point_t(0.0f, -10.0f, 0.0f), std::numeric_limits<float>::infinity()));

    auto po0 = new raptor_physics::physics_object(raptor_physics::make_cube(m.get(), point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, -9.0f, 0.0f), 10.0f);
    se.add_moving_object(po0);

    /* Add static objects */
    se.add_object(new raptor_physics::physics_object(raptor_physics::make_plane(m.get(), point_t(-10.0f, 0.0f, -10.0f), point_t(10.0f, 0.0f, -10.0f), point_t(-10.0f, 0.0f, 10.0f), point_t(10.0f, 0.0f, 10.0f)), point_t(0.0f, -10.0f, 0.0f), std::numeric_limits<float>::infinity()));

    /* Apply forces */
    se.engine()->apply_force(new raptor_physics::const_force(point_t(0.0f, 0.0f, 0.0f), point_t(10.0f, 0.0f, 0.0f), 5.0f), 0);
    // se.engine()->apply_force(new raptor_physics::const_force(point_t(0.0f, -0.5f, 0.0f), point_t(-5.0f, 0.0f, 0.0f), 0.5f), 0);

    /* Run physics simulation */
    return se.run();
}
// LCOV_EXCL_STOP
