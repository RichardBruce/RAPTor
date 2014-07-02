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
    raptor_physics::physics_engine pe(new raptor_physics::rigid_body_collider(0.9, 0.75));
    raptor_physics::simulation_environment se(&pe, &po);

    /* Lights */
    se.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    std::unique_ptr<material> m(new phong_shader(ext_colour_t(255.0, 255.0, 255.0), 1.0));
    se.add_moving_object(new raptor_physics::physics_object(raptor_physics::make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, 9.5, 0.0), 10.0));

    /* Add static objects */
    se.add_object(new raptor_physics::physics_object(raptor_physics::make_plane(m.get(), point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0)), point_t(0.0, -10.0, 0.0), numeric_limits<fp_t>::infinity()));

    /* Apply forces */
    se.engine()->apply_force(new raptor_physics::const_force(point_t(0.0, 0.5, 0.0), point_t(50.0, 0.0, 0.0), 0.5), 0);

    /* Run physics simulation */
    return se.run();
}
// LCOV_EXCL_STOP
