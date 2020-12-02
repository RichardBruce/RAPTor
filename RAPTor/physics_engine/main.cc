/* Common headers */
#include "common.h"
#include "point_t.h"

/* Raytracer headers */
#include "bih.h"
#include "phong_shader.h"

/* Physics headers */
#include "isosurface.h"
#include "meta_ball.h"
#include "particle.h"
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
    // raptor_physics::physics_options po(argv, argc);

    // /* Set up the physics simulation */
    // raptor_physics::physics_engine pe(new raptor_physics::rigid_body_collider(0.75f, 0.75f, 0.5f));
    // raptor_physics::simulation_environment se(&pe, &po);

    // /* Lights */
    // se.add_light(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), point_t(10.0f,  10.0f, -100.0f));
    // // se.add_light(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), point_t(10.0f, -20.0f, -100.0f));

    // /* Add moving objects */
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), 1.0f));
    // // auto po0 = new raptor_physics::physics_object(raptor_physics::make_cube(m.get(), point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, 5.0f, 0.0f), 10.0f);
    // // po0->set_angular_velocity(point_t(0.0f, 0.0f, 5.0f));
    // // se.add_moving_object(po0);

    // //  Add static objects 
    // // se.add_object(new raptor_physics::physics_object(raptor_physics::make_plane(m.get(), point_t(-10.0f, 0.0f, -10.0f), point_t(10.0f, 0.0f, -10.0f), point_t(-10.0f, 0.0f, 10.0f), point_t(10.0f, 0.0f, 10.0f)), point_t(0.0f, -10.0f, 0.0f), std::numeric_limits<float>::infinity()));

    // auto po0 = new raptor_physics::physics_object(raptor_physics::make_cube(m.get(), point_t(-0.5f, -0.5f, -0.5f), point_t(0.5f, 0.5f, 0.5f)), point_t(0.0f, -9.0f, 0.0f), 10.0f);
    // se.add_moving_object(po0);

    // /* Add static objects */
    // se.add_object(new raptor_physics::physics_object(raptor_physics::make_plane(m.get(), point_t(-10.0f, 0.0f, -10.0f), point_t(10.0f, 0.0f, -10.0f), point_t(-10.0f, 0.0f, 10.0f), point_t(10.0f, 0.0f, 10.0f)), point_t(0.0f, -10.0f, 0.0f), std::numeric_limits<float>::infinity()));

    // /* Apply forces */
    // se.engine()->apply_force(new raptor_physics::const_force(point_t(0.0f, 0.0f, 0.0f), point_t(10.0f, 0.0f, 0.0f), 5.0f), 0);
    // // se.engine()->apply_force(new raptor_physics::const_force(point_t(0.0f, -0.5f, 0.0f), point_t(-5.0f, 0.0f, 0.0f), 0.5f), 0);

    /* Run physics simulation */
    // return se.run();



    std::vector<raptor_physics::meta_ball> meta_balls;
    meta_balls.emplace_back(point_t( -5.0f,  16.0f,   2.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  5.0f,  16.0f,   2.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,  12.0f,  -1.0f), 12.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   7.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   2.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,   0.0f,   2.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,   0.0f,   2.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  2.0f,  -7.0f,  -1.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -2.0f,  -7.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f, -12.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f, -14.0f,   2.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f, -14.0f,   2.0f), 10.0f, false);


    meta_balls.emplace_back(point_t( -5.0f,   2.0f,  16.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  5.0f,   2.0f,  16.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,  -1.0f,  12.0f), 12.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   0.0f,   7.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   0.0f,   2.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,   2.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,   2.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  2.0f,  -1.0f,  -7.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -2.0f,   0.0f,  -7.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   0.0f, -12.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,   2.0f, -14.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,   2.0f, -14.0f), 10.0f, false);

    meta_balls.emplace_back(point_t( 16.0f, -5.0f,   2.0f),  6.0f, false);
    meta_balls.emplace_back(point_t( 16.0f,  5.0f,   2.0f),  6.0f, false);
    meta_balls.emplace_back(point_t( 12.0f,  0.0f,  -1.0f), 12.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,  0.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  2.0f,  0.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f, -7.0f,   2.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,  7.0f,   2.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,  2.0f,  -1.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f, -2.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(-12.0f,  0.0f,   0.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(-14.0f, -7.0f,   2.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(-14.0f,  7.0f,   2.0f), 10.0f, false);

    meta_balls.emplace_back(point_t( -5.0f,  16.0f,   22.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  5.0f,  16.0f,   22.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,  12.0f,  -21.0f), 12.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   7.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   2.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,   0.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,   0.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  2.0f,  -7.0f,  -21.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -2.0f,  -7.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f, -12.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f, -14.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f, -14.0f,   22.0f), 10.0f, false);


    meta_balls.emplace_back(point_t( -5.0f,   2.0f,  216.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  5.0f,   2.0f,  216.0f),  6.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,  -1.0f,  212.0f), 12.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   0.0f,   27.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   0.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,   2.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,   2.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  2.0f,  -1.0f,  -27.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -2.0f,   0.0f,  -27.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,   0.0f, -212.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,   2.0f, -214.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,   2.0f, -214.0f), 10.0f, false);

    meta_balls.emplace_back(point_t( 16.0f, -5.0f,   22.0f),  6.0f, false);
    meta_balls.emplace_back(point_t( 16.0f,  5.0f,   22.0f),  6.0f, false);
    meta_balls.emplace_back(point_t( 12.0f,  0.0f,  -21.0f), 12.0f, false);
    meta_balls.emplace_back(point_t(  7.0f,  0.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  2.0f,  0.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f, -7.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(  0.0f,  7.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f,  2.0f,  -21.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -7.0f, -2.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(-12.0f,  0.0f,   20.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(-14.0f, -7.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t(-14.0f,  7.0f,   22.0f), 10.0f, false);
    meta_balls.emplace_back(point_t( -5.0f, 0.0f, 0.0f), 7.0f, true);

    auto *tris = new raptor_raytracer::primitive_store();
    raptor_physics::isosurface surface(meta_balls, m.get(), 0.25f);
    surface.to_triangles(tris);
    std::cout << "Created " << tris->size() << " triangles" << std::endl;

    const point_t cam_p( 0.0f,  0.0f, 50.0f);                                                      /* Position                 */
    const point_t x_vec( 1.0f,  0.0f,  0.0f);                                                        /* Horizontal vector        */
    const point_t y_vec( 0.0f,  1.0f,  0.0f);                                                        /* Virtical vector          */
    const point_t z_vec( 0.0f,  0.0f, -1.0f);                                                        /* Forward vector           */
    const raptor_raytracer::ext_colour_t bg(0.0f, 0.0f, 0.0f);                                      /* Background colour        */
    const unsigned xr = 640;                                                                        /* X resolution             */
    const unsigned yr = 480;                                                                        /* Y resolution             */
    const unsigned xa = 1;                                                                          /* X anti-aliasing factor   */
    const unsigned ya = 1;                                                                          /* Y anti-aliasing factor   */
    const float screen_width    = 10.0f;                                                            /* Width of screen          */
    const float screen_height   = screen_width * (static_cast<float>(yr) / static_cast<float>(xr)); /* Height of screen         */
    raptor_raytracer::camera cam(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 10, xr, yr, xa, ya, 0.0f, 0.0f);

    raptor_raytracer::light_list lights;
    new_light(&lights, raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), point_t(0.0f,  0.0f, 100.0f), 0.0f, 0.0f);
    
    std::unique_ptr<raptor_raytracer::bih> ssd(new raptor_raytracer::bih(*tris));
    raptor_raytracer::ray_tracer(ssd.get(), lights, *tris, cam);
    cam.write_tga_file("test.tag");
}
// LCOV_EXCL_STOP
