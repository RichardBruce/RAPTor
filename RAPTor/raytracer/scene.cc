#include "scene.h"
#include "parser_common.h"


namespace raptor_raytracer
{
/* Auto generated init function do not edit directly */
void scene_init(light_list &lights, primitive_list &everything, list<material *> &materials, camera **c)
{
    /* Set up the camera */
    *c = new camera(point_t(0.0, 0.0, 0.0), vector_t(1.0, 0.0, 0.0), vector_t(0.0, 1.0, 0.0), vector_t(0.0, 0.0, 1.0), ext_colour_t(0.0,0.0,255.0), (fp_t)7.5, (fp_t)10.0, (fp_t)10.0, 640, 480);

    phong_shader *frame_mat = new phong_shader(ext_colour_t(125, 125, 125), 0.3330, 0.3330, 0.0300);
    materials.push_back(frame_mat);

    triangle *tr0 = new triangle(frame_mat, point_t(-10.0, -10.0, 100.0), point_t(-10.0, 10.0, 100.0), point_t(10.0, 10.0, 100.0));
    everything.push_back(tr0);
    
    triangle *tr1 = new triangle(frame_mat, point_t(10.0, 10.0, 100.0), point_t(10.0, -10.0, 100.0), point_t(-10.0, -10.0, 100.0));
    everything.push_back(tr1);

    new_light(&lights, ext_colour_t(255.0,255.0,255.0), point_t(2.5, -2.5, 50.0), 0.0, 0.0);

    /* Assert there must be some lights to light the scene */
    assert(!lights.empty());

    /* Assert there is an imagine to trace */
    assert(!everything.empty());
    
    return;
}
}; /* namespace raptor_raytracer */
