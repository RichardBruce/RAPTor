#include "scene.h"
#include "parser_common.h"
#include "polygon_to_triangles.h"


namespace raptor_raytracer
{
/* Auto generated init function do not edit directly */
void scene_init(light_list &lights, primitive_store &everything, std::list<material *> &materials, camera **c)
{
    /* Set up the camera */
    *c = new camera(point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), ext_colour_t(0.0f ,0.0f ,255.0f), 7.5f, 10.0f, 10.0f, 640, 480, 1, 1, 0.0f, 0.0f);

    phong_shader *frame_mat = new phong_shader(ext_colour_t(125.0f, 125.0f, 125.0f), 0.3330f, 0.3330f, 0.0300f);
    materials.push_back(frame_mat);

    everything.emplace_back(frame_mat, point_t(-10.0f, -10.0f, 100.0f), point_t(-10.0f, 10.0f, 100.0f), point_t(10.0f, 10.0f, 100.0f));
    everything.emplace_back(frame_mat, point_t(10.0f, 10.0f, 100.0f), point_t(10.0f, -10.0f, 100.0f), point_t(-10.0f, -10.0f, 100.0f));

    new_light(&lights, ext_colour_t(255.0f ,255.0f ,255.0f), point_t(2.5f, -2.5f, 50.0f), 0.0f, 0.0f);

    /* Assert there must be some lights to light the scene */
    assert(!lights.empty());

    /* Assert there is an imagine to trace */
    assert(!everything.empty());
    
    return;
}
}; /* namespace raptor_raytracer */
