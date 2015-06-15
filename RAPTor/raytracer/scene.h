#pragma once

/* Ray tracer header */
#include "camera.h"
#include "light_shader.h"
#include "phong_shader.h"
#include "cook_torrance_cxy.h"
#include "primitive_store.h"


namespace raptor_raytracer
{
/* C coded scene set up */
void scene_init(light_list &lights, primitive_store &everything, std::list<material *> &materials, camera **c);

inline void scene_clean(std::list<material *> *m, camera *c)
{
    /* Clean up the scene primitives */
    /* Clean up the materials */
    while (!m->empty())
    {
        delete m->front();
        m->pop_front();
    }
    
    /* CLean up the camera */
    delete c;
    
    return;
}
}; /* namespace raptor_raytracer */
