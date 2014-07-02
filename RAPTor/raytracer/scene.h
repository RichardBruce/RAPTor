#ifndef __SCENE_INIT_H__
#define __SCENE_INIT_H__

/* Primitives */
#include "triangle.h"

/* Camera */
#include "camera.h"

/* Shaders */
#include "light_shader.h"
#include "phong_shader.h"
#include "cook_torrance_cxy.h"

/* Texture mappers */
#include "texture_mapper.h"
#include "planar_mapper.h"
#include "mandelbrot_shader.h"


/* C coded scene set up */
void scene_init(light_list &lights, primitive_list &everything, list<material *> &materials, camera **c);

inline void scene_clean(primitive_list *e, list<material *> *m, camera *c)
{
    /* Clean up the scene primitives */
    /* This is faster for vectors */
    for(primitive_list::iterator i = e->begin(); i < e->end(); ++i)
    {
        delete (*i);
    }
    
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

#endif /* #ifndef __SCENE_INIT_H__ */
