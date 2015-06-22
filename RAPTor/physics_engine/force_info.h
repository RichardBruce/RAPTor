#pragma once

#include "common.h"

/* Struct to pack information needed to apply a force */
struct force_info
{
    /* CTOR, make defensive copies here */
    force_info(const point_t &f, const point_t &at, const float t)
        : f(f), at(at), t(t) {  };
        
    point_t f;  /* The size and direction of the force to apply                         */
    point_t at; /* The point the force is applied at with the object com as the origin  */
    float   t;  /* The length of time to apply the force for                            */
};
