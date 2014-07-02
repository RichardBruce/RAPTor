#ifndef __FORCE_INFO_H__
#define __FORCE_INFO_H__

#include "common.h"

/* Struct to pack information needed to apply a force */
struct force_info
{
    /* CTOR, make defensive copies here */
    force_info(const point_t &f, const point_t &at, const fp_t t)
        : f(f), at(at), t(t) {  };
        
    point_t f;  /* The size and direction of the force to apply                         */
    point_t at; /* The point the force is applied at with the object com as the origin  */
    fp_t    t;  /* The length of time to apply the force for                            */
};
        
#endif /* #ifndef __FORCE_INFO_H__ */

