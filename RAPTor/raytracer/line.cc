/***********************************************************
 Implementation of the shape virtual functions for a sphere
************************************************************/

#include "line.h"

void line::rotate(const vector_t &r, const point_t &c, const fp_t theta)
{
    /* Rotate the origin into the new coordinate system */
    point_t p = this->ogn;
    point_t q(0.0,0.0,0.0);
   
    fp_t costheta = cos(theta);
    fp_t sintheta = sin(theta);
    
    /* Transform r into the origin */
    p.x -= c.x;
    p.y -= c.y;
    p.z -= c.z;

    /* Rotate about r */
    q.x += (costheta + (1 - costheta) * r.x * r.x) * p.x;
    q.x += ((1 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
    q.x += ((1 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

    q.y += ((1 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
    q.y += (costheta + (1 - costheta) * r.y * r.y) * p.y;
    q.y += ((1 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

    q.z += ((1 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
    q.z += ((1 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
    q.z += (costheta + (1 - costheta) * r.z * r.z) * p.z;
    
    
    /* Move the point up a unit distance */
    point_t q2(0.0,0.0,0.0);
    p += this->dir;
    
    /* Rotate about r */
    q2.x += (costheta + (1 - costheta) * r.x * r.x) * p.x;
    q2.x += ((1 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
    q2.x += ((1 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

    q2.y += ((1 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
    q2.y += (costheta + (1 - costheta) * r.y * r.y) * p.y;
    q2.y += ((1 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

    q2.z += ((1 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
    q2.z += ((1 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
    q2.z += (costheta + (1 - costheta) * r.z * r.z) * p.z;
    
    /* Convert back to gradiant */
    q2.x -= q.x;
    q2.y -= q.y;
    q2.z -= q.z;
    
    /* Move points back to where they came from */
    q.x  += c.x;
    q.y  += c.y;
    q.z  += c.z;
    
    this->ogn = q;
    this->dir = q2;
}

