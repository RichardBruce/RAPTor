#include "cylindrical_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
/***********************************************************
  Overloaded virtual texture mapping function. Takes the 
  destination of the ray (query point) and returns the colour
  at the location and an alpha value
************************************************************/
fp_t cylindrical_mapper::texture_map(const point_t &dst, const point_t &dir, ext_colour_t *const c, const point_t &vt) const
{
    /* Find the u,v co-ordinates of the hit */
    /* Offset */
    point_t dist = dst - this->c;
//    cout << "dists: " << dist.x << ", " << dist.y << ", " << dist.z << endl;

    point_t vp  = dist / this->r;
    vp.y = 0;
    normalise(&vp);
    fp_t theta  = acos(dot_product(vp, this->u));
    fp_t u_co   = theta * (fp_t)this->w * (1.0/PI);
//    cout << "vp: " << vp.x << ", " << vp.y << ", " << vp.z << endl;
    
    
    /* Scale */
    dist += this->s * this->v * 0.5;
    fp_t v_co = (dot_product(dist, this->v)) * ((fp_t)this->h / abs(dot_product(this->s, this->v)));
//    cout << "Co-ordinates: " << u_co << ", " << v_co << endl;
//    cout << "Size: " << this->h << ", " << this->w << endl;

    assert(u_co >= 0.0);
    assert(v_co >= 0.0);
    
    /* Addresses and weights */
    unsigned u0 = ((unsigned)u_co);
    unsigned v0 = ((unsigned)v_co);
    unsigned u1 = min(((unsigned)u_co) + 1, this->w - 1);
    unsigned v1 = min(((unsigned)v_co) + 1, this->h - 1);
    assert(u0 < this->w);
    assert(v0 < this->h);
    assert(u1 < this->w);
    assert(v1 < this->h);
    
	fp_t fu = u_co - floor(u_co);
	fp_t fv = v_co - floor(v_co);
	fp_t w0 = ((fp_t)1.0 - fu) * ((fp_t)1.0 - fv);
	fp_t w1 = fu * ((fp_t)1.0 - fv);
	fp_t w2 = fv * ((fp_t)1.0 - fu);
	fp_t w3 = fu * fv;
    
    /* Texel look up */
    unsigned addr0 = (u0 + (v0 * this->w)) * this->cpp;
    ext_colour_t c0(this->img[addr0], this->img[addr0 + 1], this->img[addr0 + 2]);
    
    unsigned addr1 = (u1 + (v0 * this->w)) * this->cpp;
    ext_colour_t c1(this->img[addr1], this->img[addr1 + 1], this->img[addr1 + 2]);
    
    unsigned addr2 = (u0 + (v1 * this->w)) * this->cpp;
    ext_colour_t c2(this->img[addr2], this->img[addr2 + 1], this->img[addr2 + 2]);

    unsigned addr3 = (u1 + (v1 * this->w)) * this->cpp;
    ext_colour_t c3(this->img[addr3], this->img[addr3 + 1], this->img[addr3 + 2]);

    /* Weight and return */
    (*c) = ((c0 * w0) + (c1 * w1) + (c2 * w2) + (c3 * w3)) / (fp_t)255.0;
    return (fp_t)1.0;
}
}; /* namespace raptor_raytracer */
