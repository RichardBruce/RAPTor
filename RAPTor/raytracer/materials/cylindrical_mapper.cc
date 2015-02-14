#include "cylindrical_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
/***********************************************************
  Overloaded virtual texture mapping function. Takes the 
  destination of the ray (query point) and returns the colour
  at the location and an alpha value
************************************************************/
float cylindrical_mapper::texture_map(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const
{
    /* Find the u,v co-ordinates of the hit */
    /* Offset */
    point_t dist = dst - this->c;
    // BOOST_LOG_TRIVIAL(trace) << "dists: " << dist;

    point_t vp  = dist / this->r;
    vp.y = 0;
    normalise(&vp);
    const float theta  = acos(dot_product(vp, this->u));
    const float u_co   = theta * static_cast<float>(this->w) * (1.0f / PI);
    // BOOST_LOG_TRIVIAL(trace) << "vp: " << vp;
    
    
    /* Scale */
    dist += this->s * this->v * 0.5f;
    const float v_co = (dot_product(dist, this->v)) * (static_cast<float>(this->h) / abs(dot_product(this->s, this->v)));
    // BOOST_LOG_TRIVIAL(trace) << "Co-ordinates: (" << u_co << ", " << v_co << "), texture size: (" << this->w << ", " << this->h << ")";

    assert(u_co >= 0.0f);
    assert(v_co >= 0.0f);
    
    /* Addresses and weights */
    const unsigned int u0 = std::min(static_cast<unsigned int>(u_co), this->w - 1);
    const unsigned int v0 = std::min(static_cast<unsigned int>(v_co), this->h - 1);
    const unsigned int u1 = std::min(static_cast<unsigned int>(u_co) + 1, this->w - 1);
    const unsigned int v1 = std::min(static_cast<unsigned int>(v_co) + 1, this->h - 1);
    assert(u0 < this->w);
    assert(v0 < this->h);
    assert(u1 < this->w);
    assert(v1 < this->h);
    
	const float fu = u_co - floor(u_co);
	const float fv = v_co - floor(v_co);
	const float w0 = (1.0f - fu) * (1.0f - fv);
	const float w1 = fu * (1.0f - fv);
	const float w2 = fv * (1.0f - fu);
	const float w3 = fu * fv;
    
    /* Texel look up */
    const unsigned int addr0 = (u0 + (v0 * this->w)) * this->cpp;
    ext_colour_t c0(this->img[addr0], this->img[addr0 + 1], this->img[addr0 + 2]);
    
    const unsigned int addr1 = (u1 + (v0 * this->w)) * this->cpp;
    ext_colour_t c1(this->img[addr1], this->img[addr1 + 1], this->img[addr1 + 2]);
    
    const unsigned int addr2 = (u0 + (v1 * this->w)) * this->cpp;
    ext_colour_t c2(this->img[addr2], this->img[addr2 + 1], this->img[addr2 + 2]);

    const unsigned int addr3 = (u1 + (v1 * this->w)) * this->cpp;
    ext_colour_t c3(this->img[addr3], this->img[addr3 + 1], this->img[addr3 + 2]);

    /* Weight and return */
    (*c) = ((c0 * w0) + (c1 * w1) + (c2 * w2) + (c3 * w3)) / 255.0f;
    return 1.0f;
}
}; /* namespace raptor_raytracer */
