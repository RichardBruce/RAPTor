#include "cubic_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
/***********************************************************
  Overloaded virtual texture mapping function. Takes the 
  destination of the ray (query point) and returns the colour
  at the location and an alpha value
************************************************************/
float cubic_mapper::sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const
{
    float u_co, v_co;
    
    /* Use the interpolated texture address */
    if (vt.x != MAX_DIST)
    {
        u_co = vt.x * static_cast<float>(this->w);
        v_co = vt.y * static_cast<float>(this->h);
    }
    /* Calculate the texture address */
    else
    {
        /* Find the u,v co-ordinates of the hit */
        /* Offset */
        const point_t dist((dst - this->c) + (((this->s * this->u) + (this->s * this->v)) * 0.5f));

        /* Work out manhattan distances */
        const float dist_u = dot_product(dist, this->u) + dot_product(dist, this->n);
        const float dist_v = dot_product(dist, this->v);
        
        /* Scale */
        u_co = dist_u * (static_cast<float>(this->w) / fabs(dot_product(this->s, this->u)));
        v_co = dist_v * (static_cast<float>(this->h) / fabs(dot_product(this->s, this->v)));
    }

   
    /* Addresses and weights */
    int u0 = static_cast<int>(u_co) + this->u_off;
    int v0 = static_cast<int>(v_co) + this->v_off;

    /* Apply wrapping modes */
    assert(this->u_max == this->w);
    assert(this->v_max == this->h);
    if(!apply_wrapping_mode(&u0, (int)this->u_max, this->uw))
    {
        (*c) = ext_colour_t(0.0f, 0.0f, 0.0f);
        return 1.0f;
    }
    
    if(!apply_wrapping_mode(&v0, (int)this->v_max, this->vw))
    {
        (*c) = ext_colour_t(0.0f, 0.0f, 0.0f);
        return 1.0f;
    }

    const int u1 = std::min(u0 + 1, static_cast<int>(this->w) - 1);
    const int v1 = std::min(v0 + 1, static_cast<int>(this->h) - 1);
    assert(u0 <  static_cast<int>(this->w));
    assert(v0 <  static_cast<int>(this->h));
    assert(u1 <  static_cast<int>(this->w));
    assert(v1 <  static_cast<int>(this->h));
    assert(u0 >= 0);
    assert(v0 >= 0);
    assert(u1 >= 0);
    assert(v1 >= 0);
    
    const float fu = u_co - floor(u_co);
    const float fv = v_co - floor(v_co);
    const float w0 = (1.0f - fu) * (1.0f - fv);
    const float w1 = fu * (1.0f - fv);
    const float w2 = fv * (1.0f - fu);
    const float w3 = fu * fv;
    
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
    (*c) = (c0 * w0) + (c1 * w1) + (c2 * w2) + (c3 * w3);
    return 1.0f;
}
}; /* namespace raptor_raytracer */
