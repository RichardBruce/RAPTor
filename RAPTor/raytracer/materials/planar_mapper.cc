#include "planar_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
/***********************************************************
  Overloaded virtual texture mapping function. Takes the 
  destination of the ray (query point) and returns the colour
  at the location and an alpha value
************************************************************/
fp_t planar_mapper::texture_map(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const
{
    fp_t u_co, v_co;
    
    /* Use the interpolated texture address */
    if (vt.x != MAX_DIST)
    {
        u_co = vt.x * (fp_t)this->w;
        v_co = vt.y * (fp_t)this->h;
    }
    /* Calculate the texture address */
    else
    {
        /* Check the ray came from outside the texture */
        /* If it didnt the texture is flipped horizontally */
        point_t flip_u;
    //    if (dot_product(dst, this->n) > 0.0)
    //    {
    ////        cout << "flipping" << endl;
    //        flip_u = -this->u;
    //    }
    //    else
    //    {
            flip_u =  this->u;
    //    }
        
        /* Find the u,v co-ordinates of the hit */
        /* Offset */
        point_t dist = dst - this->c;
    //    cout << "dists: " << dist.x << ", " << dist.y << ", " << dist.z << endl;
        dist += ((this->s * flip_u) + (this->s * this->v)) * 0.5;
        
        /* Scale */
        u_co = (dot_product(dist, flip_u)) * ((fp_t)this->w / fabs(dot_product(this->s, flip_u)));
        v_co = (dot_product(dist, this->v)) * ((fp_t)this->h / fabs(dot_product(this->s, this->v)));
    }

//    if (!(v_co >= 0.0) || !(u_co >= 0.0))
//    {
//        cout << "wrap : " << this->uw  << ", " << this->vw << endl;
//        cout << "u vec: " << flip_u.x << ", " << flip_u.y << ", " << flip_u.z << endl;
//        cout << "v vec: " << this->v.x << ", " << this->v.y << ", " << this->v.z << endl;
//        cout << "s vec: " << this->s.x << ", " << this->s.y << ", " << this->s.z << endl;
//        cout << "dists: " << dist.x << ", " << dist.y << ", " << dist.z << endl;
//        cout << "co-ords: " << u_co << ", " << v_co << endl;
//    }
    
//    cout << u_co << endl;
//    assert(u_co >= 0.0);
//    assert(v_co >= 0.0);
    
    /* Addresses and weights */
    int u0 = (int)u_co + this->u_off;
    int v0 = (int)v_co + this->v_off;

    /* Apply wrapping modes */
    assert(this->u_max == this->w);
    assert(this->v_max == this->h);
    if(!apply_wrapping_mode(&u0, (int)this->u_max, this->uw))
    {
        (*c) = ext_colour_t(0.0,0.0,0.0);
        return 1.0;
    }
    
    if(!apply_wrapping_mode(&v0, (int)this->v_max, this->vw))
    {
        (*c) = ext_colour_t(0.0,0.0,0.0);
        return 1.0;
    }

    const int u1 = min(u0 + 1, (int)this->w - 1);
    const int v1 = min(v0 + 1, (int)this->h - 1);
    assert(u0 <  (int)this->w);
    assert(v0 <  (int)this->h);
    assert(u1 <  (int)this->w);
    assert(v1 <  (int)this->h);
    assert(u0 >= 0);
    assert(v0 >= 0);
    assert(u1 >= 0);
    assert(v1 >= 0);
    
    const fp_t fu = u_co - floor(u_co);
    const fp_t fv = v_co - floor(v_co);
    const fp_t w0 = ((fp_t)1.0 - fu) * ((fp_t)1.0 - fv);
    const fp_t w1 = fu * ((fp_t)1.0 - fv);
    const fp_t w2 = fv * ((fp_t)1.0 - fu);
    const fp_t w3 = fu * fv;
    
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
    return (fp_t)1.0;
}
}; /* namespace raptor_raytracer */
