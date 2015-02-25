#include "planar_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
/***********************************************************
  Overloaded virtual texture mapping function. Takes the 
  destination of the ray (query point) and returns the colour
  at the location and an alpha value
************************************************************/
float planar_mapper::sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const
{
    float u_co;
    float v_co;
    
    /* Use the interpolated texture address */
    if (vt.x != MAX_DIST)
    {
        u_co = vt.x * static_cast<float>(this->w);
        v_co = vt.y * static_cast<float>(this->h);
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
        dist += ((this->s * flip_u) + (this->s * this->v)) * 0.5f;
        
        /* Scale */
        u_co = (dot_product(dist, flip_u)) * (static_cast<float>(this->w) / fabs(dot_product(this->s, flip_u)));
        v_co = (dot_product(dist, this->v)) * (static_cast<float>(this->h) / fabs(dot_product(this->s, this->v)));
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
    int u0 = static_cast<int>(u_co) + this->u_off;
    int v0 = static_cast<int>(v_co) + this->v_off;

    /* Apply wrapping modes */
    assert(this->u_max == this->w);
    assert(this->v_max == this->h);
    if(!apply_wrapping_mode(&u0, (int)this->u_max, this->uw))
    {
        (*c) = ext_colour_t(0.0f, 0.0f, 0.0f);
        return 1.0;
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

float planar_mapper::sample_texture_monochrome(point_t *const p, const point_t &dst, const point_t &n, const point_t &vt, const int x_off, const int y_off) const
{
    // assert(cpp == 1);
    // float u_co, v_co;
    
    // /* Use the interpolated texture address */
    // const float pixel_width_u = fabs(dot_product(this->s, this->u)) / static_cast<float>(this->w);
    // const float pixel_width_v = fabs(dot_product(this->s, this->v)) / static_cast<float>(this->h);
    // if (vt.x != MAX_DIST)
    // {
    //     u_co = vt.x * static_cast<float>(this->w);
    //     v_co = vt.y * static_cast<float>(this->h);
    // }
    // /* Calculate the texture address */
    // else
    // {
    //     /* Find the u,v co-ordinates of the hit */
    //     /* Offset */
    //     point_t dist = dst - this->c;
    //     dist += ((this->s * this->u) + (this->s * this->v)) * 0.5f;
        
    //     /* Scale */
    //     u_co = (dot_product(dist, this->u)) / pixel_width_u;
    //     v_co = (dot_product(dist, this->v)) / pixel_width_v;
    // }

    // /* Addresses and weights */
    // int u0 = static_cast<int>(u_co) + this->u_off + x_off;
    // int v0 = static_cast<int>(v_co) + this->v_off + y_off;

    // /* Apply wrapping modes */
    // assert(this->u_max == this->w);
    // assert(this->v_max == this->h);
    // const float x_pos = (u_co + x_off) * pixel_width_u;
    // const float y_pos = (v_co + y_off) * pixel_width_v;
    // if(!apply_wrapping_mode(&u0, static_cast<int>(this->u_max), this->uw))
    // {
    //     (*p) = point_t(x_pos, y_pos, 0.0f);
    //     return 1.0f;
    // }
    
    // if(!apply_wrapping_mode(&v0, static_cast<int>(this->v_max), this->vw))
    // {
    //     (*p) = point_t(x_pos, y_pos, 0.0f);
    //     return 1.0f;
    // }

    // const int u1 = min(u0 + 1, static_cast<int>(this->w) - 1);
    // const int v1 = min(v0 + 1, static_cast<int>(this->h) - 1);
    // assert(u0 <  static_cast<int>(this->w));
    // assert(v0 <  static_cast<int>(this->h));
    // assert(u1 <  static_cast<int>(this->w));
    // assert(v1 <  static_cast<int>(this->h));
    // assert(u0 >= 0);
    // assert(v0 >= 0);
    // assert(u1 >= 0);
    // assert(v1 >= 0);
    
    // const float fu = u_co - floor(u_co);
    // const float fv = v_co - floor(v_co);
    // const float w0 = (1.0f - fu) * (1.0f - fv);
    // const float w1 = fu * (1.0f - fv);
    // const float w2 = fv * (1.0f - fu);
    // const float w3 = fu * fv;
    
    // /* Texel look up */
    // unsigned addr0 = (u0 + (v0 * this->w));
    // const float c0 = this->img[addr0];
    
    // unsigned addr1 = (u1 + (v0 * this->w));
    // const float c1 = this->img[addr1];
    
    // unsigned addr2 = (u0 + (v1 * this->w));
    // const float c2 = this->img[addr2];

    // unsigned addr3 = (u1 + (v1 * this->w));
    // const float c3 = this->img[addr3];

    // /* Weight and return */
    // const float c = (c0 * w0) + (c1 * w1) + (c2 * w2) + (c3 * w3);
    // (*p) = point_t(x_pos, y_pos, c);
    return 1.0f;
}
}; /* namespace raptor_raytracer */
