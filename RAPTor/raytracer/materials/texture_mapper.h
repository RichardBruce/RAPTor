#ifndef __TEXTURE_MAPPER_H__
#define __TEXTURE_MAPPER_H__

/* Boost headers */
#include "boost/scoped_ptr.hpp"

/* Common headers */
#include "common.h"
#include "logging.h"

/* Raytracer headers */
#include "ray.h"
#include "ext_colour_t.h"
#include "mapper_falloff.h"

/* C style headers */
extern "C"
{
#include <jpeglib.h>
#include <stdio.h>
}

/* Forward declarations */
class point_t;

namespace raptor_raytracer
{
enum class texture_wrapping_mode_t : char { blank = 0, clamp = 1, tile = 2, mirror = 3 };

bool apply_wrapping_mode(int *const c, const int s, const texture_wrapping_mode_t m);

/* Pure virtual class for texture mappers data and mapping */
class texture_mapper
{
    public :
        texture_mapper() : _falloff(nullptr), _inverse(false) { };

        virtual ~texture_mapper() {  };

        /* Texture mapping functions. Calls the derived class to sample the texture and the applies fall off */
        float texture_map(const ray &r, ext_colour_t *const c, const point_t &n, const point_t &vt) const
        {
            /* Sample image */
            const float opac = sample_texture(r, c, n, vt);

            /* Apply falloff for this layer */
            if (_falloff != nullptr)
            {
                _falloff->falloff(c, r.get_dst());
            }

            /* Invert layer */
            if (_inverse)
            {
                (*c) = ext_colour_t(255.0f, 255.0f, 255.0f) - (*c);
            }

            return opac;
        }

        float texture_map_monochrome(const ray &r, point_t *const p, const point_t &n, const point_t &vt, const int x_off = 0, const int y_off = 0) const
        {
            /* Sample image */
            const float opac = sample_texture_monochrome(r, p, n, vt, x_off, y_off);

            /* Apply falloff for this layer */
            if (_falloff != nullptr)
            {
                ext_colour_t c(p->z, p->z, p->z);
                _falloff->falloff(&c, r.get_dst());
                p->z = c.r;
            }

            /* Invert layer */
            if (_inverse)
            {
                p->z = 255.0f - p->z;
            }

            return opac;
        }

        void falloff(const mapper_falloff *falloff)
        {
            _falloff.reset(falloff);
        }

        void inverse(const bool inverse)
        {
            _inverse = inverse;
        }

        const mapper_falloff *  falloff() const { return _falloff.get();    }
        bool                    inverse() const { return _inverse;          }

    protected :
        /* Pure virtual texture mapping function. Takes the destination of the incident ray and normal of the hit surface.
           returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        virtual float sample_texture(const ray &r, ext_colour_t *const c, const point_t &n, const point_t &vt) const = 0;
        virtual float sample_texture_monochrome(const ray &r, point_t *const p, const point_t &n, const point_t &vt, const int x_off = 0, const int y_off = 0) const = 0;

    private :
        boost::scoped_ptr<const mapper_falloff> _falloff;
        bool                                    _inverse;
};

class image_texture_mapper : public texture_mapper
{
    public :
        image_texture_mapper(const boost::shared_array<float> &img, const point_t &c, const point_t &s, const point_t &u, const point_t &v, const float u_ps, const float v_ps,
            const unsigned int w, const unsigned int h, const unsigned int cpp, const texture_wrapping_mode_t uw, const texture_wrapping_mode_t vw,
            const int u_off = 0, const int v_off = 0, const int u_max = -1, const int v_max = -1) : 
                texture_mapper(), _img(img), _c(c), _s(s), _u(u), _v(v), _u_ps(fabs(dot_product(_s, _u)) * u_ps), _v_ps(fabs(dot_product(_s, _v)) * v_ps), _h(h), _w(w), _cpp(cpp), _u_max(u_max < 0 ? w : u_max), _v_max(v_max < 0 ? h : v_max),
                _u_off(u_off), _v_off(v_off), _uw(uw), _vw(vw)
            { 
                assert((_cpp == 1) || (_cpp == 3));
            }

        virtual ~image_texture_mapper() { };

    protected :
        /* Virtual function for different image mappers to generate co-ordinates in their own way */
        virtual void texture_coordinates(float *const u_co, float *const v_co, const point_t &dst, const point_t &n) const = 0;

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float sample_texture(const ray &r, ext_colour_t *const c, const point_t &n, const point_t &vt) const override
        {
            /* Use the interpolated texture address */
            float u_co;
            float v_co;
            if (vt.x != MAX_DIST)
            {
                u_co = vt.x * static_cast<float>(_w);
                v_co = vt.y * static_cast<float>(_h);
            }
            else
            {
                texture_coordinates(&u_co, &v_co, r.get_dst(), n);
            }

            /* Addresses and weights */
            int u0 = static_cast<int>(u_co) + _u_off;
            int v0 = static_cast<int>(v_co) + _v_off;

            /* Apply wrapping modes */
            assert(_u_max == _w);
            assert(_v_max == _h);
            if (!apply_wrapping_mode(&u0, static_cast<int>(_u_max), _uw))
            {
                (*c) = ext_colour_t(0.0f, 0.0f, 0.0f);
                return 1.0;
            }
            
            if (!apply_wrapping_mode(&v0, static_cast<int>(_v_max), _vw))
            {
                (*c) = ext_colour_t(0.0f, 0.0f, 0.0f);
                return 1.0f;
            }

            /* Texel lookup */
            texel_lookup(c, u_co, v_co, u0, v0);

            return 1.0f;
        }

        float sample_texture_monochrome(const ray &r, point_t *const p, const point_t &n, const point_t &vt, const int x_off, const int y_off) const override
        {
            /* Use the interpolated texture address */
            float u_co;
            float v_co;
            if (vt.x != MAX_DIST)
            {
                u_co = vt.x * static_cast<float>(_w);
                v_co = vt.y * static_cast<float>(_h);
            }
            else
            {
                texture_coordinates(&u_co, &v_co, r.get_dst(), n);
            }

            /* Offset */
            u_co += x_off;
            v_co += y_off;

            /* Addresses and weights */
            int u0 = static_cast<int>(u_co) + _u_off;
            int v0 = static_cast<int>(v_co) + _v_off;

            /* Apply wrapping modes */
            assert(_u_max == _w);
            assert(_v_max == _h);
            if (!apply_wrapping_mode(&u0, static_cast<int>(_u_max), _uw))
            {
                *p = point_t(0.0f, x_off, y_off);
                return 1.0;
            }
            
            if (!apply_wrapping_mode(&v0, static_cast<int>(_v_max), _vw))
            {
                *p = point_t(0.0f, x_off, y_off);
                return 1.0f;
            }

            /* Texel lookup */
            ext_colour_t c;
            texel_lookup(&c, u_co, v_co, u0, v0);

            /* Average down from rgb image */
            p->x = x_off * _u_ps;
            p->y = y_off * _v_ps;
            if (_cpp == 3)
            {
                p->z = (c.r + c.g + c.b) * (1.0f / 3.0f);
            }
            else
            {
                p->z = c.r;
            }

            return 1.0f;
        }

        boost::shared_array<float>      _img;   /* Image data                           */
        const point_t                   _c;     /* Center of the texture                */
        const point_t                   _s;     /* Size of the texture                  */
        point_t                         _u;     /* U vector in the plane of the texture */
        point_t                         _v;     /* V vector in the plane of the texture */
        const float                     _u_ps;  /* The size of 1 pixl in u              */
        const float                     _v_ps;  /* The size of 1 pixl in v              */
        unsigned int                    _h;     /* Image height                         */
        unsigned int                    _w;     /* Image width                          */
        unsigned int                    _cpp;   /* Componants per pixel                 */
        unsigned int                    _u_max; /* Max u value for early wrapping       */
        unsigned int                    _v_max; /* Max v value for early wrapping       */
        const int                       _u_off; /* U offset to be added to every pixel  */
        const int                       _v_off; /* V offset to be added to every pixel  */
        const texture_wrapping_mode_t   _uw;    /* U wrapping mode                      */
        const texture_wrapping_mode_t   _vw;    /* V wrapping mode                      */

    private :
        void texel_lookup(ext_colour_t *const c, const float u_co, const float v_co, const int u0, const int v0) const
        {
            /* Calculate upper coordinate */
            const int u1 = std::min(u0 + 1, static_cast<int>(_w) - 1);
            const int v1 = std::min(v0 + 1, static_cast<int>(_h) - 1);
            assert(u0 <  static_cast<int>(_w));
            assert(v0 <  static_cast<int>(_h));
            assert(u0 >= 0);
            assert(v0 >= 0);
            assert(u1 >= 0);
            assert(v1 >= 0);

            /* Calculate weights */
            const float fu = u_co - floor(u_co);
            const float fv = v_co - floor(v_co);
            const float w0 = (1.0f - fu) * (1.0f - fv);
            const float w1 = fu * (1.0f - fv);
            const float w2 = fv * (1.0f - fu);
            const float w3 = fu * fv;
            
            /* Weight and return */
            if (_cpp == 3)
            {
                const unsigned addr0 = (u0 + (v0 * _w)) * 3;
                const ext_colour_t c0(_img[addr0], _img[addr0 + 1], _img[addr0 + 2]);
                
                const unsigned addr1 = (u1 + (v0 * _w)) * 3;
                const ext_colour_t c1(_img[addr1], _img[addr1 + 1], _img[addr1 + 2]);
                
                const unsigned addr2 = (u0 + (v1 * _w)) * 3;
                const ext_colour_t c2(_img[addr2], _img[addr2 + 1], _img[addr2 + 2]);

                const unsigned addr3 = (u1 + (v1 * _w)) * 3;
                const ext_colour_t c3(_img[addr3], _img[addr3 + 1], _img[addr3 + 2]);

                (*c) = (c0 * w0) + (c1 * w1) + (c2 * w2) + (c3 * w3);
            }
            else if (_cpp == 1)
            {
                const unsigned addr0 = (u0 + (v0 * _w));
                const unsigned addr1 = (u1 + (v0 * _w));
                const unsigned addr2 = (u0 + (v1 * _w));
                const unsigned addr3 = (u1 + (v1 * _w));
                c->r = (_img[addr0] * w0) + (_img[addr1] * w1) + (_img[addr2] * w2) + (_img[addr3] * w3);
                c->g = c->r;
                c->b = c->r;
            }
        }
};

class procedural_texture_mapper : public texture_mapper
{
    public :
        procedural_texture_mapper(const float u_ps, const float v_ps) : 
            texture_mapper(), _u_ps(u_ps), _v_ps(v_ps) {  }

        virtual ~procedural_texture_mapper() { };

    protected :
        /* Virtual function for different procedural mappers to generate colours in their own way */
        virtual float run_procedure(ext_colour_t *const c, const point_t &dst, const point_t &dir, const point_t &n) const = 0;

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float sample_texture(const ray &r, ext_colour_t *const c, const point_t &n, const point_t &vt) const override
        {
            /* Run procedure */
            const float alpha = run_procedure(c, r.get_dst(), r.get_dir(), n);
            return alpha;
        }

        float sample_texture_monochrome(const ray &r, point_t *const p, const point_t &n, const point_t &vt, const int x_off, const int y_off) const override
        {
            /* Calculate offset distance */
            const float x_off_dist = x_off * _u_ps;
            const float y_off_dist = y_off * _v_ps;

            const point_t u(n.y + n.z, 0.0f, n.x);
            const point_t v(0.0f, -n.x - n.z, -n.y);
            const point_t dst(r.get_dst() + (u * x_off_dist) + (v * y_off_dist));

            /* Run procedure */
            ext_colour_t c;
            const float alpha = run_procedure(&c, dst, r.get_dir(), n);

            /* Average down from rgb image */
            p->x = x_off_dist;
            p->y = y_off_dist;
            p->z = (c.r + c.g + c.b) * (1.0f / 3.0f);

            return alpha;
        }

        const float _u_ps;  /* The size of 1 pixl in u  */
        const float _v_ps;  /* The size of 1 pixl in v  */
};


inline unsigned read_jpeg(float **img, const char *const filename, unsigned *const h, unsigned *const w)
{
    /* Open the input file */
    FILE *infile = fopen(filename, "rb");
    if (!infile)
    {
    	std::cout << "Error opening jpeg file " << filename << " for input"  << std::endl;
    	assert(!"Cannot open file");
    }

    /* Set up the jpeg decompressor */
    struct jpeg_decompress_struct   cinfo;
    struct jpeg_error_mgr           jerr;

    /* Get header information */
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, infile);
    jpeg_read_header(&cinfo, TRUE);

    /* Decompression the jpeg */
    jpeg_start_decompress(&cinfo);

    /* Allocate memory to hold the uncompressed image */
    *img = new float[cinfo.output_width * cinfo.output_height * cinfo.num_components];
    *h   = cinfo.output_height;
    *w   = cinfo.output_width;

    /* now actually read the jpeg into the raw buffer */
    JSAMPROW row_pointer[1];
    row_pointer[0] = (unsigned char *) malloc(cinfo.output_width * cinfo.num_components);

    /* read one scan line at a time */
    unsigned long location = 0;
    while(cinfo.output_scanline < cinfo.image_height)
    {
    	jpeg_read_scanlines(&cinfo, row_pointer, 1);
    	for(unsigned i = 0; i < (cinfo.image_width * cinfo.num_components); i++)
        {
    		(*img)[location++] = row_pointer[0][i];
        }
    }

    /* Clean up */
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    free(row_pointer[0]);
    fclose(infile);
    
    return cinfo.num_components;
}

inline bool apply_wrapping_mode(int *const c, const int s, const texture_wrapping_mode_t m)
{
    switch (m)
    {
        /* Set black outside the texture */
        case texture_wrapping_mode_t::blank :
            if (((*c) < 0.0) || ((*c) >= s))
            {
                return false;
            }
            break;
        /* Clamp to the edge colour */
        case texture_wrapping_mode_t::clamp :
            if ((*c) < 0)
            {
                (*c) = 0;
            }
            else if ((*c) >= s)
            {
                (*c) = s - 1;
            }
            break;
        /* Tile the texture */
        case texture_wrapping_mode_t::tile :
            if ((*c) < 0)
            {
                assert(s > 0);
                (*c) = (*c) % (s - 1);

                /* NOTE -- -ve % +ve is still -ve */
                (*c) += s - 1;
            }
            else if ((*c) >= s)
            {
                (*c) = (*c) % s;
            }
            break;
        /* Mirror the texture */
        case texture_wrapping_mode_t::mirror :
            if ((*c) < 0)
            {
                (*c) = (*c) % ((s - 1) << 1);
                /* NOTE -- -ve % +ve is still -ve */
                (*c) += s - 1;
                (*c)  = abs((*c));
            }
            else if ((*c) >= s)
            {
                (*c)  = (*c) % ((s - 1) << 1);
                (*c) -= s - 1;
                (*c)  = abs((*c));
            }
            break;

        default :
            assert(!"Unknown wrapping mode");
            break;
    }
    
    return true;
}

/* Bump map */
/* Takes the object normal, n, and texture heights, x_0, x_1, y_0 and y_1 and the distance between the texture heights */
/* The texture heights are used to calculate the gradients of the texture in x and y and from that the normal of the texture */
/* The texture normal is then rotated into the objects co-ordinates using the object normal */
inline point_t bump_map(const point_t &n, const point_t &x_0, const point_t &x_1, const point_t &y_0, const point_t &y_1)
{
    /* Calculate texture normal */
    const point_t x_d(normalise(x_1 - x_0));
    const point_t y_d(normalise(y_1 - y_0));
    const point_t text_n(cross_product(x_d, y_d));

    /* Work out rotation */
    const point_t t(n.z, n.x, n.y);
    const point_t b(cross_product(n, t));

    const point_t t_2(t - (n * dot_product(n, t)));
    const point_t b_2(b - (n * dot_product(n, b)) - (t_2 * dot_product(t_2, b)));

    /* Rotate texture normal into objects space */
    const float norm_x = (text_n.x * t_2.x) + (text_n.y * b_2.x) + (text_n.z * n.x);
    const float norm_y = (text_n.x * t_2.y) + (text_n.y * b_2.y) + (text_n.z * n.y);
    const float norm_z = (text_n.x * t_2.z) + (text_n.y * b_2.z) + (text_n.z * n.z);
    return point_t(norm_x, norm_y, norm_z);
}
}; /* namespace raptor_raytracer */
#endif /* #ifndef __TEXTURE_MAPPER_H__ */
