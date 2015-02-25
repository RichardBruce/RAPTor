#ifndef __TEXTURE_MAPPER_H__
#define __TEXTURE_MAPPER_H__

/* Boost headers */
#include "boost/scoped_ptr.hpp"

/* Common headers */
#include "common.h"

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
enum texture_wrapping_mode_t { blank = 0, clamp = 1, tile = 2, mirror = 3 };

/* Pure virtual class for texture mappers data and mapping */
class texture_mapper
{
    public :
        texture_mapper() : _falloff(nullptr) { };

        virtual ~texture_mapper() {  };

        /* Texture mapping functions. Calls the derived class to sample the texture and the applies fall off */
        virtual float texture_map(const ray &r, ext_colour_t *const c, const point_t &n, const point_t &vt) const
        {
            /* Sample image */
            const float opac = sample_texture(c, r.get_dst(), n, vt);

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

        virtual float texture_map_monochrome(const ray &r, point_t *const p, const point_t &n, const point_t &vt, const int x_off = 0, const int y_off = 0) const
        {
            /* Sample image */
            const float opac = sample_texture_monochrome(p, r.get_dst(), n, vt, x_off, y_off);

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
        virtual float sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const = 0;
        virtual float sample_texture_monochrome(point_t *const c, const point_t &dst, const point_t &n, const point_t &vt, const int x_off, const int y_off) const = 0;

    private :
        boost::scoped_ptr<const mapper_falloff> _falloff;
        bool                                    _inverse;
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
        case blank :
            if (((*c) < 0.0) || ((*c) >= s))
            {
                return false;
            }
            break;
        /* Clamp to the edge colour */
        case clamp :
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
        case tile :
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
        case mirror :
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
inline point_t bump_map(const point_t &n, const point_t &x_0, const point_t &x_1, const point_t &y_0, const point_t &y_1, const float dist)
{
    /* Calculate texture normal */
    const point_t x_d(normalise((x_1 - x_0) / dist));
    const point_t y_d(normalise((y_1 - y_0) / dist));
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
