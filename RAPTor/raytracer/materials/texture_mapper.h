#ifndef __TEXTURE_MAPPER_H__
#define __TEXTURE_MAPPER_H__

/* Common headers */
#include "common.h"

/* C style headers */
extern "C"
{
#include <jpeglib.h>
#include <stdio.h>
}

/* Forward declarations */
class ext_colour_t;
class point_t;


enum texture_wrapping_mode_t { blank = 0, clamp = 1, tile = 2, mirror = 3 };

/* Pure virtual class for texture mappers data and mapping */
class texture_mapper
{
    public :
        texture_mapper() { };
        virtual ~texture_mapper() { };

        /* Pure virtual texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        virtual fp_t texture_map(const point_t &dst, const point_t &dir, ext_colour_t *const c, const point_t &vt) const = 0;

    private :
};


inline unsigned read_jpeg(fp_t **img, const char *const filename, unsigned *const h, unsigned *const w)
{
    /* Open the input file */
    FILE *infile = fopen(filename, "rb");
    if (!infile)
    {
    	cout << "Error opening jpeg file " << filename << " for input"  << endl;
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
    *img = new fp_t[cinfo.output_width * cinfo.output_height * cinfo.num_components];
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

#endif /* #ifndef __TEXTURE_MAPPER_H__ */
