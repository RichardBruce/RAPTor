/* Standard headers */
#include <cstdint>

/* Common headers */
#include "common.h"
#include "logging.h"

/* Ray tracer headers */
#include "parser_common.h"
#include "lwo_parser.h"
#include "camera.h"


namespace raptor_raytracer
{
enum mapper_type_t { non = 0, f_noise = 1, planar = 2, cubic = 3, spherical = 4, cylindrical = 5 };
enum mapper_of_t   { map_btex = 0, map_ctex = 1, map_dtex = 2, map_ltex = 3, map_stex = 4, map_rtex = 5, map_ttex = 6 };

struct texture_info_t
{
    void reset(const mapper_of_t m, const mapper_type_t s)
    {
        trgb            = ext_colour_t(255.0, 255.0, 255.0);
        topc            = 1.0;
        shader          = s;
        map_of          = m;
        twrp_mode_x     = (texture_wrapping_mode_t)2;
        twrp_mode_y     = (texture_wrapping_mode_t)2;    
    }
    
    void add_shader_to(list<texture_mapper *>  *const t)
    {
        texture_mapper  *tm;
        fp_t *img;
        std::uint32_t img_width;
        std::uint32_t img_height;
        std::uint32_t cpp;
        switch (this->shader)
        {
            case f_noise :
                tm = new perlin_noise_3d_mapper(this->trgb, this->tfp[1], this->tfp[0], this->tip, 4);
                t->push_front(tm);
                break;
                
            case cylindrical  :
                tm = new cylindrical_mapper(this->filename.c_str(), this->tctr, this->tnorm, this->tsiz, this->tfp[0]);
                t->push_front(tm);
                break;
    
            case planar  :
                cpp = read_jpeg(&img, this->filename.c_str(), &img_height, &img_width);
                tm  = new planar_mapper(img, this->tctr, this->tnorm, this->tsiz, cpp, img_width, img_height, this->twrp_mode_x, this->twrp_mode_y);
                t->push_front(tm);
                break;
    
            case cubic  :
                tm  = new cubic_mapper(this->filename.c_str(), this->tctr, this->tnorm, this->tsiz, this->twrp_mode_x, this->twrp_mode_y);
                t->push_front(tm);
                break;
    
            default :
                assert(false);
                break;
        }
    }
    
    void add_shader()
    {
        switch (this->map_of)
        {
            case map_btex :
                this->add_shader_to(&this->btex);
                break;

            case map_ctex :
                this->add_shader_to(&this->ctex);
                break;
                
            case map_dtex :
                this->add_shader_to(&this->dtex);
                break;
                
            case map_rtex :
                this->add_shader_to(&this->rtex);
                break;

            case map_ttex :
                this->add_shader_to(&this->ttex);
                break;
                
            default :
                assert(false);
                break;
        }
    }

    list<texture_mapper *>  btex;
    list<texture_mapper *>  ctex;
    list<texture_mapper *>  dtex;
    list<texture_mapper *>  ttex;
    list<texture_mapper *>  rtex;
    std::string             filename;
    ext_colour_t            trgb;
    point_t                 tctr;
    point_t                 tnorm;
    point_t                 tsiz;
    fp_t                    tfp[4];
    fp_t                    topc;
    mapper_type_t           shader;
    mapper_of_t             map_of;
    std::uint16_t           tip;
    texture_wrapping_mode_t twrp_mode_x;
    texture_wrapping_mode_t twrp_mode_y;
};


/***********************************************************
  find_surf_chunk will move s to point to the first byte of 
  the SURF chunk (the MSB of the size of the SURF chunk).
  The number of surfaces will also be returned.
  
  l is the number of bytes in the SRFS chunk. s is a pointer
  to move the the start of the SURF chunk and p is a pointer
  to the input file.
************************************************************/
inline std::uint32_t find_surf_chunk(const char *p, const char **s, std::uint32_t l)
{
    /* Parse through the SRFS chunk counting the number of SRFS */
    std::uint32_t num_of_surfs = 0;
    for (std::uint32_t i = 0; i < l; i++)
    {
        if ((*p == 0x00) && ((i & 0x1) == 1))
        {
            num_of_surfs++;
        }
        p++;
    }
    
    /* Check p points where it should */
    check_for_chunk(&p, "POLS", 4);
    
    /* Skip the POLS chunk */
    std::uint32_t tmp = from_byte_stream<std::uint32_t>(&p);
    (*s) = p + tmp;
    
    /* Check the SURF chunk has been found */
    check_for_chunk(s, "SURF", 4);
    
    return num_of_surfs;
}


inline mapper_type_t pick_shader(const char *const c)
{
    if (strcmp(c, "Fractal Noise") == 0)
    {
        return f_noise;
    }
    else if (strcmp(c, "Planar Image Map") == 0)
    {
        return planar;
    }
    else if (strcmp(c, "Cubic Image Map") == 0)
    {
        return cubic;
    }
    else if (strcmp(c, "Cylindrical Image Map") == 0)
    {
        return cylindrical;
    }
    else
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown texture: " << c;
        assert(false);
    }
}


inline void parse_surf(material **m, const string &p, const char **ptr, const std::uint32_t surf_len)
{
    /* Variable to hold all the parameters */
    texture_info_t  current_info;
    ext_colour_t    rgb(255.0, 255.0, 255.0);
    fp_t            vkd = 0.0;
    fp_t            vks = 0.0;
    fp_t            s   = 0.0;
    fp_t            vt  = 0.0;
    fp_t            ri  = 0.0;
    fp_t            vr  = 0.0;
    const char      *tmp_ptr;
    std::uint32_t   i   = 0;
    std::uint16_t   short_tmp;
    
    current_info.shader = non;

    while (i < surf_len)
    {
        tmp_ptr = (*ptr) + 4;
        std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;
        
        if ((current_info.shader != non) && (strncmp((*ptr + 1), "TEX", 3) == 0))
        {
            current_info.add_shader();
        }
        
        /* Base image colour */
        if (strncmp((*ptr), "COLR", 4) == 0)
        {
            rgb.r = static_cast<fp_t>(static_cast<std::uint8_t>((*ptr)[6]));
            rgb.g = static_cast<fp_t>(static_cast<std::uint8_t>((*ptr)[7]));
            rgb.b = static_cast<fp_t>(static_cast<std::uint8_t>((*ptr)[8]));
        }
        else if (strncmp((*ptr), "FLAG", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "FLAG (not handled)";
        }
        /* Integer percentage diffuse co-efficient */
        else if (strncmp((*ptr), "DIFF", 4) == 0)
        {
            if (vkd == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vkd = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr) / 255.0;
            }
            BOOST_LOG_TRIVIAL(trace) << "DIFF: " << vkd;
        }
        /* Floating point percentage diffuse co-efficient */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VDIF", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vkd = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VDIF: " << vkd;
        }
        /* Integer percentage specular co-efficient */
        else if (strncmp((*ptr), "SPEC", 4) == 0)
        {
            if (vks == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vks = from_byte_stream<fp_t>(&tmp_ptr) / 255.0;
            }
            BOOST_LOG_TRIVIAL(trace) << "SPEC: " << vks;
        }
        /* Floating point percentage specular co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VSPC", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vks = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VSPC: " << vks;
        }
        else if (strncmp((*ptr), "GLOS", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            s = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "GLOS: " << s;
        }
        /* Integer percentage transmittance co-efficient */
        else if (strncmp((*ptr), "TRAN", 4) == 0)
        {
            if (vt == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vt = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr) / 255.0;
            }
            BOOST_LOG_TRIVIAL(trace) << "TRAN: " << vt;
        }
        /* Floating point percentage transmittance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VTRN", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vt = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VTRN: " << vt;
        }
        /* Integer percentage reflectance co-efficient */
        else if (strncmp((*ptr), "REFL", 4) == 0)
        {
            if (vr == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vr = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr) / 255.0;
            }
            BOOST_LOG_TRIVIAL(trace) << "REFL: " << vr;
        }
        /* Floating point percentage reflectance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VRFL", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vr = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VRFL: " << vr;
        }
        else if (strncmp((*ptr), "RFLT", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "RFLT (not handled)";
        }
        /* Refractive index */
        else if (strncmp((*ptr), "RIND", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            ri = from_byte_stream<fp_t>(&tmp_ptr);
        }
        /* Integer percentage luminance co-efficient */
        else if (strncmp((*ptr), "LUMI", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "LUMI (not handled)";
        }
        /* Floating point percentage luminance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VLUM", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "VLUM (not handled)";
        }
        else if (strncmp((*ptr), "TFLG", 4) == 0)
        {
            /* Texture flags */
            tmp_ptr = (*ptr) + 6;
            short_tmp = from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TFLG (not handled fully): " << short_tmp;
            
            /* Get the normal */
            current_info.tnorm.x = (fp_t)((short_tmp     ) & 0x1);
            current_info.tnorm.y = (fp_t)((short_tmp >> 1) & 0x1);
            current_info.tnorm.z = (fp_t)((short_tmp >> 2) & 0x1);
        }
        else if (strncmp((*ptr), "TSIZ", 4) == 0)
        {
            /* Texture size */
            tmp_ptr = (*ptr) + 6;
            current_info.tsiz.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tsiz.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tsiz.z = from_byte_stream<fp_t>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "TSIZ: " << current_info.tsiz;
        }
        else if (strncmp((*ptr), "TAAS", 4) == 0)
        {
            /* Texture percentage anti aliasing strength */
            BOOST_LOG_TRIVIAL(trace) << "TAAS (not handled)";
        }
        /* Texture colour */
        else if (strncmp((*ptr), "TCLR", 4) == 0)
        {
            current_info.trgb.r = (fp_t)(*ptr)[6];
            current_info.trgb.g = (fp_t)(*ptr)[7];
            current_info.trgb.b = (fp_t)(*ptr)[8];
            BOOST_LOG_TRIVIAL(trace) << "TCLR: " << current_info.trgb.r << ", " << current_info.trgb.g << ", " << current_info.trgb.b;
        }
        /* Integer texture parameter 0 */
        else if (strncmp((*ptr), "TIP0", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tip = from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TIP0: " << current_info.tip;
        }
        /* Floating point texture parameter 0 */
        else if (strncmp((*ptr), "TFP0", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tfp[0] = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TFP0: " << current_info.tfp[0];
        }
        /* Floating point texture parameter 1 */
        else if (strncmp((*ptr), "TFP1", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tfp[1] = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TFP1: " << current_info.tfp[1];
        }
        /* Texture image name */
        else if (strncmp((*ptr), "TIMG", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "TIMG: " << (*ptr) + 6;
            if (strncmp((*ptr) + 6, "(none)", 6) != 0)
            {
                current_info.filename = p + (*ptr + 6);
            }
        }
        else if (strncmp((*ptr), "TWRP", 4) == 0)
        {
            /* Texture wrapping options */
            tmp_ptr = (*ptr) + 6;
            current_info.twrp_mode_x = (texture_wrapping_mode_t)from_byte_stream<std::uint16_t>(&tmp_ptr);
            current_info.twrp_mode_y = (texture_wrapping_mode_t)from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TWRP: " << current_info.twrp_mode_x << ", " << current_info.twrp_mode_y;
        }
        /* Texture center */
        else if (strncmp((*ptr), "TCTR", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tctr.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tctr.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tctr.z = from_byte_stream<fp_t>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "TCTR: "<< current_info.tctr;
        }
        else if (strncmp((*ptr), "TOPC", 4) == 0)
        {
            /* Texture opaqueness */
            tmp_ptr = (*ptr) + 6;
            current_info.topc = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TOPC: " << current_info.topc;
        }
        /* Algorithmic texture mappers name */
        else if (strncmp((*ptr), "CTEX", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "CTEX: " << (*ptr) + 6;
            current_info.reset(map_ctex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "BTEX", 4) == 0)
        {
            /* Bump map */
            BOOST_LOG_TRIVIAL(trace) << "BTEX: " << (*ptr) + 6;
            current_info.reset(map_btex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "TTEX", 4) == 0)
        {
            /* Transparency texture */
            BOOST_LOG_TRIVIAL(trace) << "TTEX: " << (*ptr) + 6;
            current_info.reset(map_ttex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "RTEX", 4) == 0)
        {
            /* Reflection texture */
            BOOST_LOG_TRIVIAL(trace) << "RTEX: " << (*ptr) + 6;
            current_info.reset(map_rtex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "DTEX", 4) == 0)
        {
            /* Diffuse texture */
            BOOST_LOG_TRIVIAL(trace) << "DTEX: " << (*ptr) + 6;
            current_info.reset(map_dtex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "TAMP", 4) == 0)
        {
            /* Bump texture amplitude */
            BOOST_LOG_TRIVIAL(trace) << "TAMP (not handled)";
        }
        else if (strncmp((*ptr), "TVAL", 4) == 0)
        {
            /* Texture value (Percentage modifier for DTEX STEX RTEX TTEX LTEX) */
            BOOST_LOG_TRIVIAL(trace) << "TVAL (not handled)";
        }
        else if (strncmp((*ptr), "SMAN", 4) == 0)
        {
            /* Maximum smooting angle */
            BOOST_LOG_TRIVIAL(trace) << "SMAN (not handled)";
        }
        /* Catch unknown entities */
        else
        {
            BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< *ptr << " found in SURF chunk";
            assert(false);
        }
        (*ptr) += sec_len + 6;
        i      += sec_len + 6;
        
    }

    if (current_info.shader != non)
    {
        current_info.add_shader();
    }

    /* Create the new material and return */
    *m = new mapper_shader(current_info.ctex, current_info.dtex, current_info.rtex, current_info.ttex, rgb, vkd, vks, s, vt, ri, vr);
}


const char * lwo1_parser(
    const char *const   begin,
    const char         *at,
    string              &p,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c)
{
    METHOD_LOG;

    /* Check the first chunk is PNTS */
    check_for_chunk(&at, "PNTS", 4);
    
    /* Gather all the points */
    std::uint32_t nr_of_verts   = from_byte_stream<std::uint32_t>(&at) / 12;
    point_t *all_points         = new point_t[nr_of_verts];
    for (std::uint32_t i = 0; i < nr_of_verts; i++)
    {
        all_points[i].x = from_byte_stream<fp_t>(&at);
        all_points[i].y = from_byte_stream<fp_t>(&at);
        all_points[i].z = from_byte_stream<fp_t>(&at);
    }
    
    /* Check this is the SRFS chunk */
    check_for_chunk(&at, "SRFS", 4);
    
    /* Find the SURF chunk */
    const char *surfs_start;
    std::uint32_t srfs_len       = from_byte_stream<std::uint32_t>(&at);
    std::uint32_t num_of_surfs   = find_surf_chunk(at, &surfs_start, srfs_len);

    /* Parse the names of the surfaces (SRFS chunk) */
    std::unique_ptr<const char *[]> srfs(new const char *[num_of_surfs]);
    for (std::uint32_t i = 0; i < num_of_surfs; i++)
    {
        srfs[i] = at;
        at += strlen(srfs[i]) + 1;
        if (*at == 0x00)
        {
            ++at;
        }
    }
    
    /* Parse the materials (SURF chunk) */
    material **surf_materials = new material *[num_of_surfs];
    for (std::uint32_t i = 0; i < num_of_surfs; i ++)
    {
        const std::uint32_t surf_len = from_byte_stream<std::uint32_t>(&surfs_start);
        if(strcmp(surfs_start, srfs[i]) != 0)
        {
            BOOST_LOG_TRIVIAL(error) << "Expected: " << srfs[i];
            BOOST_LOG_TRIVIAL(error) << "Actual  : " << surfs_start;
            assert(!"Expected and current surface dont match");
        }
        
        std::uint32_t srf_len = strlen(srfs[i]) + 1;
        srf_len     += srf_len & 0x1;
        surfs_start += srf_len;
        
        BOOST_LOG_TRIVIAL(trace) << "Parsing material: " << srfs[i];
        parse_surf(&surf_materials[i], p, &surfs_start, surf_len - srf_len);
        m.push_back(surf_materials[i]);
        
        surfs_start += 4;
    }

    /* Check this is the POLS chunk */
    check_for_chunk(&at, "POLS", 4);
    
    /* Gather all the polygons */
    vector<point_t> pol_vert;
    std::uint16_t vert_this_pol = 0;
    std::uint32_t pols_bytes    = from_byte_stream<std::uint32_t>(&at);
    for (std::uint32_t i = 0; i < pols_bytes; i += (4 + (vert_this_pol << 1)))
    {
        vert_this_pol = from_byte_stream<std::uint16_t>(&at);
        for (std::uint32_t j = 0; j < vert_this_pol; j++)
        {
            std::uint16_t vert_num = from_byte_stream<std::uint16_t>(&at);
            assert(vert_num < nr_of_verts);
            
            pol_vert.push_back(all_points[vert_num]);
        }
        
        /* Parse the material to use */
        std::int16_t mat_num = from_byte_stream<std::int16_t>(&at);

        /* Check for detail polygons, but then parse them as normal polygons */
        if (mat_num < 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "Found detail polygons at: 0x" << hex << (std::uint32_t)((at) - begin) << dec;
            from_byte_stream<std::int16_t>(&at);
            mat_num = abs(mat_num);
            i += 2;
        }

        /* Range check the material */
        if ((std::uint32_t)mat_num > num_of_surfs)
        {
            BOOST_LOG_TRIVIAL(error) << "Material " << hex << mat_num << " out of range at " << (std::uint32_t)((at) - begin) << dec;
            assert(false);
        }

        /* Create the polygon */
        face_to_triangles(&e, &l, pol_vert, surf_materials[mat_num - 1], false);
        
        /* Clean up */
        pol_vert.clear();
    }

    /* Check for and skip SURF chunk, which has already been parsed */
    /* Yes all this really is necassary, for whatever reason the SURF chunk might be longer than it claims */
    for (std::uint32_t i = 0; i < num_of_surfs; i ++)
    {
        check_for_chunk(&at, "SURF", 4);

        std::uint32_t surf_len = from_byte_stream<std::uint32_t>(&at);
        std::uint32_t srf_len = strlen(srfs[i]) + 1;
        srf_len     += srf_len & 0x1;
        at          += srf_len;
        surf_len    -= srf_len;
        
        while (static_cast<int>(surf_len) > 0)
        {
            at += 4;
            const std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&at);
            at          += sec_len;
            surf_len    -= sec_len + 6;
        }
    }

    /* Clean up */
    delete [] all_points;
    delete [] surf_materials;

    return at;
}
}; /* namespace raptor_raytracer */
