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
            rgb.r = static_cast<fp_t>(static_cast<std::uint8_t>(tmp_ptr[0]));
            rgb.g = static_cast<fp_t>(static_cast<std::uint8_t>(tmp_ptr[1]));
            rgb.b = static_cast<fp_t>(static_cast<std::uint8_t>(tmp_ptr[2]));
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "COLR: "<< rgb.r << ", " << rgb.g << ", " << rgb.b;
        }
        /* Integer percentage diffuse co-efficient */
        else if (strncmp((*ptr), "DIFF", 4) == 0)
        {
            if (vkd == 0.0)
            {
                vkd = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr) / 255.0;
            }
            BOOST_LOG_TRIVIAL(trace) << "DIFF: " << vkd;
        }
        /* Floating point percentage diffuse co-efficient */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VDIF", 4) == 0)
        {
            vkd = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VDIF: " << vkd;
        }
        /* Integer percentage specular co-efficient */
        else if (strncmp((*ptr), "SPEC", 4) == 0)
        {
            if (vks == 0.0)
            {
                vks = from_byte_stream<fp_t>(&tmp_ptr) / 255.0;
            }
            else
            {
                tmp_ptr += 4;
            }
            BOOST_LOG_TRIVIAL(trace) << "SPEC: " << vks;
        }
        /* Floating point percentage specular co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VSPC", 4) == 0)
        {
            vks = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VSPC: " << vks;
        }
        else if (strncmp((*ptr), "GLOS", 4) == 0)
        {
            s = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "GLOS: " << s;
        }
        /* Integer percentage transmittance co-efficient */
        else if (strncmp((*ptr), "TRAN", 4) == 0)
        {
            if (vt == 0.0)
            {
                vt = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr) / 255.0;
            }
            else
            {
                tmp_ptr += 4;
            }
            BOOST_LOG_TRIVIAL(trace) << "TRAN: " << vt;
        }
        /* Floating point percentage transmittance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VTRN", 4) == 0)
        {
            vt = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VTRN: " << vt;
        }
        /* Integer percentage reflectance co-efficient */
        else if (strncmp((*ptr), "REFL", 4) == 0)
        {
            if (vr == 0.0)
            {
                vr = (fp_t)from_byte_stream<std::uint16_t>(&tmp_ptr)/255.0;
            }
            else
            {
                tmp_ptr += 4;
            }
            BOOST_LOG_TRIVIAL(trace) << "REFL: " << vr;
        }
        /* Floating point percentage reflectance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VRFL", 4) == 0)
        {
            vr = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VRFL: " << vr;
        }
        /* Refractive index */
        else if (strncmp((*ptr), "RIND", 4) == 0)
        {
            ri = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "RIND: " << ri;
        }
        /* Integer percentage luminance co-efficient */
        else if (strncmp((*ptr), "LUMI", 4) == 0)
        {
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "LUMI (not handled)";
        }
        else if (strncmp((*ptr), "TFLG", 4) == 0)
        {
            /* Texture flags */
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
            current_info.tsiz.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tsiz.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tsiz.z = from_byte_stream<fp_t>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "TSIZ: "<< current_info.tsiz;
        }
        /* Texture colour */
        else if (strncmp((*ptr), "TCLR", 4) == 0)
        {
            current_info.trgb.r = (fp_t)(*ptr)[6];
            current_info.trgb.g = (fp_t)(*ptr)[7];
            current_info.trgb.b = (fp_t)(*ptr)[8];
            BOOST_LOG_TRIVIAL(trace) << "TCLR: "<< current_info.trgb.r << ", " << current_info.trgb.g << ", " << current_info.trgb.b;
        }
        /* Integer texture parameter 0 */
        else if (strncmp((*ptr), "TIP0", 4) == 0)
        {
            current_info.tip = from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TIP0: " << current_info.tip;
        }
        /* Floating point texture parameter 0 */
        else if (strncmp((*ptr), "TFP0", 4) == 0)
        {
            current_info.tfp[0] = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TFP0: " << current_info.tfp[0];
        }
        /* Floating point texture parameter 1 */
        else if (strncmp((*ptr), "TFP1", 4) == 0)
        {
            current_info.tfp[1] = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TFP1: " << current_info.tfp[1];
        }
        /* Texture image name */
        else if (strncmp((*ptr), "TIMG", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "TIMG: " << tmp_ptr;
            if (strncmp(tmp_ptr, "(none)", 6) != 0)
            {
                current_info.filename = p + tmp_ptr;
            }
            std::uint32_t timg_len = strlen(tmp_ptr) + 1;
            timg_len += timg_len & 0x1;
            tmp_ptr += timg_len;
        }
        else if (strncmp((*ptr), "TWRP", 4) == 0)
        {
            /* Texture wrapping options */
            current_info.twrp_mode_x = (texture_wrapping_mode_t)from_byte_stream<std::uint16_t>(&tmp_ptr);
            current_info.twrp_mode_y = (texture_wrapping_mode_t)from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TWRP: " << current_info.twrp_mode_x << ", " << current_info.twrp_mode_y;
        }
        /* Texture center */
        else if (strncmp((*ptr), "TCTR", 4) == 0)
        {
            current_info.tctr.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tctr.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tctr.z = from_byte_stream<fp_t>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "TCTR: "<< current_info.tctr;
        }
        else if (strncmp((*ptr), "TAMP", 4) == 0)
        {
            /* Bump texture amplitude */
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "TAMP (not handled)";
        }
        else if (strncmp((*ptr), "SMAN", 4) == 0)
        {
            /* Maximum smooting angle */
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "SMAN (not handled)";
        }
        /* Catch unknown entities */
        else
        {
            BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< *ptr << " found in SURF chunk";
            assert(false);
        }

        /* Envelop */
        const std::uint16_t envelop_id = from_byte_stream<std::uint16_t>(&tmp_ptr);
        assert(envelop_id == 0);   /* The envelop must be null */

        (*ptr) += sec_len + 6;
        i += sec_len + 6;
        
    }

    if (current_info.shader != non)
    {
        current_info.add_shader();
    }

    /* Create the new material and return */
    *m = new mapper_shader(current_info.ctex, current_info.dtex, current_info.rtex, current_info.ttex, rgb, vkd, vks, s, vt, ri, vr);
}


void parse_surf(list<material *> &m, const std::string &p, const std::map<std::string, std::uint16_t> &tag_map, material **surf_materials, const char *at)
{
    /* Check this is the POLS chunk */
    check_for_chunk(&at, "POLS", 4);
    
    /* Skip the POLS chunk */
    const std::uint32_t pols_len = from_byte_stream<std::uint32_t>(&at);
    at += pols_len;

    /* Check this is the PTAG chunk */
    check_for_chunk(&at, "PTAG", 4);
    
    /* Skip the POLS chunk */
    const std::uint32_t ptag_len = from_byte_stream<std::uint32_t>(&at);
    at += ptag_len;

    /* Check for CLIP chunks */
    while (strncmp(at, "CLIP", 4) == 0)
    {
        at += 4;
        const std::uint32_t clip_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "CLIP (not handled): " << clip_len;
        at += clip_len;
    }
    
    /* Check the SURF chunk has been found */
    check_for_chunk(&at, "SURF", 4);

    for (std::uint32_t i = 0; i < tag_map.size(); ++i)
    {
        std::uint32_t surf_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "Parsing material: " << at << ". Length: " << surf_len;
        if ((tag_map.find(at) == tag_map.end()) || (tag_map.find(at)->second != i))
        {
            assert(!"Current surface not found in tags");
        }
        
        std::uint32_t srf_len = strlen(at) + 1;
        srf_len     += srf_len & 0x1;
        at          += srf_len;

        /* Parent surface */
        assert((*at) == 0x00);   /* The source surface must be null */
        std::uint32_t source_len = strlen(at) + 1;
        source_len  += source_len & 0x1;
        at          += source_len;
        BOOST_LOG_TRIVIAL(trace) << "Name length: " << srf_len << " Source length: " << source_len;
        
        parse_surf(&surf_materials[i], p, &at, surf_len - srf_len - source_len);
        m.push_back(surf_materials[i]);
    }
}


void parse_pols(light_list &l, primitive_list &e, const std::map<std::string, std::uint16_t> &tag_map, material *const *const surf_materials, const point_t *const all_points, const char **at, const char *const buffer, const std::uint32_t nr_of_verts)
{
    /* Check this is the POLS chunk */
    check_for_chunk(at, "POLS", 4);
    const std::uint32_t pols_bytes = from_byte_stream<std::uint32_t>(at);
    const char *ptags_at = (*at) + pols_bytes;

    /* Check for FACE chunk */
    check_for_chunk(at, "FACE", 4);

    /* Find PTAGS */
    check_for_chunk(&ptags_at, "PTAG", 4);
    const std::uint32_t ptag_len = from_byte_stream<std::uint32_t>(&ptags_at);

    /* Check they are surface tags */
    check_for_chunk(&ptags_at, "SURF", 4);

    /* Gather all the polygons */
    std::uint16_t pol = 0;
    vector<point_t> pol_vert;
    std::uint16_t vert_this_pol = 0;
    std::uint32_t num_of_surfs = tag_map.size();
    for (std::uint32_t i = 0; i < pols_bytes - 4; i += (2 + (vert_this_pol << 1)))
    {
        vert_this_pol = from_byte_stream<std::uint16_t>(at);
        for (std::uint32_t j = 0; j < vert_this_pol; j++)
        {
            std::uint16_t vert_num = from_byte_stream<std::uint16_t>(at);
            assert(vert_num < nr_of_verts);
            
            pol_vert.push_back(all_points[vert_num]);
        }
        
        /* Parse the material to use */
        const std::uint16_t ptag_pol = from_byte_stream<std::uint16_t>(&ptags_at);
        const std::uint16_t mat_num = from_byte_stream<std::uint16_t>(&ptags_at);
        BOOST_LOG_TRIVIAL(trace) << "Poly: " << ptag_pol << " (" << pol << ") assign surf: " << mat_num;
        assert((ptag_pol == pol) || !"Error: ptag is not for this polygon");

        /* Range check the material */
        if ((std::uint32_t)mat_num > num_of_surfs)
        {
            BOOST_LOG_TRIVIAL(error) << "Material " << hex << mat_num << " out of range at " << (std::uint32_t)((*at) - buffer) << dec;
            assert(false);
        }

        /* Create the polygon */
        face_to_triangles(&e, &l, pol_vert, surf_materials[mat_num], false);
        
        /* Clean up */
        pol_vert.clear();
        ++pol;
    }

    /* Check for and skip ptag */
    check_for_chunk(at, "PTAG", 4);
    (*at) += ptag_len + 4;
}


const char * lwo2_parser(
    const char *const   begin,
    const char         *at,
    string              &p,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c)
{
    METHOD_LOG;
    
    /* Check for TAGS */
    std::map<std::string, std::uint16_t> tag_map;
    check_for_chunk(&at, "TAGS", 4);

    std::uint32_t tag_len = from_byte_stream<std::uint32_t>(&at);
    BOOST_LOG_TRIVIAL(trace) << "TAGS: " << tag_len;
    
    std::uint16_t tag_id = 0;
    std::uint32_t tag_idx = 0;
    while (tag_idx < tag_len)
    {
        BOOST_LOG_TRIVIAL(trace) << "Adding tag: " << &at[tag_idx] << " at tag id: " << tag_id;
        tag_map.emplace(std::string(&at[tag_idx]), tag_id++);
        tag_idx += strlen(&at[tag_idx]) + 1;
        tag_idx += tag_idx & 0x1;
    }
    at += tag_len;

    /* LAYR chunk, not currently used */
    check_for_chunk(&at, "LAYR", 4);
    std::uint32_t layr_len = from_byte_stream<std::uint32_t>(&at);
    BOOST_LOG_TRIVIAL(trace) << "LAYR (not handled): " << layr_len;
    at += layr_len;

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

    /* Check for optional BBOX chunk */
    if (strncmp(at, "BBOX", 4) == 0)
    {
        at += 4;
        std::uint32_t bbox_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "BBOX (not handled): " << bbox_len;
        at += bbox_len;
    }
    
    /* Look for the SURF chunks */
    material **surf_materials;
    surf_materials = new material *[tag_map.size()];
    parse_surf(m, p, tag_map, surf_materials, at);
    
    /* Parse POLS */
    parse_pols(l, e, tag_map, surf_materials, all_points, &at, begin, nr_of_verts);

    /* Check for and skip CLIP chunks */
    while (strncmp(at, "CLIP", 4) == 0)
    {
        at += 4;
        const std::uint32_t clip_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "CLIP (not handled): " << clip_len;
        at += clip_len;
    }

    /* Check for and skip SURF chunk */
    check_for_chunk(&at, "SURF", 4);
    for (std::uint32_t i = 0; i < tag_map.size(); ++i)
    {
        const std::uint32_t surf_len = from_byte_stream<std::uint32_t>(&at);
        at += surf_len + 4;
    }
    
    /* Clean up */
    delete [] all_points;
    delete [] surf_materials;

    return at;
}
}; /* namespace raptor_raytracer */
