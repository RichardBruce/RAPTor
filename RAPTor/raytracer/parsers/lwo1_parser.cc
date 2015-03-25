/* Standard headers */
#include <cstdint>

/* Common headers */
#include "common.h"
#include "logging.h"

/* Ray tracer headers */
#include "normal_calculator.h"
#include "picture_functions.h"
#include "parser_common.h"
#include "lwo_parser.h"
#include "camera.h"


namespace raptor_raytracer
{
struct texture_info_t
{
    void reset(const mapper_of_t m, const mapper_type_t s)
    {
        filename        = "";
        trgb            = ext_colour_t(255.0f, 255.0f, 255.0f);
        topc            = 1.0f;
        tamp            = 1.0f;
        tval            = 1.0f;
        shader          = s;
        map_of          = m;
        twrp_mode_x     = static_cast<texture_wrapping_mode_t>(2);
        twrp_mode_y     = static_cast<texture_wrapping_mode_t>(2);
        tneg            = false;
    }
    
    void add_shader_to(std::vector<texture_mapper *>  *const t, const float scale)
    {
        float *img;
        std::uint32_t img_width;
        std::uint32_t img_height;
        std::uint32_t cpp;
        texture_mapper  *tm = nullptr;
        switch (this->shader)
        {
            case mapper_type_t::f_noise :
                tm = new perlin_noise_3d_mapper(this->trgb * scale, this->tfp[1], this->tfp[0], this->tip, 4);
                break;
                
            case mapper_type_t::cylindrical  :
                if (!filename.empty())
                {
                    cpp = read_jpeg(&img, this->filename.c_str(), &img_height, &img_width);
                    brightness_scale(img, img_width * img_height * cpp, scale);
                    tm = new cylindrical_mapper(boost::shared_array<float>(img), this->tctr, this->tnorm, this->tsiz, this->tfp[0], img_height, img_width, cpp);
                }
                break;
    
            case mapper_type_t::planar  :
                if (!filename.empty())
                {
                    cpp = read_jpeg(&img, this->filename.c_str(), &img_height, &img_width);
                    brightness_scale(img, img_width * img_height * cpp, scale);
                    tm  = new planar_mapper(boost::shared_array<float>(img), this->tctr, this->tnorm, this->tsiz, cpp, img_width, img_height, this->twrp_mode_x, this->twrp_mode_y);
                }
                break;
    
            case mapper_type_t::cubic  :
                if (!filename.empty())
                {
                    cpp = read_jpeg(&img, this->filename.c_str(), &img_height, &img_width);
                    brightness_scale(img, img_width * img_height * cpp, scale);
                    tm  = new cubic_mapper(boost::shared_array<float>(img), this->tctr, this->tnorm, this->tsiz, this->twrp_mode_x, this->twrp_mode_y, cpp, img_width, img_height);
                }
                break;
    
            default :
                assert(false);
                break;
        }

        if (tm != nullptr)
        {
            tm->inverse(tneg);
            t->push_back(tm);
        }
    }
    
    void add_shader()
    {
        switch (this->map_of)
        {
            case mapper_of_t::map_btex :
                this->add_shader_to(&this->btex, tamp);
                break;

            case mapper_of_t::map_ctex :
                this->add_shader_to(&this->ctex, 1.0f);
                break;
                
            case mapper_of_t::map_dtex :
                this->add_shader_to(&this->dtex, tval);
                break;
                
            case mapper_of_t::map_stex :
                this->add_shader_to(&this->stex, tval);
                break;
                
            case mapper_of_t::map_rtex :
                this->add_shader_to(&this->rtex, tval);
                break;

            case mapper_of_t::map_ttex :
                this->add_shader_to(&this->ttex, tval);
                break;
                
            default :
                assert(false);
                break;
        }
    }

    std::vector<texture_mapper *>   btex;
    std::vector<texture_mapper *>   ctex;
    std::vector<texture_mapper *>   dtex;
    std::vector<texture_mapper *>   stex;
    std::vector<texture_mapper *>   ttex;
    std::vector<texture_mapper *>   rtex;
    std::string                     filename;
    ext_colour_t                    trgb;
    point_t                         tctr;
    point_t                         tnorm;
    point_t                         tsiz;
    float                           tfp[4];
    float                           topc;
    float                           tamp;
    float                           tval;
    mapper_type_t                   shader;
    mapper_of_t                     map_of;
    std::uint16_t                   tip;
    texture_wrapping_mode_t         twrp_mode_x;
    texture_wrapping_mode_t         twrp_mode_y;
    bool                            tneg;
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
            ++num_of_surfs;
        }
        ++p;
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
        return mapper_type_t::f_noise;
    }
    else if (strcmp(c, "Planar Image Map") == 0)
    {
        return mapper_type_t::planar;
    }
    else if (strcmp(c, "Cubic Image Map") == 0)
    {
        return mapper_type_t::cubic;
    }
    else if (strcmp(c, "Cylindrical Image Map") == 0)
    {
        return mapper_type_t::cylindrical;
    }
    else
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown texture: " << c;
        assert(false);
    }
}


inline static float parse_surf(material **m, const std::string &p, const char **ptr, const std::uint32_t surf_len)
{
    /* Variable to hold all the parameters */
    texture_info_t  current_info;
    ext_colour_t    rgb(255.0f, 255.0f, 255.0f);
    float           vkd     = 0.0f;
    float           vks     = 0.0f;
    float           s       = 64.0f;
    float           vt      = 0.0f;
    float           ri      = 0.0f;
    float           vr      = 0.0f;
    float           sman    = 0.0f;
    const char      *tmp_ptr;
    std::uint32_t   i   = 0;
    std::uint16_t   short_tmp;
    
    current_info.shader = mapper_type_t::non;

    while (i < surf_len)
    {
        tmp_ptr = (*ptr) + 4;
        std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;
        
        if ((current_info.shader != mapper_type_t::non) && (strncmp((*ptr + 1), "TEX", 3) == 0))
        {
            current_info.add_shader();
        }
        
        /* Base image colour */
        if (strncmp((*ptr), "COLR", 4) == 0)
        {
            rgb.r = static_cast<float>(static_cast<std::uint8_t>((*ptr)[6]));
            rgb.g = static_cast<float>(static_cast<std::uint8_t>((*ptr)[7]));
            rgb.b = static_cast<float>(static_cast<std::uint8_t>((*ptr)[8]));
        }
        else if (strncmp((*ptr), "FLAG", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(warning) << "FLAG (not handled)";
        }
        /* Integer percentage diffuse co-efficient */
        else if (strncmp((*ptr), "DIFF", 4) == 0)
        {
            if (vkd == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vkd = static_cast<float>(from_byte_stream<std::uint16_t>(&tmp_ptr)) / 255.0f;
            }
            BOOST_LOG_TRIVIAL(trace) << "DIFF: " << vkd;
        }
        /* Floating point percentage diffuse co-efficient */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VDIF", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vkd = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VDIF: " << vkd;
        }
        /* Integer percentage specular co-efficient */
        else if (strncmp((*ptr), "SPEC", 4) == 0)
        {
            if (vks == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vks = from_byte_stream<float>(&tmp_ptr) / 255.0f;
            }
            BOOST_LOG_TRIVIAL(trace) << "SPEC: " << vks;
        }
        /* Floating point percentage specular co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VSPC", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vks = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VSPC: " << vks;
        }
        else if (strncmp((*ptr), "GLOS", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            s = static_cast<float>(from_byte_stream<std::uint16_t>(&tmp_ptr));
            BOOST_LOG_TRIVIAL(trace) << "GLOS: " << s;
        }
        /* Integer percentage transmittance co-efficient */
        else if (strncmp((*ptr), "TRAN", 4) == 0)
        {
            if (vt == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vt = static_cast<float>(from_byte_stream<std::uint16_t>(&tmp_ptr)) / 255.0f;
            }
            BOOST_LOG_TRIVIAL(trace) << "TRAN: " << vt;
        }
        /* Floating point percentage transmittance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VTRN", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vt = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VTRN: " << vt;
        }
        /* Integer percentage reflectance co-efficient */
        else if (strncmp((*ptr), "REFL", 4) == 0)
        {
            if (vr == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vr = static_cast<float>(from_byte_stream<std::uint16_t>(&tmp_ptr)) / 255.0f;
            }
            BOOST_LOG_TRIVIAL(trace) << "REFL: " << vr;
        }
        /* Floating point percentage reflectance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VRFL", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vr = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VRFL: " << vr;
        }
        else if (strncmp((*ptr), "RFLT", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(warning) << "RFLT (not handled)";
        }
        /* Refractive index */
        else if (strncmp((*ptr), "RIND", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            ri = from_byte_stream<float>(&tmp_ptr);
        }
        /* Integer percentage luminance co-efficient */
        else if (strncmp((*ptr), "LUMI", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(warning) << "LUMI (not handled)";
        }
        /* Floating point percentage luminance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VLUM", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(warning) << "VLUM (not handled)";
        }
        else if (strncmp((*ptr), "TFLG", 4) == 0)
        {
            /* Texture flags */
            tmp_ptr = (*ptr) + 6;
            short_tmp = from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "TFLG (not handled fully): " << short_tmp;
            
            /* Get the normal */
            current_info.tnorm.x = static_cast<float>((short_tmp     ) & 0x1);
            current_info.tnorm.y = static_cast<float>((short_tmp >> 1) & 0x1);
            current_info.tnorm.z = static_cast<float>((short_tmp >> 2) & 0x1);

            current_info.tneg = (short_tmp >> 4) & 0x1;
        }
        else if (strncmp((*ptr), "TSIZ", 4) == 0)
        {
            /* Texture size */
            tmp_ptr = (*ptr) + 6;
            current_info.tsiz.x = from_byte_stream<float>(&tmp_ptr);
            current_info.tsiz.y = from_byte_stream<float>(&tmp_ptr);
            current_info.tsiz.z = from_byte_stream<float>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "TSIZ: " << current_info.tsiz;
        }
        else if (strncmp((*ptr), "TAAS", 4) == 0)
        {
            /* Texture percentage anti aliasing strength */
            BOOST_LOG_TRIVIAL(warning) << "TAAS (not handled)";
        }
        /* Texture colour */
        else if (strncmp((*ptr), "TCLR", 4) == 0)
        {
            current_info.trgb.r = static_cast<float>((*ptr)[6]);
            current_info.trgb.g = static_cast<float>((*ptr)[7]);
            current_info.trgb.b = static_cast<float>((*ptr)[8]);
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
            current_info.tfp[0] = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TFP0: " << current_info.tfp[0];
        }
        /* Floating point texture parameter 1 */
        else if (strncmp((*ptr), "TFP1", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tfp[1] = from_byte_stream<float>(&tmp_ptr);
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
            current_info.twrp_mode_x = static_cast<texture_wrapping_mode_t>(from_byte_stream<std::uint16_t>(&tmp_ptr));
            current_info.twrp_mode_y = static_cast<texture_wrapping_mode_t>(from_byte_stream<std::uint16_t>(&tmp_ptr));
            BOOST_LOG_TRIVIAL(trace) << "TWRP: " << static_cast<int>(current_info.twrp_mode_x) << ", " << static_cast<int>(current_info.twrp_mode_y);
        }
        /* Texture center */
        else if (strncmp((*ptr), "TCTR", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tctr.x = from_byte_stream<float>(&tmp_ptr);
            current_info.tctr.y = from_byte_stream<float>(&tmp_ptr);
            current_info.tctr.z = from_byte_stream<float>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "TCTR: "<< current_info.tctr;
        }
        else if (strncmp((*ptr), "TOPC", 4) == 0)
        {
            /* Texture opaqueness */
            tmp_ptr = (*ptr) + 6;
            current_info.topc = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TOPC: " << current_info.topc;
        }
        /* Algorithmic texture mappers name */
        else if (strncmp((*ptr), "CTEX", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "CTEX: " << (*ptr) + 6;
            current_info.reset(mapper_of_t::map_ctex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "BTEX", 4) == 0)
        {
            /* Bump map */
            BOOST_LOG_TRIVIAL(trace) << "BTEX: " << (*ptr) + 6;
            current_info.reset(mapper_of_t::map_btex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "TTEX", 4) == 0)
        {
            /* Transparency texture */
            BOOST_LOG_TRIVIAL(trace) << "TTEX: " << (*ptr) + 6;
            current_info.reset(mapper_of_t::map_ttex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "RTEX", 4) == 0)
        {
            /* Reflection texture */
            BOOST_LOG_TRIVIAL(trace) << "RTEX: " << (*ptr) + 6;
            current_info.reset(mapper_of_t::map_rtex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "DTEX", 4) == 0)
        {
            /* Diffuse texture */
            BOOST_LOG_TRIVIAL(trace) << "DTEX: " << (*ptr) + 6;
            current_info.reset(mapper_of_t::map_dtex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "TAMP", 4) == 0)
        {
            /* Bump texture amplitude */
            tmp_ptr = (*ptr) + 6;
            current_info.tamp = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TAMP: " << current_info.tamp;
        }
        else if (strncmp((*ptr), "TVAL", 4) == 0)
        {
            /* Texture value (Percentage modifier for DTEX STEX RTEX TTEX LTEX) */
            tmp_ptr = (*ptr) + 6;
            current_info.tval = from_byte_stream<std::uint16_t>(&tmp_ptr) / 256.0f;
            BOOST_LOG_TRIVIAL(trace) << "TVAL: " << current_info.tval;
        }
        else if (strncmp((*ptr), "SMAN", 4) == 0)
        {
            /* Maximum smooting angle */
            tmp_ptr = (*ptr) + 6;
            sman = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "SMAN: " << sman;
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

    if (current_info.shader != mapper_type_t::non)
    {
        current_info.add_shader();
    }

    /* Create the new material and return */
    *m = new mapper_shader(current_info.btex, current_info.ctex, current_info.dtex, current_info.stex, current_info.rtex, current_info.ttex, rgb, vkd, vks, s, vt, ri, vr);
    return sman;
}


const char * lwo1_parser(
    const char *const       begin,
    const char              *at,
    std::string             &p,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c)
{
    METHOD_LOG;

    /* Check the first chunk is PNTS */
    check_for_chunk(&at, "PNTS", 4);
    
    /* Gather all the points */
    std::uint32_t nr_of_verts   = from_byte_stream<std::uint32_t>(&at) / 12;
    std::vector<point_t> all_points(nr_of_verts);
    for (std::uint32_t i = 0; i < nr_of_verts; i++)
    {
        all_points[i].x = from_byte_stream<float>(&at);
        all_points[i].y = from_byte_stream<float>(&at);
        all_points[i].z = from_byte_stream<float>(&at);
    }
    
    /* Check this is the SRFS chunk */
    check_for_chunk(&at, "SRFS", 4);
    
    /* Find the SURF chunk */
    const char *surfs_start;
    std::uint32_t srfs_len       = from_byte_stream<std::uint32_t>(&at);
    std::uint32_t num_of_surfs   = find_surf_chunk(at, &surfs_start, srfs_len);

    /* Parse the names of the surfaces (SRFS chunk) */
    std::unique_ptr<const char *[]> srfs(new const char *[num_of_surfs]);
    for (std::uint32_t i = 0; i < num_of_surfs; ++i)
    {
        srfs[i] = at;
        at += strlen(srfs[i]) + 1;
        if (*at == 0x00)
        {
            ++at;
        }
    }
    
    /* Parse the materials (SURF chunk) */
    std::vector<float> smoothing_threshold;
    material **surf_materials = new material *[num_of_surfs];
    for (std::uint32_t i = 0; i < num_of_surfs; ++i)
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
        smoothing_threshold.push_back(parse_surf(&surf_materials[i], p, &surfs_start, surf_len - srf_len));
        m.push_back(surf_materials[i]);
        
        surfs_start += 4;
    }

    /* Check this is the POLS chunk */
    check_for_chunk(&at, "POLS", 4);
    
    /* Gather all the polygons */
    std::uint32_t pol = 0;
    normal_calculator nc(all_points);

    std::vector<int> pol_vert;
    std::uint16_t vert_this_pol = 0;
    std::uint32_t pols_bytes    = from_byte_stream<std::uint32_t>(&at);
    for (std::uint32_t i = 0; i < pols_bytes; i += (4 + (vert_this_pol << 1)))
    {
        vert_this_pol = from_byte_stream<std::uint16_t>(&at);
        for (std::uint32_t j = 0; j < vert_this_pol; j++)
        {
            std::uint16_t vert_num = from_byte_stream<std::uint16_t>(&at);
            assert(vert_num < nr_of_verts);
            
            pol_vert.push_back(vert_num);
        }
        
        /* Parse the material to use */
        std::int16_t mat_num = from_byte_stream<std::int16_t>(&at);

        /* Check for detail polygons, but then parse them as normal polygons */
        if (mat_num < 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "Found detail polygons at: 0x" << std::hex << static_cast<std::uint32_t>((at) - begin) << std::dec;
            from_byte_stream<std::int16_t>(&at);
            mat_num = abs(mat_num);
            i += 2;
        }

        /* Range check the material */
        if ((std::uint32_t)mat_num > num_of_surfs)
        {
            BOOST_LOG_TRIVIAL(error) << "Material " << std::hex << mat_num << " out of range at " << static_cast<std::uint32_t>((at) - begin) << std::dec;
            assert(false);
        }

        /* Remember the polygon */
        nc.add_point_usage(pol_vert, smoothing_threshold[mat_num - 1], mat_num, pol);
        
        /* Clean up */
        pol_vert.clear();
        ++pol;
    }

    /* Calculate normals for smoothed polygons */
    nc.calculate();

    /* Build polyongs */
    std::vector<point_t> pol_pnts;
    std::vector<point_t> pol_norm;
    for (int i = 0; i < nc.number_of_polygons(); ++i)
    {
        /* Check we have enough points */
        if (nc.number_of_points(i) < 3)
        {
            continue;
        }

        /* Get normals */
        auto *const norms = nc.normals(&pol_norm, i);
        nc.points_on_polygon(&pol_pnts, i);

        /* Create the polygon */
        assert((norms == nullptr) || (pol_norm.size() == pol_pnts.size()));
        face_to_triangles(&e, &l, pol_pnts, surf_materials[nc.group(i) - 1], false, norms);
        
        /* Clean up */
        pol_pnts.clear();
        pol_norm.clear();
    }

    /* Check for and skip SURF chunk, which has already been parsed */
    /* Yes all this really is necassary, for whatever reason the SURF chunk might be longer than it claims */
    for (std::uint32_t i = 0; i < num_of_surfs; ++i)
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
    delete [] surf_materials;

    return at;
}
}; /* namespace raptor_raytracer */
