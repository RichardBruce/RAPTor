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
enum mapper_type_t { non = 0, f_noise = 1, planar = 2, cubic = 3, spherical = 4, cylindrical = 5, f_checker = 6 };
enum mapper_of_t   { map_btex = 0, map_ctex = 1, map_dtex = 2, map_ltex = 3, map_stex = 4, map_rtex = 5, map_ttex = 6, map_grad = 7 };


std::uint32_t parse_vx(const char **at)
{
    std::uint32_t idx = from_byte_stream<std::uint16_t>(at);
    if ((idx & 0xff00) == 0xff00)
    {
        idx &= 0x00ff;
        idx <<= 16;
        idx += from_byte_stream<std::uint16_t>(at);
    }

    return idx;
}


struct lwo2_texture_info_t
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
                BOOST_LOG_TRIVIAL(trace) << "Type of shader is f_noise";
                tm = new perlin_noise_3d_mapper(this->trgb, this->tfp[1], this->tfp[0], this->tip, 4);
                t->push_front(tm);
                break;
                
            case cylindrical  :
                BOOST_LOG_TRIVIAL(trace) << "Type of shader is cylindrical";
                tm = new cylindrical_mapper(this->filename.c_str(), this->tctr, this->tnorm, this->tsiz, this->tfp[0]);
                t->push_front(tm);
                break;
    
            case planar  :
                BOOST_LOG_TRIVIAL(trace) << "Type of shader is planar";
                cpp = read_jpeg(&img, this->filename.c_str(), &img_height, &img_width);
                tm  = new planar_mapper(img, this->tctr, this->tnorm, this->tsiz, cpp, img_width, img_height, this->twrp_mode_x, this->twrp_mode_y);
                t->push_front(tm);
                break;
    
            case cubic  :
                BOOST_LOG_TRIVIAL(trace) << "Type of shader is cubic";
                tm  = new cubic_mapper(this->filename.c_str(), this->tctr, this->tnorm, this->tsiz, this->twrp_mode_x, this->twrp_mode_y);
                t->push_front(tm);
                break;

            case f_checker :
                BOOST_LOG_TRIVIAL(trace) << "Type of shader is checker board";
                tm = new checker_board_mapper(this->trgb, point_t(this->valu[2], this->valu[1], this->valu[0]));
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
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_btex";
                this->add_shader_to(&this->btex);
                break;

            case map_ctex :
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_ctex";
                this->add_shader_to(&this->ctex);
                break;
                
            case map_dtex :
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_dtex";
                this->add_shader_to(&this->dtex);
                break;
                
            case map_rtex :
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_rtex";
                this->add_shader_to(&this->rtex);
                break;

            case map_ttex :
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_ttex";
                this->add_shader_to(&this->ttex);
                break;

            case map_stex :
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_stex (not handled)";
                break;

            case map_ltex :
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_ltex (not handled)";
                break;

            case map_grad :
                BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_grad (not handled)";
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
    fp_t                    valu[3];
    fp_t                    topc;
    mapper_type_t           shader;
    mapper_of_t             map_of;
    std::uint16_t           tip;
    texture_wrapping_mode_t twrp_mode_x;
    texture_wrapping_mode_t twrp_mode_y;
};

struct image_info_t
{
};


inline static mapper_type_t pick_shader(const std::uint16_t m)
{
    switch (m)
    {
        case 0 : 
            return planar;
        case 1 : 
            return cylindrical;
        // case 2 : - Spherical
        case 3 : 
            return cubic;
        // case 4 :- Front Projection
        case 5 :
            return planar;  /* UV, must also remember to use texture UV */
        default :
            BOOST_LOG_TRIVIAL(error) << "Unknown texture: " << m;
            assert(false);
    }
}


inline void pick_procedural_shader(lwo2_texture_info_t *const current_info, const char *c)
{
    if (strcmp(c, "Fractal Noise") == 0)
    {
        c += 14;

        current_info->shader = f_noise;
        current_info->tip = from_byte_stream<std::uint32_t>(&c);
        current_info->tfp[1] = from_byte_stream<float>(&c);
        current_info->tfp[0] = from_byte_stream<float>(&c);
        BOOST_LOG_TRIVIAL(trace) << "Fractal noise: " << current_info->tip << ", " << current_info->tfp[1] << ", " << current_info->tfp[0];

    }
    else if (strcmp(c, "Checkerboard") == 0)
    {
        c += 13;
        current_info->shader = f_checker;
        BOOST_LOG_TRIVIAL(trace) << "Checkerboard";

    }
    else if (strcmp(c, "Turbulence") == 0)
    {
        c += 12;
        current_info->shader = f_noise;
        current_info->tip = from_byte_stream<std::uint32_t>(&c);
        current_info->tfp[1] = from_byte_stream<float>(&c);
        current_info->tfp[0] = from_byte_stream<float>(&c);
        BOOST_LOG_TRIVIAL(trace) << "Turbulent noise: " << current_info->tip << ", " << current_info->tfp[1] << ", " << current_info->tfp[0];

    }
    else
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown procedural shader: " << c;
        assert(false);
    }
}


inline static mapper_of_t pick_channel(const char *const c)
{
    if (strncmp(c, "COLR", 4) == 0)
    {
        return map_ctex;
    }
    else if (strncmp(c, "DIFF", 4) == 0)
    {
        return map_dtex;
    }
    else if (strncmp(c, "LUMI", 4) == 0)
    {
        return map_ltex;
    }
    else if (strncmp(c, "SPEC", 4) == 0)
    {
        return map_stex;
    }
    else if (strncmp(c, "REFL", 4) == 0)
    {
        return map_rtex;
    }
    else if (strncmp(c, "TRAN", 4) == 0)
    {
        return map_ttex;
    }
    else if (strncmp(c, "BUMP", 4) == 0)
    {
        return map_btex;
    }
    else
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown channel: " << c;
        assert(false);
    }
}


inline static texture_wrapping_mode_t get_wrapping_mode(const std::uint16_t m)
{
    switch (m)
    {
        case 0 : 
            return blank;
        case 1 : 
            return tile;
        case 2 :
            return mirror;
        case 3 : 
            return clamp;
        default :
            BOOST_LOG_TRIVIAL(error) << "Unknown wrapping mode: " << m;
            assert(false);
    }
}


inline void parse_tmap(lwo2_texture_info_t *current_info, const char **ptr, const std::uint32_t tmap_len)
{
    std::uint32_t i = 0;
    while (i < tmap_len)
    {
        const char *tmp_ptr = (*ptr) + 4;
        const std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;

        /* Texture center */
        if (strncmp((*ptr), "CNTR", 4) == 0)
        {
            current_info->tctr.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info->tctr.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info->tctr.z = from_byte_stream<fp_t>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "CNTR:" << current_info->tctr;
        }
        /* Texture size */
        else if (strncmp((*ptr), "SIZE", 4) == 0)
        {
            current_info->tsiz.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info->tsiz.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info->tsiz.z = from_byte_stream<fp_t>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "SIZE: "<< current_info->tsiz;
        }
        else if (strncmp((*ptr), "ROTA", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "ROTA (not handled)";
        }
        else if (strncmp((*ptr), "FALL", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "FALL (not handled)";
        }
        else if (strncmp((*ptr), "OREF", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "OREF (not handled)";
        }
        else if (strncmp((*ptr), "CSYS", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "CSYS (not handled)";
        }
        /* Catch unknown entities */
        else
        {
            BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< *ptr << " found in TMAP sub-chunk";
            assert(false);
        }

        (*ptr) += sec_len + 6;
        i += sec_len + 6;
    }
}


inline void parse_header(lwo2_texture_info_t *current_info, const char **ptr, const std::uint32_t header_len)
{
    std::uint32_t ord_len = strlen(*ptr) + 1;
    ord_len += (ord_len & 0x1);

    (*ptr) += ord_len;
    std::uint32_t i = 0;
    while (i < header_len)
    {
        const char *tmp_ptr = (*ptr) + 4;
        const std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;

        /* The mapped channel */
        if (strncmp((*ptr), "CHAN", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "CHAN: " << tmp_ptr;
            current_info->map_of = pick_channel(tmp_ptr);
        }
        else if (strncmp((*ptr), "OPAC", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "OPAC (not handled)";
        }
        else if (strncmp((*ptr), "ENAB", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "ENAB (not handled)";
        }
        else if (strncmp((*ptr), "NEGA", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "NEGA (not handled)";
        }
        // else if (strncmp((*ptr), "AXIS", 4) == 0)
        // {
        //     BOOST_LOG_TRIVIAL(trace) << "AXIS";
        // }
        /* Catch unknown entities */
        else
        {
            BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< *ptr << " found in IMAP sub-chunk";
            assert(false);
        }

        (*ptr) += sec_len + 6;
        i += sec_len + 6;
    }
}


inline void parse_blok(lwo2_texture_info_t *current_info, const std::map<std::uint32_t, std::string> &clips, const char **ptr, const std::uint32_t blok_len)
{
    std::uint32_t i = 0;
    current_info->shader = non;
    while (i < blok_len)
    {
        const char *tmp_ptr = (*ptr) + 4;
        const std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;

        /* Image map */
        if (strncmp((*ptr), "IMAP", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "IMAP";
            parse_header(current_info, &tmp_ptr, sec_len - 2);
        }
        /* Procedural image mapping info */
        else if (strncmp((*ptr), "PROC", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "PROC";
            parse_header(current_info, &tmp_ptr, sec_len - 2);
        }
        /* Texture mapping info */
        else if (strncmp((*ptr), "TMAP", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "TMAP";
            parse_tmap(current_info, &tmp_ptr, sec_len - 2);
        }
        else if (strncmp((*ptr), "PROJ", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "PROJ";
            current_info->shader = pick_shader(from_byte_stream<std::uint16_t>(&tmp_ptr));
        }
        /* Texture axis */
        else if (strncmp((*ptr), "AXIS", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "AXIS";
            
            /* Get the normal */
            const std::uint16_t tflg = from_byte_stream<std::uint16_t>(&tmp_ptr);
            current_info->tnorm.x = (fp_t)((tflg     ) & 0x1);
            current_info->tnorm.y = (fp_t)((tflg >> 1) & 0x1);
            current_info->tnorm.z = (fp_t)((tflg >> 2) & 0x1);
        }
        /* Texture wrapping options */
        else if (strncmp((*ptr), "WRAP", 4) == 0)
        {
            current_info->twrp_mode_x = get_wrapping_mode(from_byte_stream<std::uint16_t>(&tmp_ptr));
            current_info->twrp_mode_y = get_wrapping_mode(from_byte_stream<std::uint16_t>(&tmp_ptr));
            BOOST_LOG_TRIVIAL(trace) << "WRAP: " << current_info->twrp_mode_x << ", " << current_info->twrp_mode_y;
        }
        /* Image index */
        else if (strncmp((*ptr), "IMAG", 4) == 0)
        {
            std::uint32_t idx = parse_vx(&tmp_ptr);
            current_info->filename = clips.at(idx);
            BOOST_LOG_TRIVIAL(trace) << "IMAG: " << current_info->filename;
        }
        /* Procedural shader function */
        else if (strncmp((*ptr), "FUNC", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "FUNC: " << tmp_ptr;
            pick_procedural_shader(current_info, tmp_ptr);
        }
        else if (strncmp((*ptr), "WRPW", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "WRPW (not handled)";
        }
        else if (strncmp((*ptr), "WRPH", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "WRPH (not handled)";
        }
        else if (strncmp((*ptr), "AAST", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "AAST (not handled)";
        }
        else if (strncmp((*ptr), "PIXB", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "PIXB (not handled)";
        }
        else if (strncmp((*ptr), "VALU", 4) == 0)
        {
            current_info->valu[0] = from_byte_stream<float>(&tmp_ptr);
            if (sec_len > 0x4)
            {
                current_info->valu[1] = from_byte_stream<float>(&tmp_ptr);
                current_info->valu[2] = from_byte_stream<float>(&tmp_ptr);
            }
            BOOST_LOG_TRIVIAL(trace) << "VALU: " << current_info->valu[0] << ", " << current_info->valu[1] << ", " << current_info->valu[2];
        }
        // else if (strncmp((*ptr), "STCK", 4) == 0)
        // {
        //     BOOST_LOG_TRIVIAL(trace) << "STCK";
        // }
        else if (strncmp((*ptr), "TAMP", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "TAMP (not handled)";
        }
        else if (strncmp((*ptr), "VMAP", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "VMAP (not handled)";
        }
        else if (strncmp((*ptr), "GRAD", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "GRAD (not handled)";
            current_info->map_of = map_grad;
        }
        else if (strncmp((*ptr), "PNAM", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "PNAM (not handled)";
        }
        else if (strncmp((*ptr), "INAM", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "INAM (not handled)";
        }
        else if (strncmp((*ptr), "GRST", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "GRST (not handled)";
        }
        else if (strncmp((*ptr), "GREN", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "GREN (not handled)";
        }
        else if (strncmp((*ptr), "GRPT", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "GRPT (not handled)";
        }
        else if (strncmp((*ptr), "FKEY", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "FKEY (not handled)";
        }
        else if (strncmp((*ptr), "IKEY", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "IKEY (not handled)";
        }
        /* Catch unknown entities */
        else
        {
            BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< *ptr << " found in BLOK sub-chunk";
            assert(false);
        }

        (*ptr) += sec_len + 6;
        i += sec_len + 6;
    }

    BOOST_LOG_TRIVIAL(trace) << "BLOK parsed, adding shader";
    current_info->add_shader();
}


inline static void parse_surf(material **m, const std::map<std::uint32_t, std::string> &clips, const char **ptr, const std::uint32_t surf_len)
{
    /* Variable to hold all the parameters */
    lwo2_texture_info_t  current_info;
    ext_colour_t    rgb(255.0, 255.0, 255.0);
    fp_t            vkd = 0.0;
    fp_t            vks = 0.0;
    fp_t            s   = 0.0;
    fp_t            vt  = 0.0;
    fp_t            ri  = 1.0;
    fp_t            vr  = 0.0;
    std::uint32_t   i   = 0;
    
    while (i < surf_len)
    {
        const char *tmp_ptr = (*ptr) + 4;
        const std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;
        
        /* Base image colour */
        if (strncmp((*ptr), "COLR", 4) == 0)
        {
            rgb.r = from_byte_stream<fp_t>(&tmp_ptr) * 255.0;
            rgb.g = from_byte_stream<fp_t>(&tmp_ptr) * 255.0;
            rgb.b = from_byte_stream<fp_t>(&tmp_ptr) * 255.0;
            BOOST_LOG_TRIVIAL(trace) << "COLR: "<< rgb.r << ", " << rgb.g << ", " << rgb.b;
        }
        /* Floating point percentage diffuse co-efficient */
        else if (strncmp((*ptr), "DIFF", 4) == 0)
        {
            vkd = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "DIFF: " << vkd;
        }
        /* Floating point percentage specular co-efficient */
        else if (strncmp((*ptr), "SPEC", 4) == 0)
        {
            vks = from_byte_stream<fp_t>(&tmp_ptr) / 255.0;
            BOOST_LOG_TRIVIAL(trace) << "SPEC: " << vks;
        }
        else if (strncmp((*ptr), "GLOS", 4) == 0)
        {
            s = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "GLOS: " << s;
        }
        /* Floating point percentage transmittance co-efficient. */
        else if (strncmp((*ptr), "TRAN", 4) == 0)
        {
            vt = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TRAN: " << vt;
        }
        /* Floating point percentage reflectance co-efficient. */
        else if (strncmp((*ptr), "REFL", 4) == 0)
        {
            vr = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "REFL: " << vr;
        }
        /* Refractive index */
        else if (strncmp((*ptr), "RIND", 4) == 0)
        {
            ri = from_byte_stream<fp_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "RIND: " << ri;
        }
        /* Floating point percentage luminance co-efficient */
        else if (strncmp((*ptr), "LUMI", 4) == 0)
        {
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "LUMI (not handled)";
        }
        /* Texture mapper or shader block */
        else if (strncmp((*ptr), "BLOK", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "BLOK";
            parse_blok(&current_info, clips, &tmp_ptr, sec_len);
        }
        else if (strncmp((*ptr), "CLRH", 4) == 0)
        {
            /* The blending of specular highlight colour between the object and lights colour */
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "CLRH (not handled)";
        }
        else if (strncmp((*ptr), "TRNL", 4) == 0)
        {
            /* Translucency */
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "TRNL (not handled)";
        }
        else if (strncmp((*ptr), "TIMG", 4) == 0)
        {
            /* Transparency image name */
            BOOST_LOG_TRIVIAL(trace) << "TIMG: (not handled)";
            /*std::uint32_t idx = */ parse_vx(&tmp_ptr);
        }
        else if (strncmp((*ptr), "RIMG", 4) == 0)
        {
            /* Reflection image name */
            BOOST_LOG_TRIVIAL(trace) << "RIMG: (not handled)";
            /*std::uint32_t idx = */ parse_vx(&tmp_ptr);
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
        else if (strncmp((*ptr), "BUMP", 4) == 0)
        {
            /* Bump height scaling */
            tmp_ptr += 4;
            BOOST_LOG_TRIVIAL(trace) << "BUMP (not handled)";
        }
        else if (strncmp((*ptr), "SIDE", 4) == 0)
        {
            /* Sidedness */
            tmp_ptr += 2;
            BOOST_LOG_TRIVIAL(trace) << "SIDE (not handled)";
        }
        else if (strncmp((*ptr), "RFOP", 4) == 0)
        {
            /* Reflection options, ignored because we always just raytrace */
            tmp_ptr += 2;
            BOOST_LOG_TRIVIAL(trace) << "RFOP (ignored)";
        }
        else if (strncmp((*ptr), "TROP", 4) == 0)
        {
            /* Transparency options, ignored because we always just raytrace */
            tmp_ptr += 2;
            BOOST_LOG_TRIVIAL(trace) << "TROP (ignored)";
        }
        /* Catch unknown entities */
        else
        {
            BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< *ptr << " found in SURF chunk";
            assert(false);
        }

        /* Envelop */
        if ((strncmp((*ptr), "SMAN", 4) != 0) && (strncmp((*ptr), "RFOP", 4) != 0) && (strncmp((*ptr), "TROP", 4) != 0) && (strncmp((*ptr), "SIDE", 4) != 0) &&
            (strncmp((*ptr), "BLOK", 4) != 0) && (strncmp((*ptr), "RIMG", 4) != 0) && (strncmp((*ptr), "TIMG", 4) != 0))
        {
            const std::uint16_t envelop_id = from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "Envelop: " << envelop_id;
            assert(envelop_id == 0);   /* The envelop must be null */
        }

        (*ptr) += sec_len + 6;
        i += sec_len + 6;
        
    }

    /* Create the new material and return */
    *m = new mapper_shader(current_info.ctex, current_info.dtex, current_info.rtex, current_info.ttex, rgb, vkd, vks, s, vt, ri, vr);
}


void parse_surf(list<material *> &m, const std::string &p, const std::map<std::string, std::uint16_t> &tag_map, material **surf_materials, const char *at)
{
    /* Check this is the POLS chunk */
    check_for_chunk(&at, "POLS", 4, __LINE__);
    
    /* Skip the POLS chunk */
    const std::uint32_t pols_len = from_byte_stream<std::uint32_t>(&at);
    at += pols_len;

    /* Check this is the PTAG chunk */
    check_for_chunk(&at, "PTAG", 4, __LINE__);
    
    /* Skip the POLS chunk */
    const std::uint32_t ptag_len = from_byte_stream<std::uint32_t>(&at);
    at += ptag_len;

    /* Skip the optional VMAD chunk */
    while (strncmp(at, "VMAD", 4) == 0)
    {
        at += 4;
        const std::uint32_t vmad_len = from_byte_stream<std::uint32_t>(&at);
        at += vmad_len;
    }

    /* Parse CLIP chunks */
    std::map<std::uint32_t, std::string> clips;
    while (strncmp(at, "CLIP", 4) == 0)
    {
        BOOST_LOG_TRIVIAL(trace) << "CLIP";

        at += 4;
        const std::uint32_t clip_len = from_byte_stream<std::uint32_t>(&at);
        const char *tmp_ptr = at;
        at += clip_len;

        const std::uint32_t clip_idx = from_byte_stream<std::uint32_t>(&tmp_ptr);
        while (tmp_ptr < at)
        {
            if (strncmp(tmp_ptr, "STIL", 4) == 0)
            {
                tmp_ptr += 4;
                const std::uint16_t stil_len = from_byte_stream<std::uint16_t>(&tmp_ptr);
                BOOST_LOG_TRIVIAL(trace) << "STIL: " << (p + tmp_ptr);
                clips.emplace(clip_idx, p + tmp_ptr);
                tmp_ptr += stil_len;
            }
            else if (strncmp(tmp_ptr, "BRIT", 4) == 0)
            {
                /* Brightness adjust for above still */
                tmp_ptr += 12;
                BOOST_LOG_TRIVIAL(trace) << "BRIT (not handled)";
            }
            else if (strncmp(tmp_ptr, "CONT", 4) == 0)
            {
                /* Contrast adjust for above still */
                tmp_ptr += 12;
                BOOST_LOG_TRIVIAL(trace) << "CONT (not handled)";
            }
            else if (strncmp(tmp_ptr, "SATR", 4) == 0)
            {
                /* Contrast adjust for above still */
                tmp_ptr += 12;
                BOOST_LOG_TRIVIAL(trace) << "SATR (not handled)";
            }
            else if (strncmp(tmp_ptr, "GAMM", 4) == 0)
            {
                /* Contrast adjust for above still */
                tmp_ptr += 12;
                BOOST_LOG_TRIVIAL(trace) << "GAMM (not handled)";
            }
            else if (strncmp(tmp_ptr, "NEGA", 4) == 0)
            {
                /* Contrast adjust for above still */
                tmp_ptr += 8;
                BOOST_LOG_TRIVIAL(trace) << "NEGA (not handled)";
            }
            else
            {
                BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< tmp_ptr << " found in CLIP chunk";
                assert(false);
            }
        }
    }
    
    /* Check the SURF chunk has been found */
    check_for_chunk(&at, "SURF", 4, __LINE__);

    for (std::uint32_t i = 0; i < tag_map.size(); ++i)
    {
        std::uint32_t surf_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "Parsing material: " << at << ". Length: " << surf_len;
        if (tag_map.find(at) == tag_map.end())
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
        
        parse_surf(&surf_materials[i], clips, &at, surf_len - srf_len - source_len);

        at += 4;
    }

    /* Copy to materials */
    for (std::uint32_t i = 0; i < tag_map.size(); ++i)
    {
        m.push_back(surf_materials[i]);
    }
}

void parse_pols(light_list &l, primitive_list &e, const std::map<std::string, std::uint16_t> &tag_map, material *const *const surf_materials, const point_t *const all_points, const char **at, const char *const buffer, const std::uint32_t nr_of_verts)
{
    /* Check this is the POLS chunk */
    check_for_chunk(at, "POLS", 4, __LINE__);
    const std::uint32_t pols_bytes = from_byte_stream<std::uint32_t>(at);
    const char *ptags_at = (*at) + pols_bytes;
    const char *ptags_start = ptags_at;

    /* Check for FACE chunk */
    check_for_chunk(at, "FACE", 4, __LINE__);

    /* Find PTAGS */
    check_for_chunk(&ptags_at, "PTAG", 4, __LINE__);
    const std::uint32_t ptag_len = from_byte_stream<std::uint32_t>(&ptags_at);

    /* Check they are surface tags */
    check_for_chunk(&ptags_at, "SURF", 4, __LINE__);

    /* Gather all the polygons */
    std::uint32_t pol = 0;
    vector<point_t> pol_vert;
    std::uint16_t vert_this_pol = 0;
    std::uint32_t num_of_surfs = tag_map.size();
    while ((*at) < ptags_start)
    {
        vert_this_pol = from_byte_stream<std::uint16_t>(at);
        for (std::uint32_t j = 0; j < vert_this_pol; j++)
        {
            const std::uint32_t vert_num = parse_vx(at);
            assert(vert_num < nr_of_verts);
            
            pol_vert.push_back(all_points[vert_num]);
        }
        
        /* Parse the material to use */
        const std::uint32_t ptag_pol = parse_vx(&ptags_at);
        const std::uint16_t mat_num = from_byte_stream<std::uint16_t>(&ptags_at);
        assert((ptag_pol == pol) || !"Error: ptag is not for this polygon");

        /* Range check the material */
        if ((std::uint32_t)mat_num > num_of_surfs)
        {
            BOOST_LOG_TRIVIAL(error) << "Material " << hex << mat_num << " out of range at " << (std::uint32_t)((*at) - buffer) << dec;
            assert(false);
        }

        /* Create the polygon */
        if (vert_this_pol > 2)
        {
            face_to_triangles(&e, &l, pol_vert, surf_materials[mat_num], false);
        }
        
        /* Clean up */
        pol_vert.clear();
        ++pol;
    }

    /* Check for and skip ptag */
    check_for_chunk(at, "PTAG", 4, __LINE__);
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
    check_for_chunk(&at, "TAGS", 4, __LINE__);

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
    while (strncmp(at, "LAYR", 4) == 0)
    {
        at += 4;
        const std::uint32_t layr_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "LAYR (not handled): " << layr_len;
        at += layr_len;
    }

    /* Check the first chunk is PNTS */
    check_for_chunk(&at, "PNTS", 4, __LINE__);
    
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
        const std::uint32_t bbox_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "BBOX (not handled): " << bbox_len;
        at += bbox_len;
    }

    /* Check for optional VMAP chunk */
    while (strncmp(at, "VMAP", 4) == 0)
    {
        at += 4;
        const std::uint32_t vmap_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "VMAP (not handled): " << vmap_len;
        at += vmap_len;
    }

    /* Look for the SURF chunks */
    material **surf_materials;
    surf_materials = new material *[tag_map.size()];
    parse_surf(m, p, tag_map, surf_materials, at);
    
    /* Parse POLS */
    parse_pols(l, e, tag_map, surf_materials, all_points, &at, begin, nr_of_verts);

    /* Check for optional VMAD chunk */
    while (strncmp(at, "VMAD", 4) == 0)
    {
        at += 4;
        const std::uint32_t vmad_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "VMAD (not handled): " << vmad_len;
        at += vmad_len;
    }

    /* Check for and skip CLIP chunks */
    BOOST_LOG_TRIVIAL(trace) << "skipping CLIP";
    while (strncmp(at, "CLIP", 4) == 0)
    {
        at += 4;
        const std::uint32_t clip_len = from_byte_stream<std::uint32_t>(&at);
        BOOST_LOG_TRIVIAL(trace) << "CLIP " << clip_len;
        at += clip_len;
    }

    /* Check for and skip SURF chunk */
    for (std::uint32_t i = 0; i < tag_map.size(); ++i)
    {
        check_for_chunk(&at, "SURF", 4, __LINE__);
        const std::uint32_t surf_len = from_byte_stream<std::uint32_t>(&at);
        at += surf_len;
    }
    
    /* Clean up */
    delete [] all_points;
    delete [] surf_materials;

    return at;
}
}; /* namespace raptor_raytracer */
