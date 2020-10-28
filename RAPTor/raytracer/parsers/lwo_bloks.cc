/* Standard headers*/

/* Boost headers */

/* Common headers*/
#include "logging.h"

/* Raytracer headers*/
#include "parser_common.h"
#include "lwo_parser.h"
#include "lwo_clip.h"
#include "lwo_bloks.h"

/* Texture mappers */
#include "checker_board_mapper.h"
#include "cubic_mapper.h"
#include "cylindrical_mapper.h"
#include "planar_mapper.h"
#include "perlin_noise_3d_mapper.h"


namespace raptor_raytracer
{
std::string lwo_bloks::parse(const std::map<std::uint32_t, lwo_clip *> &clips, const char **ptr, const std::uint32_t blok_len)
{
    std::string vmap;
    bool enabled = false;
    std::uint32_t i = 0;
    _shader = mapper_type_t::non;
    while (i < blok_len)
    {
        const char *tmp_ptr = (*ptr) + 4;
        const std::uint16_t sec_len = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;

        /* Image map */
        if (strncmp((*ptr), "IMAP", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "IMAP";
            enabled = parse_header(&tmp_ptr, sec_len - 2);
        }
        /* Procedural image mapping info */
        else if (strncmp((*ptr), "PROC", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "PROC";
            enabled = parse_header(&tmp_ptr, sec_len - 2);
        }
        /* Gradient texture */
        else if (strncmp((*ptr), "GRAD", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "GRAD";
            _shader = mapper_type_t::map_grad;
            enabled = parse_header(&tmp_ptr, sec_len - 2);
        }
        /* Texture mapping info */
        else if (strncmp((*ptr), "TMAP", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "TMAP";
            parse_tmap(&tmp_ptr, sec_len - 2);
        }
        else if (strncmp((*ptr), "PROJ", 4) == 0)
        {
            const std::uint16_t proj = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "PROJ: " << proj;
            _shader = pick_shader(proj);
        }
        /* Texture axis */
        else if (strncmp((*ptr), "AXIS", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "AXIS";
            
            /* Get the normal */
            const std::uint16_t tflg = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            _tnorm.x = (tflg == 0) ? 1.0f : 0.0f;
            _tnorm.y = (tflg == 1) ? 1.0f : 0.0f;
            _tnorm.z = (tflg == 2) ? 1.0f : 0.0f;
        }
        /* Texture wrapping options */
        else if (strncmp((*ptr), "WRAP", 4) == 0)
        {
            _twrp_mode_x = get_wrapping_mode(raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr));
            _twrp_mode_y = get_wrapping_mode(raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr));
            BOOST_LOG_TRIVIAL(trace) << "WRAP: " << static_cast<int>(_twrp_mode_x) << ", " << static_cast<int>(_twrp_mode_y);
        }
        /* Image index */
        else if (strncmp((*ptr), "IMAG", 4) == 0)
        {
            std::uint32_t idx = parse_vx(&tmp_ptr);
            _clip = clips.at(idx);
            BOOST_LOG_TRIVIAL(trace) << "IMAG: " << _clip->filename();
        }
        /* Procedural shader function */
        else if (strncmp((*ptr), "FUNC", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "FUNC: " << tmp_ptr;
            pick_procedural_shader(tmp_ptr);
        }
        else if (strncmp((*ptr), "WRPW", 4) == 0)
        {
            _wrpw = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "WRPW: " << _wrpw;
        }
        else if (strncmp((*ptr), "WRPH", 4) == 0)
        {
            _wrph = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "WRPH (not handled): " << _wrph;
        }
        else if (strncmp((*ptr), "AAST", 4) == 0)
        {
            const std::uint16_t flag = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            const float stren = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "AAST (not handled): " << flag << ", strength: " << stren;
        }
        else if (strncmp((*ptr), "PIXB", 4) == 0)
        {
            const std::uint16_t pixb = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "PIXB (not handled): " << pixb;
        }
        else if (strncmp((*ptr), "VALU", 4) == 0)
        {
            _valu[0] = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "VALU 0: " << _valu[0];
            if (sec_len > 4)
            {
                _valu[1] = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                BOOST_LOG_TRIVIAL(trace) << "VALU 1: " << _valu[1];
            }

            if (sec_len > 8)
            {
                _valu[2] = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                BOOST_LOG_TRIVIAL(trace) << "VALU 2: " << _valu[2];
            }
        }
        // else if (strncmp((*ptr), "STCK", 4) == 0)
        // {
        //     BOOST_LOG_TRIVIAL(trace) << "STCK";
        // }
        else if (strncmp((*ptr), "TAMP", 4) == 0)
        {
            _tamp = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TAMP: " << _tamp;
        }
        else if (strncmp((*ptr), "VMAP", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "VMAP: " << tmp_ptr;
            vmap = tmp_ptr;
        }
        else if (strncmp((*ptr), "PNAM", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "PNAM: " << tmp_ptr;
            pick_gradient_shader(tmp_ptr);
        }
        else if (strncmp((*ptr), "INAM", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(warning) << "INAM (not handled): " << tmp_ptr;
            assert((strlen(tmp_ptr) == 0) || (strncmp(tmp_ptr, "(none)", 6) == 0));
        }
        else if (strncmp((*ptr), "GRST", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(info) << "GRST (ignored)";
        }
        else if (strncmp((*ptr), "GREN", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(info) << "GREN (ignored)";
        }
        else if (strncmp((*ptr), "GRPT", 4) == 0)
        {
            const std::uint16_t grpt = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "GRPT: " << grpt;
            assert(grpt == 0);
        }
        else if (strncmp((*ptr), "FKEY", 4) == 0)
        {
            while (tmp_ptr < ((*ptr) + sec_len + 6))
            {
                const float key = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                _fkey_key.push_back(key);

                ext_colour_t rgb;
                rgb.r = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                rgb.g = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                rgb.b = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                rgb *= 255.0f;

                const float alpha = raptor_parsers::from_byte_stream<float>(&tmp_ptr);

                _fkey_value.emplace_back(rgb, alpha);
                BOOST_LOG_TRIVIAL(trace) << "FKEY, key: " << key << ", colour: " << rgb << ", alpha: " << alpha;
            }
        }
        else if (strncmp((*ptr), "IKEY", 4) == 0)
        {
            while (tmp_ptr < ((*ptr) + sec_len + 6))
            {
                const std::uint16_t ikey = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
                BOOST_LOG_TRIVIAL(trace) << "IKEY: " << ikey;
                _ikey.push_back(static_cast<grad_interpolation_t>(ikey));
            }
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

    if (enabled)
    {
        BOOST_LOG_TRIVIAL(trace) << "BLOK parsed, adding shader";
        add_shader();
    }
    else
    {
        BOOST_LOG_TRIVIAL(trace) << "BLOK parsed, not enabled";
    }

    return vmap;
}

void lwo_bloks::add_shader_to(std::vector<texture_mapper *>  *const t)
{
    texture_mapper *tm = nullptr;
    switch (_shader)
    {
        case mapper_type_t::f_noise :
            BOOST_LOG_TRIVIAL(trace) << "Type of shader is f_noise";
            tm = new perlin_noise_3d_mapper(ext_colour_t(_valu[0] * 255.0f, _valu[1] * 255.0f, _valu[2] * 255.0f), _tfp[1], _tfp[0], _tip, 4);
            break;
            
        case mapper_type_t::cylindrical  :
            BOOST_LOG_TRIVIAL(trace) << "Type of shader is cylindrical";
            tm = new cylindrical_mapper(_clip->image(), _tctr, _tnorm, _tsiz, 0.3f, _clip->image_height(), _clip->image_width(), _clip->colour_parts(), _wrpw);
            break;

        case mapper_type_t::planar  :
            BOOST_LOG_TRIVIAL(trace) << "Type of shader is planar";
            tm = new planar_mapper(_clip->image(), _tctr, _tnorm, _tsiz, _clip->colour_parts(), _clip->image_width(), _clip->image_height(), _twrp_mode_x, _twrp_mode_y);
            break;

        case mapper_type_t::cubic  :
            BOOST_LOG_TRIVIAL(trace) << "Type of shader is cubic";
            tm = new cubic_mapper(_clip->image(), _tctr, _tnorm, _tsiz, _twrp_mode_x, _twrp_mode_y, _clip->colour_parts(), _clip->image_width(), _clip->image_height());
            break;

        case mapper_type_t::f_checker :
            BOOST_LOG_TRIVIAL(trace) << "Type of shader is checker board";
            tm = new checker_board_mapper(ext_colour_t(_valu[0] * 255.0f, _valu[1] * 255.0f, _valu[2] * 255.0f), _tctr, _tsiz);
            break;

        case mapper_type_t::f_honeycomb :
            BOOST_LOG_TRIVIAL(warning) << "Type of shader is honey comb (not handled)";
            break;

        case mapper_type_t::map_grad :
            BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_grad";
            tm = new gradient_mapper(_fkey_key, _fkey_value, _ikey, _grad_of);
            break;

        default :
            assert(false);
            break;
    }

    /* Set if we want the inverse image */
    if (tm != nullptr)
    {
        t->push_back(tm);
        tm->inverse(_nega);
    }

    /* Set fall off if asked for */
    if ((_fall_type != falloff_type_t::none) && (tm != nullptr))
    {
        tm->falloff(new mapper_falloff(_tctr, _fall_grad, _fall_type));
    }
}

void lwo_bloks::add_shader()
{
    switch (_map_of)
    {
        case mapper_of_t::map_btex :
        {
            /* Scale image or procedural texture colour for tamp before adding */
            lwo_clip *orig_clip = nullptr;
            if (_clip != nullptr)
            {
                orig_clip = _clip;
                _clip = orig_clip->scaled_clone(_tamp * (1.0f / 255.0f));
            }
            else
            {
                _valu[0] *= _tamp * (1.0f / 255.0f);
                _valu[1] *= _tamp * (1.0f / 255.0f);
                _valu[2] *= _tamp * (1.0f / 255.0f);
            }

            BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_btex";
            add_shader_to(&_btex);

            /* Delete the cloned clip */
            if (_clip != nullptr)
            {
                delete _clip;
                _clip = orig_clip;
            }
            break;
        }
        case mapper_of_t::map_ctex :
            BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_ctex";
            add_shader_to(&_ctex);
            break;
            
        case mapper_of_t::map_dtex :
            BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_dtex";
            add_shader_to(&_dtex);
            break;
            
        case mapper_of_t::map_rtex :
            BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_rtex";
            add_shader_to(&_rtex);
            break;

        case mapper_of_t::map_ttex :
            BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_ttex";
            add_shader_to(&_ttex);
            break;

        case mapper_of_t::map_stex :
            BOOST_LOG_TRIVIAL(trace) << "Adding shader to map_stex";
            add_shader_to(&_stex);
            break;

        case mapper_of_t::map_ltex :
            BOOST_LOG_TRIVIAL(warning) << "Adding shader to map_ltex (not handled)";
            break;
            
        default :
            assert(false);
            break;
    }
}

inline mapper_type_t lwo_bloks::pick_shader(const std::uint16_t m)
{
    switch (m)
    {
        case 0 : 
            return mapper_type_t::planar;
        case 1 : 
            return mapper_type_t::cylindrical;
        // case 2 : - Spherical
        case 3 : 
            return mapper_type_t::cubic;
        // case 4 :- Front Projection
        case 5 :
            return mapper_type_t::planar;  /* UV, must also remember to use texture UV */
        default :
            BOOST_LOG_TRIVIAL(error) << "Unknown texture: " << m;
            assert(false);
    }

    return mapper_type_t::planar;
}


inline void lwo_bloks::pick_procedural_shader(const char *c)
{
    if (strcmp(c, "Fractal Noise") == 0)
    {
        c += 14;

        _shader = mapper_type_t::f_noise;
        _tip = raptor_parsers::from_byte_stream<std::uint32_t>(&c);
        _tfp[1] = raptor_parsers::from_byte_stream<float>(&c);
        _tfp[0] = raptor_parsers::from_byte_stream<float>(&c);
        BOOST_LOG_TRIVIAL(trace) << "Fractal noise: " << _tip << ", " << _tfp[1] << ", " << _tfp[0];

    }
    else if (strcmp(c, "Checkerboard") == 0)
    {
        c += 13;
        _shader = mapper_type_t::f_checker;
        BOOST_LOG_TRIVIAL(trace) << "Checkerboard";

    }
    else if (strcmp(c, "Turbulence") == 0)
    {
        c += 12;
        _shader = mapper_type_t::f_noise;
        _tip = raptor_parsers::from_byte_stream<std::uint32_t>(&c);
        _tfp[1] = raptor_parsers::from_byte_stream<float>(&c);
        _tfp[0] = raptor_parsers::from_byte_stream<float>(&c);
        BOOST_LOG_TRIVIAL(trace) << "Turbulent noise: " << _tip << ", " << _tfp[1] << ", " << _tfp[0];

    }
    else if (strcmp(c, "Honeycomb") == 0)
    {
        c += 10;
        _shader = mapper_type_t::f_honeycomb;
        BOOST_LOG_TRIVIAL(trace) << "Honeycomb";
    }
    else
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown procedural shader: " << c;
        assert(false);
    }
}

inline void lwo_bloks::pick_gradient_shader(const char *c)
{
    if (strcmp(c, "Previous Layer") == 0)
    {
        _grad_of = grad_of_t::previous_layer;
    }
    else if (strcmp(c, "Bump") == 0)
    {
        _grad_of = grad_of_t::bump;
    }
    else if (strcmp(c, "Slope") == 0)
    {
        _grad_of = grad_of_t::slope;
    }
    else if (strcmp(c, "Incidence Angle") == 0)
    {
        _grad_of = grad_of_t::incidence_angle;
    }
    else if (strcmp(c, "Light Incidence") == 0)
    {
        _grad_of = grad_of_t::light_incidence;
    }
    else if (strcmp(c, "Distance to Camera") == 0)
    {
        _grad_of = grad_of_t::distance_to_camera;
    }
    else if (strcmp(c, "Distance to Object") == 0)
    {
        _grad_of = grad_of_t::distance_to_object;
    }
    else if (strcmp(c, "X Distance to Object") == 0)
    {
        _grad_of = grad_of_t::x_distance_to_object;
    }
    else if (strcmp(c, "Y Distance to Object") == 0)
    {
        _grad_of = grad_of_t::y_distance_to_object;
    }
    else if (strcmp(c, "Z Distance to Object") == 0)
    {
        _grad_of = grad_of_t::z_distance_to_object;
    }
    else if (strcmp(c, "Weight Map") == 0)
    {
        _grad_of = grad_of_t::weight_map;
    }
    else
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown gradient shader: " << c;
        assert(false);
    }
}


inline mapper_of_t lwo_bloks::pick_channel(const char *const c)
{
    if (strncmp(c, "COLR", 4) == 0)
    {
        return mapper_of_t::map_ctex;
    }
    else if (strncmp(c, "DIFF", 4) == 0)
    {
        return mapper_of_t::map_dtex;
    }
    else if (strncmp(c, "LUMI", 4) == 0)
    {
        return mapper_of_t::map_ltex;
    }
    else if (strncmp(c, "SPEC", 4) == 0)
    {
        return mapper_of_t::map_stex;
    }
    else if (strncmp(c, "REFL", 4) == 0)
    {
        return mapper_of_t::map_rtex;
    }
    else if (strncmp(c, "TRAN", 4) == 0)
    {
        return mapper_of_t::map_ttex;
    }
    else if (strncmp(c, "BUMP", 4) == 0)
    {
        return mapper_of_t::map_btex;
    }
    else
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown channel: " << c;
        assert(false);
    }

    return mapper_of_t::map_ctex;
}


inline texture_wrapping_mode_t lwo_bloks::get_wrapping_mode(const std::uint16_t m)
{
    switch (m)
    {
        case 0 : 
            return texture_wrapping_mode_t::blank;
        case 1 : 
            return texture_wrapping_mode_t::tile;
        case 2 :
            return texture_wrapping_mode_t::mirror;
        case 3 : 
            return texture_wrapping_mode_t::clamp;
        default :
            BOOST_LOG_TRIVIAL(error) << "Unknown wrapping mode: " << m;
            assert(false);
    }

    return texture_wrapping_mode_t::blank;
}


inline void lwo_bloks::parse_tmap(const char **ptr, const std::uint32_t tmap_len)
{
    std::uint32_t i = 0;
    while (i < tmap_len)
    {
        const char *tmp_ptr = (*ptr) + 4;
        const std::uint16_t sec_len = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;

        /* Texture center */
        if (strncmp((*ptr), "CNTR", 4) == 0)
        {
            _tctr.x = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            _tctr.y = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            _tctr.z = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "CNTR:" << _tctr;
        }
        /* Texture size */
        else if (strncmp((*ptr), "SIZE", 4) == 0)
        {
            _tsiz.x = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            _tsiz.y = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            _tsiz.z = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            
            BOOST_LOG_TRIVIAL(trace) << "SIZE: "<< _tsiz;
        }
        else if (strncmp((*ptr), "ROTA", 4) == 0)
        {
            const float rota_x = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            const float rota_y = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            const float rota_z = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "ROTA: " << rota_x << ", " << rota_y << ", " << rota_z;
            assert(rota_x == 0.0f);
            assert(rota_y == 0.0f);
            assert(rota_z == 0.0f);
        }
        else if (strncmp((*ptr), "FALL", 4) == 0)
        {
            _fall_type = static_cast<falloff_type_t>(raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr));
            _fall_grad.x = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            _fall_grad.y = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            _fall_grad.z = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "FALL: " << static_cast<int>(_fall_type) << ", in: " << _fall_grad;
        }
        else if (strncmp((*ptr), "OREF", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "OREF: " << tmp_ptr;
            assert((strlen(tmp_ptr) == 0) || (strncmp(tmp_ptr, "(none)", 6) == 0));
        }
        else if (strncmp((*ptr), "CSYS", 4) == 0)
        {
            const std::uint16_t csys = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "CSYS (not handled): " << csys;
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


inline bool lwo_bloks::parse_header(const char **ptr, const std::uint32_t header_len)
{
    bool enabled = true;
    std::uint32_t ord_len = strlen(*ptr) + 1;
    ord_len += (ord_len & 0x1);

    (*ptr) += ord_len;
    std::uint32_t i = 0;
    while (i < header_len)
    {
        const char *tmp_ptr = (*ptr) + 4;
        const std::uint16_t sec_len = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << *ptr << " with length " << sec_len;

        /* The mapped channel */
        if (strncmp((*ptr), "CHAN", 4) == 0)
        {
            BOOST_LOG_TRIVIAL(trace) << "CHAN: " << tmp_ptr;
            _map_of = pick_channel(tmp_ptr);
        }
        else if (strncmp((*ptr), "OPAC", 4) == 0)
        {
            const std::uint16_t opac = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            const float value = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "OPAC (not handled): " << opac << ", value: " << value;
        }
        else if (strncmp((*ptr), "ENAB", 4) == 0)
        {
            const std::uint16_t enab = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            enabled = (enab != 0);
            BOOST_LOG_TRIVIAL(trace) << "ENAB: " << enab;
        }
        else if (strncmp((*ptr), "NEGA", 4) == 0)
        {
            const std::uint16_t nega = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
            _nega = (nega != 0);
            BOOST_LOG_TRIVIAL(trace) << "NEGA: " << nega;
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

    return enabled;
}
}; /* namespace raptor_raytracer */
