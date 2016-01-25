/* Standard headers */

/* Boost headers */

/* Common headers */
#include "logging.h"

/* Raytracer headers */
#include "mapper_shader.h"
#include "mapper_falloff.h"
#include "parser_common.h"
#include "lwo_parser.h"
#include "lwo_surf.h"
#include "lwo_bloks.h"


namespace raptor_raytracer
{
bool lwo_surf::parse(const std::map<std::uint32_t, lwo_clip *> &clips, const char *data, const int size, const int mat_num)
{
    /* Variable to hold all the parameters */
    lwo_bloks bloks;
    ext_colour_t    rgb(255.0f, 255.0f, 255.0f);
    float           vkd     = 0.0f;
    float           vks     = 0.0f;
    float           s       = 64.0f;
    float           vt      = 0.0f;
    float           ri      = 1.0f;
    float           vr      = 0.0f;
    float           clrf    = 0.0f;
    float           clrh    = 0.0f;
    float           bump    = 1.0f;
    
    const char *tmp_ptr = data;
    while (tmp_ptr < (data + size))
    {
        const char *sec_ptr = tmp_ptr;
        const char *len_ptr = tmp_ptr + 4;
        const std::uint16_t sec_len = from_byte_stream<std::uint16_t>(&len_ptr);
        BOOST_LOG_TRIVIAL(trace) << "Parsing: " << tmp_ptr << " with length " << sec_len;
        
        /* Base image colour */
        if (strncmp(tmp_ptr, "COLR", 4) == 0)
        {
            tmp_ptr += 6;
            rgb.r = from_byte_stream<float>(&tmp_ptr) * 255.0f;
            rgb.g = from_byte_stream<float>(&tmp_ptr) * 255.0f;
            rgb.b = from_byte_stream<float>(&tmp_ptr) * 255.0f;
            BOOST_LOG_TRIVIAL(trace) << "COLR: "<< rgb.r << ", " << rgb.g << ", " << rgb.b;
        }
        /* Floating point percentage diffuse co-efficient */
        else if (strncmp(tmp_ptr, "DIFF", 4) == 0)
        {
            tmp_ptr += 6;
            vkd = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "DIFF: " << vkd;
        }
        /* Floating point percentage specular co-efficient */
        else if (strncmp(tmp_ptr, "SPEC", 4) == 0)
        {
            tmp_ptr += 6;
            vks = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "SPEC: " << vks;
        }
        else if (strncmp(tmp_ptr, "GLOS", 4) == 0)
        {
            tmp_ptr += 6;
            s = pow(2.0f, ((10.0f * from_byte_stream<float>(&tmp_ptr)) + 2.0f));
            BOOST_LOG_TRIVIAL(trace) << "GLOS: " << s;
        }
        /* Floating point percentage transmittance co-efficient. */
        else if (strncmp(tmp_ptr, "TRAN", 4) == 0)
        {
            tmp_ptr += 6;
            vt = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "TRAN: " << vt;
        }
        /* Floating point percentage reflectance co-efficient. */
        else if (strncmp(tmp_ptr, "REFL", 4) == 0)
        {
            tmp_ptr += 6;
            vr = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "REFL: " << vr;
        }
        /* Refractive index */
        else if (strncmp(tmp_ptr, "RIND", 4) == 0)
        {
            tmp_ptr += 6;
            ri = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "RIND: " << ri;
        }
        /* Floating point percentage luminance co-efficient */
        else if (strncmp(tmp_ptr, "LUMI", 4) == 0)
        {
            tmp_ptr += 6;
            const float lumi = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "LUMI (not handled): " << lumi;
        }
        /* Floating point percentage luminance co-efficient */
        else if (strncmp(tmp_ptr, "FLAG", 4) == 0)
        {
            tmp_ptr += 6;
            const std::uint16_t flag = from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "FLAG (not handled): " << flag;
        }
        /* Texture mapper or shader block */
        else if (strncmp(tmp_ptr, "BLOK", 4) == 0)
        {
            tmp_ptr += 6;
            BOOST_LOG_TRIVIAL(trace) << "BLOK";
            const std::string blok_vmap(bloks.parse(clips, &tmp_ptr, sec_len));
            if (!_vmap.empty() && (_vmap != blok_vmap))
            {
                BOOST_LOG_TRIVIAL(trace) << "Multiple VMAP used on the same surface";
                return false;
            }

            if (!blok_vmap.empty())
            {
                _vmap = blok_vmap;
                BOOST_LOG_TRIVIAL(trace) << "VMAP set to: " << _vmap << ", mat_num: " << mat_num;
            }
        }
        else if (strncmp(tmp_ptr, "TRNL", 4) == 0)
        {
            /* Translucency */
            tmp_ptr += 6;
            const float trnl = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "TRNL (not handled): " << trnl;
        }
        else if (strncmp(tmp_ptr, "TIMG", 4) == 0)
        {
            /* Transparency image name */
            tmp_ptr += 6;
            BOOST_LOG_TRIVIAL(warning) << "TIMG: (not handled)";
            /*std::uint32_t idx = */ parse_vx(&tmp_ptr);
        }
        else if (strncmp(tmp_ptr, "RIMG", 4) == 0)
        {
            /* Reflection image name */
            tmp_ptr += 6;
            const std::uint32_t idx =  parse_vx(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "RIMG: (not handled): " << idx;
        }
        else if (strncmp(tmp_ptr, "SMAN", 4) == 0)
        {
            /* Maximum smooting angle */
            tmp_ptr += 6;
            _smoothing_threshold = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "SMAN: " << _smoothing_threshold;
        }
        else if (strncmp(tmp_ptr, "BUMP", 4) == 0)
        {
            /* Bump height scaling */
            tmp_ptr += 6;
            bump = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "BUMP: " << bump;
        }
        else if (strncmp(tmp_ptr, "SHRP", 4) == 0)
        {
            /* Sharpness of shadow cutoff, ignored because this should be determined by geometry */
            tmp_ptr += 6;
            const float shrp = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(info) << "SHRP (ignored): " << shrp;
        }
        else if (strncmp(tmp_ptr, "CLRF", 4) == 0)
        {
            /* The blending of transparency colour between the object and lights colour */
            tmp_ptr += 6;
            clrf = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "CLRF: " << clrf;
        }
        else if (strncmp(tmp_ptr, "CLRH", 4) == 0)
        {
            /* The blending of specular highlight colour between the object and lights colour */
            tmp_ptr += 6;
            clrh = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "CLRH: " << clrh;
        }
        else if (strncmp(tmp_ptr, "ADTR", 4) == 0)
        {
            /* Additive transparency */
            tmp_ptr += 6;
            const float adtr = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "ADTR: " << adtr;

            /* Use colour filter if present, otherwise additive transparency */
            if (clrf == 0.0f)
            {
                clrf = adtr;
            }
        }
        else if (strncmp(tmp_ptr, "ALPH", 4) == 0)
        {
            /* Alpha */
            tmp_ptr += 6;
            const std::uint16_t mode = from_byte_stream<std::uint16_t>(&tmp_ptr);
            const float alph = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "ALPH (not handled): " << mode << ", value: " << alph;
        }
        else if (strncmp(tmp_ptr, "SIDE", 4) == 0)
        {
            /* Sidedness */
            tmp_ptr += 6;
            tmp_ptr += 2;
            BOOST_LOG_TRIVIAL(info) << "SIDE (ignored)";
        }
        else if (strncmp(tmp_ptr, "GVAL", 4) == 0)
        {
            /* Glow value */
            tmp_ptr += 6;
            const float gval = from_byte_stream<float>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "GVAL (not handled): " << gval;
        }
        else if (strncmp(tmp_ptr, "RFOP", 4) == 0)
        {
            /* Reflection options, ignored because we always just raytrace */
            tmp_ptr += 6;
            tmp_ptr += 2;
            BOOST_LOG_TRIVIAL(info) << "RFOP (ignored)";
        }
        else if (strncmp(tmp_ptr, "TROP", 4) == 0)
        {
            /* Transparency options, ignored because we always just raytrace */
            tmp_ptr += 6;
            tmp_ptr += 2;
            BOOST_LOG_TRIVIAL(info) << "TROP (ignored)";
        }
        /* Catch unknown entities */
        else
        {
            BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< *tmp_ptr << " found in SURF chunk";
            return false;
        }

        /* Envelop */
        if ((strncmp(sec_ptr, "SMAN", 4) != 0) && (strncmp(sec_ptr, "RFOP", 4) != 0) && (strncmp(sec_ptr, "TROP", 4) != 0) && (strncmp(sec_ptr, "SIDE", 4) != 0) &&
            (strncmp(sec_ptr, "BLOK", 4) != 0) && (strncmp(sec_ptr, "RIMG", 4) != 0) && (strncmp(sec_ptr, "TIMG", 4) != 0) && (strncmp(sec_ptr, "ALPH", 4) != 0) && 
            (strncmp(sec_ptr, "FLAG", 4) != 0))
        {
            const std::uint16_t envelop_id = from_byte_stream<std::uint16_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(warning) << "Envelop (not handled): " << envelop_id;
            assert(envelop_id == 0);   /* The envelop must be null */
        }
    }

    /* Create the new material and return */
    _mat = new mapper_shader(bloks.btex(), bloks.ctex(), bloks.dtex(), bloks.stex(), bloks.rtex(), bloks.ttex(), rgb, vkd, vks, s, vt, ri, vr, clrh, clrf, bump);
    return true;
}
}; /* namespace raptor_raytracer */
