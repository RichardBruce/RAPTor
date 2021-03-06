#pragma once

/* Standard headers */
#include <list>
#include <string>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"

/* Raytracer headers */
#include "texture_mapper.h"
#include "gradient_mapper.h"
#include "ext_colour_t.h"


namespace raptor_raytracer
{
/* Class to track chunks in current layer */
class lwo_bloks : private boost::noncopyable
{
    public :
        lwo_bloks() :
            _clip(nullptr),
            _trgb(ext_colour_t(255.0f, 255.0f, 255.0f)),
            _tfp{0.0f, 0.0f, 0.0f, 0.0f },
            _valu{0.0f, 0.0f, 0.0f },
            _topc(1.0f),
            _wrpw(1.0f),
            _wrph(1.0f),
            _tamp(1.0f),
            _shader(mapper_type_t::non),
            _map_of(mapper_of_t::map_ctex), 
            _twrp_mode_x(static_cast<texture_wrapping_mode_t>(2)),
            _twrp_mode_y(static_cast<texture_wrapping_mode_t>(2)),
            _fall_type(falloff_type_t::none),
            _nega(false)
            { }

        std::string parse(const std::map<std::uint32_t, lwo_clip *> &clips, const char **ptr, const std::uint32_t blok_len);

        const std::vector<texture_mapper *> & btex() const { return _btex; }
        const std::vector<texture_mapper *> & ctex() const { return _ctex; }
        const std::vector<texture_mapper *> & dtex() const { return _dtex; }
        const std::vector<texture_mapper *> & stex() const { return _stex; }
        const std::vector<texture_mapper *> & ttex() const { return _ttex; }
        const std::vector<texture_mapper *> & rtex() const { return _rtex; }

    private :
        inline void parse_tmap(const char **ptr, const std::uint32_t tmap_len);
        inline bool parse_header(const char **ptr, const std::uint32_t header_len);

        void add_shader();
        void add_shader_to(std::vector<texture_mapper *>  *const t);

        inline mapper_type_t pick_shader(const std::uint16_t m);
        inline void pick_procedural_shader(const char *c);
        inline void pick_gradient_shader(const char *c);
        inline mapper_of_t pick_channel(const char *const c);
        inline texture_wrapping_mode_t get_wrapping_mode(const std::uint16_t m);

        std::vector<texture_mapper *>               _btex;
        std::vector<texture_mapper *>               _ctex;
        std::vector<texture_mapper *>               _dtex;
        std::vector<texture_mapper *>               _stex;
        std::vector<texture_mapper *>               _ttex;
        std::vector<texture_mapper *>               _rtex;
        std::vector<grad_interpolation_t>           _ikey;
        std::vector<float>                          _fkey_key;
        std::vector<std::pair<ext_colour_t, float>> _fkey_value;
        lwo_clip *                                  _clip;
        ext_colour_t                                _trgb;
        point_t<>                                   _tctr;
        point_t<>                                   _tnorm;
        point_t<>                                   _tsiz;
        point_t<>                                   _fall_grad;
        float                                       _tfp[4];
        float                                       _valu[3];
        float                                       _topc;
        float                                       _wrpw;
        float                                       _wrph;
        float                                       _tamp;
        mapper_type_t                               _shader;
        mapper_of_t                                 _map_of;
        std::uint16_t                               _tip;
        texture_wrapping_mode_t                     _twrp_mode_x;
        texture_wrapping_mode_t                     _twrp_mode_y;
        falloff_type_t                              _fall_type;
        grad_of_t                                   _grad_of;
        bool                                        _nega;
};
}; /* namespace raptor_raytracer */
