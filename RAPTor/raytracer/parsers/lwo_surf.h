#pragma once

/* Standard headers */
#include <string>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */

/* Raytracer headers */
#include "lwo_clip.h"


namespace raptor_raytracer
{
/* Class to track chunks in current layer */
class lwo_surf : private boost::noncopyable
{
    public :
        lwo_surf() : _mat(nullptr), _smoothing_threshold(0.0f) { };

        bool parse(const std::map<std::uint32_t, lwo_clip *> &clips, const char *data, const int size, const int mat_num);

        mapper_shader *     material()              const { return _mat;                    }
        const std::string & vmap_name()             const { return _vmap;                   }
        float               smoothing_threshold()   const { return _smoothing_threshold;    }

    private :
        mapper_shader *     _mat;
        std::string         _vmap;
        float               _smoothing_threshold;
};
}; /* namespace raptor_raytracer */
