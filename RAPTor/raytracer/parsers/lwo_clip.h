#pragma once

/* Standard headers */

/* Boost headers */
#include "boost/shared_array.hpp"

/* Common headers */

/* Raytracer headers */
#include "picture_functions.h"
#include "texture_mapper.h"


namespace raptor_raytracer
{
/* Class to track chunks in current layer */
class lwo_clip
{
    public :
        lwo_clip(const std::string &path, const char *data, const int size) :
        _img(), _path(path), _data(data), _size(size) {  }

        bool parse()
        {
            const char *tmp_ptr = _data;
            _clip_idx = raptor_parsers::from_byte_stream<std::uint32_t>(&tmp_ptr);
            BOOST_LOG_TRIVIAL(trace) << "CLIP idx: " << _clip_idx;
        
            while (tmp_ptr < (_data + _size))
            {
                if (strncmp(tmp_ptr, "STIL", 4) == 0)
                {
                    tmp_ptr += 4;
                    const std::uint16_t stil_len = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
                    BOOST_LOG_TRIVIAL(trace) << "STIL: " << (_path + tmp_ptr);
                    
                    /* Load image */
                    float *img;
                    _filename = _path + tmp_ptr;
                    _cpp = read_jpeg(&img, _filename.c_str(), &_img_height, &_img_width);

                    /* Rescale to 0-1 */
                    _img.reset(img);
                    brightness_scale(_img.get(), _img_width * _img_height * _cpp, 1.0f / 255.0f);

                    tmp_ptr += stil_len;
                }
                else if (strncmp(tmp_ptr, "BRIT", 4) == 0)
                {
                    /* Brightness adjust for above still */
                    tmp_ptr += 6;
                    const float brit = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                    BOOST_LOG_TRIVIAL(trace) << "BRIT: " << brit;
                    assert(_img.get() != nullptr);
                    brightness_adjust(_img.get(), _img_width * _img_height * _cpp, brit);
                    tmp_ptr += 2;
                }
                else if (strncmp(tmp_ptr, "CONT", 4) == 0)
                {
                    /* Contrast adjust for above still */
                    tmp_ptr += 6;
                    const float cont = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                    BOOST_LOG_TRIVIAL(trace) << "CONT: " << cont;
                    assert(_img.get() != nullptr);
                    contrast_adjust(_img.get(), _img_width * _img_height * _cpp, cont);
                    tmp_ptr += 2;
                }
                else if (strncmp(tmp_ptr, "SATR", 4) == 0)
                {
                    /* Contrast adjust for above still */
                    tmp_ptr += 6;
                    const float satr = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                    BOOST_LOG_TRIVIAL(trace) << "SATR: " << satr;
                    assert(_img.get() != nullptr);
                    saturation_adjust(_img.get(), _img_width * _img_height * _cpp, satr);
                    tmp_ptr += 2;
                }
                else if (strncmp(tmp_ptr, "GAMM", 4) == 0)
                {
                    /* Contrast adjust for above still */
                    tmp_ptr += 6;
                    const float gamm = raptor_parsers::from_byte_stream<float>(&tmp_ptr);
                    BOOST_LOG_TRIVIAL(trace) << "GAMM: " << gamm;
                    assert(_img.get() != nullptr);
                    gamma_adjust(_img.get(), _img_width * _img_height * _cpp, gamm);
                    tmp_ptr += 2;
                }
                else if (strncmp(tmp_ptr, "NEGA", 4) == 0)
                {
                    /* Contrast adjust for above still */
                    tmp_ptr += 6;
                    const std::uint16_t nega = raptor_parsers::from_byte_stream<std::uint16_t>(&tmp_ptr);
                    BOOST_LOG_TRIVIAL(trace) << "NEGA: " << nega;
                    assert(_img.get() != nullptr);
                    negative(_img.get(), _img_width * _img_height * _cpp);
                }
                else
                {
                    BOOST_LOG_TRIVIAL(error) << "Unknown entity: "<< tmp_ptr << " found in CLIP chunk";
                    return false;
                }
            }

            /* Rescale back to 0 - 255 */
            brightness_scale(_img.get(), _img_width * _img_height * _cpp, 255.0f);

            return true;
        }

        /* Create a clone, but with the image scaled. This is used for bump mapping where each image can represent a different height */
        lwo_clip* scaled_clone(const float scale) const
        {
            /* Copy over most parameters */
            lwo_clip *const ret = new lwo_clip(_path, _data, _size);
            ret->_filename      = _filename;
            ret->_clip_idx      = _clip_idx;
            ret->_img_width     = _img_width;
            ret->_img_height    = _img_height;
            ret->_cpp           = _cpp;

            /* Copy image and scale */
            ret->_img.reset(new float [_img_width * _img_height * _cpp]);
            std::copy(&_img[0], &_img[_img_width * _img_height * _cpp], &ret->_img[0]);
            brightness_scale(ret->_img.get(), _img_width * _img_height * _cpp, scale);

            return ret;
        }

        /* Access functions */
        const boost::shared_array<float> &  image()         const { return _img;        }
        std::string                         filename()      const { return _filename;   }
        std::uint32_t                       clip_index()    const { return _clip_idx;   }
        std::uint32_t                       image_width()   const { return _img_width;  }
        std::uint32_t                       image_height()  const { return _img_height; }
        std::uint32_t                       colour_parts()  const { return _cpp;        }

    private :
        boost::shared_array<float>  _img;
        const std::string &         _path;
        const char * const          _data;
        std::string                 _filename;
        std::uint32_t               _clip_idx;
        std::uint32_t               _img_width;
        std::uint32_t               _img_height;
        std::uint32_t               _cpp;
        const int                   _size;
};
}; /* namespace raptor_raytracer */
