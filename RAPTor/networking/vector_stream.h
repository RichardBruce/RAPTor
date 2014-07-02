#ifndef __VECTOR_STREAM_H__
#define __VECTOR_STREAM_H__

/* Standard headers */
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <vector>

/* Boost headers */
#include "boost/iostreams/concepts.hpp"

/* Common headers */
#include "common.h"


namespace raptor_networking
{
/* Class to wrap a vector as an input stream */
class ivector_stream : public boost::iostreams::source
{
    public :
        /* CTOR */
        constexpr ivector_stream(const std::vector<char> * vec) : _vec(vec), _idx(0) {  };

        /* Move CTOR (really just copy constructor) */
        constexpr ivector_stream(ivector_stream &&rhs) : _vec(rhs._vec), _idx(rhs._idx) { };

        /* Copy CTOR */
        constexpr ivector_stream(const ivector_stream &rhs) : _vec(rhs._vec), _idx(rhs._idx) { }

        /* Read access to data */
        std::streamsize read(char* s, std::streamsize n)
        {
            /* Check how much stream is left */
            const std::streamsize result = std::min(n, static_cast<std::streamsize>(_vec->size() - _idx));

            /* Copy output */
            if (result != 0)
            {
                std::copy(_vec->begin() + _idx, _vec->begin() + _idx + result, s);
                _idx += result;
                return result;
            }
            /* EOF */
            else
            {
                return -1;
            }
        }

    private :
        const std::vector<char> *   _vec;   /* Vector to hold the data  */
        size_t                      _idx;   /* Position in the data     */
};


/* Class to wrap a vector as an output stream */
class ovector_stream : public boost::iostreams::sink
{
    public :
        /* CTOR */
        constexpr ovector_stream(std::vector<char> *const vec) : _vec(vec) {  };

        /* Write access to data */
        std::streamsize write(const char_type* s, std::streamsize n)
        {
            _vec->insert(_vec->end(), s, s + n);
            return n;
        }

    private :
        std::vector<char> *const _vec;  /* Vector to hold the data */
};


/* Class to wrap a vector as an input stream */
class ivectorbuf_stream : public boost::iostreams::source
{
    private :
        typedef std::vector<std::unique_ptr<const char[]>> data_vec;

    public :
        /* CTOR */
        ivectorbuf_stream(const std::shared_ptr<data_vec> vec, const size_t total_size, const size_t frag_size)
            : _vec(vec), 
              _total_size(total_size), 
              _frag_size(frag_size),
              _idx(0), 
              _seg(0)
              {  };

        /* Move CTOR */
        ivectorbuf_stream(ivectorbuf_stream &&rhs)
            : _vec(rhs._vec), _total_size(rhs._total_size), _frag_size(rhs._frag_size), _idx(rhs._idx), _seg(rhs._seg) { };

        /* Copy CTOR */
        ivectorbuf_stream(const ivectorbuf_stream &rhs)
            : _vec(rhs._vec), _total_size(rhs._total_size), _frag_size(rhs._frag_size), _idx(rhs._idx), _seg(rhs._seg) { };

        /* Read access to data */
        std::streamsize read(char* s, std::streamsize n)
        {
            /* Check how much stream is left */
            std::streamsize result = std::min(std::min(n, 
                                                static_cast<std::streamsize>(_frag_size - _idx)), 
                                            static_cast<std::streamsize>(_total_size - ((_seg * _frag_size) + _idx)));

            /* Copy output */
            if ((result != 0) || ((_seg + 1) < _vec->size()))
            {
                std::copy(&((*_vec)[_seg])[_idx], &((*_vec)[_seg])[_idx + result], s);

                /* Recurse if more output and available in another segment and required */
                if ((result < n) && ((_seg + 1) < _vec->size()))
                {
                    ++_seg;
                    _idx = 0;
                    result += read(s + result, n - result);
                }
                else
                {
                    _idx += result;
                }

                return result;
            }
            /* EOF */
            else
            {
                return -1;
            }
        }

    private :
        const std::shared_ptr<data_vec> _vec;           /* Vector to hold the data                  */
        const size_t                    _total_size;    /* The total number of bytes held           */
        const size_t                    _frag_size;     /* The number of bytes in each sub-vector   */
        size_t                          _idx;           /* The position in the current sub-vector   */
        size_t                          _seg;           /* The current sub-vector                   */
};
}; /* namespace raptor_networking */

#endif /* #ifndef __VECTOR_STREAM_H__ */
