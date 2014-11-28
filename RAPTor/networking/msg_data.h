#ifndef __MSG_DATA_H__
#define __MSG_DATA_H__

/* Standard headers */
#include <algorithm>
#include <stdexcept>
#include <vector>

/* Boost headers */
#include "boost/iostreams/stream.hpp"

/* Networking headers */
#include "vector_stream.h"


namespace raptor_networking
{
/* Class to represent the data in a message */
class msg_data
{
    private :
        typedef std::vector<std::unique_ptr<const char[]>> data_vec;

    public :
        msg_data(const char *data, const size_t frag_size)
            : _data(new data_vec()), 
              _frag_size(frag_size), 
              _total_size(frag_size), 
              _missing(0)
            {
                _data->emplace_back(data);
            };

        /* Allow more messages to be combined */
        msg_data& expand_slots(const size_t nr_frags, const size_t total_size)
        {
            /* Check that the number of slots are expanded */
            assert((nr_frags >= _data->size()) || !"Error: nr_frag is less than current number of slots");

            /* The total size must also be increasing */
            assert((total_size >= _total_size) || !"Error: total_size is being reduced by reducing the fragment size");

            /* Assumes size is always increasing */
            _missing += (nr_frags - _data->size());

            _total_size = total_size;
            _data->resize(nr_frags);
            return *this;
        }

        /* Combine another message into this one */
        msg_data& move(const std::shared_ptr<msg_data> &comb, const int start)
        {
            /* Calculate the upper limit of iteration */
            const int limit = (comb.get() == this) ? _data->size() : (start + comb->_data->size());

            /* The fragment sizes must be the same */
            assert(((static_cast<unsigned int>(limit) == _data->size()) || (comb->_frag_size == _frag_size)) || !"Error: Fragment sizes miss match");

            /* Check the upper range for data movement */
            assert((static_cast<unsigned int>(limit) <= _data->size()) || !"Error: Data to be moved out of range");

            /* Move data from comb */
            for (int i = limit - 1; i >= start; --i)
            {
                std::swap(_data->data()[i], comb->_data->data()[i - start]);
            }

            /* Assumes no duplicate data */
            _missing -= ((comb.get() == this) ? 0 : comb->_data->size());

            return *this;
        }

        /* Are all the fragments there */
        bool complete() const
        {
            return _missing == 0;
        }

        /* Convert to an istream */
        std::istream* as_stream()
        {
            return new boost::iostreams::stream<ivectorbuf_stream>(_data, _total_size, _frag_size);
        }

        /* Check if size bytes can be returned as a continious block */
        bool can_peek(const size_t size)
        {
            return ((_data->data()[0] != nullptr) && (_frag_size >= size));
        }

        bool first_fragment_begins(const char *const begin, const size_t size)
        {
            return can_peek(size) && (strncmp(_data->data()[0].get(), begin, size) == 0);
        }

        const char *const peek()
        {
            return _data->data()[0].get();
        }

    private :
        std::shared_ptr<data_vec>   _data;
        const size_t                _frag_size;
        size_t                      _total_size;
        size_t                      _missing;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __MSG_DATA_H__ */
