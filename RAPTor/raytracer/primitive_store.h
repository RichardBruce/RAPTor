#pragma once

/* Standard headers */
#include <vector>

/* Boost headers */

/* Common headers */

/* Raytracer headers */
#include "triangle.h"



namespace raptor_raytracer
{
class primitive_store
{
    public :
        /* Adding primitives */
        primitive_store& reserve(const int size)
        {
            _prims.reserve(size);
            _indirect.reserve(size);
            return *this;
        }

        template<class ...Args>
        int emplace_back(Args... args)
        {
            _prims.emplace_back(args...);
            _indirect.push_back(_indirect.size());
            return _prims.size() - 1;
        }

        /* Direct access to primitives */
        triangle *          primitive(const int i)          { return &_prims[i];    }
        const triangle *    primitive(const int i) const    { return &_prims[i];    }

        /* Indirect access to primitives */
        triangle *          indirect_primitive(const int i)         { return primitive(_indirect[i]);   }
        const triangle *    indirect_primitive(const int i) const   { return primitive(_indirect[i]);   }

        int     indirection(const int i) const  { return _indirect[i];  }
        int &   indirection(const int i)        { return _indirect[i];  }

        primitive_store& reset_indirection()
        {
            for (int i = 0; i < static_cast<int>(_indirect.size()); ++i)
            {
                _indirect[i] = i;
            }

            return *this;
        }

        primitive_store& swap(std::vector<int> &indirect)
        {
            _indirect.swap(indirect);
            return *this;
        }

        /* Iteration */
        std::vector<triangle>::iterator begin()
        {
            return _prims.begin();
        }
     
        std::vector<triangle>::iterator end()
        {
            return _prims.end();
        }

        std::vector<triangle>::const_iterator begin() const
        {
            return _prims.begin();
        }
     
        std::vector<triangle>::const_iterator end() const
        {
            return _prims.end();
        }

        /* Size */
        bool    empty()     const { return _prims.empty();      }
        int     size()      const { return _prims.size();       }
        int     capacity()  const { return _prims.capacity();   }

        /* Special */
        primitive_store& move_to_indirect()
        {
            /* Move primitives and reset indices */
            std::vector<triangle> moved;
            moved.reserve(_prims.size());
            for (unsigned int i = 0; i < _indirect.size(); ++i)
            {
                moved.push_back(std::move(_prims[_indirect[i]]));
                _indirect[i] = i;
            }

            /* Swap in new order */
            _prims.swap(moved);

            return *this;
        }


    private :
        std::vector<triangle>   _prims;
        std::vector<int>        _indirect;
};
}; /* namespace raptor_raytracer */
