#pragma once

#include "common.h"
#include "triangle.h"


namespace raptor_raytracer
{
class primitive_count 
{
    public:
        primitive_count(primitive_list &a[]) : a(a) 
        {
            this->sum_l   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            this->sum_r   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        }
        ~primitive_count() {  }
        
        /* Split into two primitive counters */
        primitive_count(primitive_count& x, split) : a(x.a)
        {
            this->sum_l   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            this->sum_r   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        }

        /* Sum the list */
        void operator()(const blocked_range<size_t> &r)
        {
            size_t begin    = r.begin();
            size_t end      = r.end();
            
            /* Move the iterator to its start location */
            primitive_list::const_iterator iter = a.begin();
            for(size_t i = 0; i != begin; ++i)
            {
                iter++;
            }
            
            /* Count the primitives */
            for(size_t i = r.begin(); i != end; ++i)
            {
                for (int j = 0; j < 8; j++)
                {
                    if ((*iter)->is_intersecting_x(s[j]))
                    {
                        this->sum_l[j]++;
                        this->sum_r[j]++;
                    }
                    else if ((*iter)->get_x0() < s[j])
                    {
                        this->sum_l[j]++;
                    }
                    else
                    {
                        this->sum_r[j]++;
                    }
                }
                iter++;
            }
        }

        /* Join two primitive counters */
        void join(const primitive_count &y)
        {
            for (int i = 0; i < 8; i++)
            {
                this->sum_l[i] += y.sum_l[i];
                this->sum_r[i] += y.sum_r[i];
            }
        }

        /* Public data */
        fp_t    sum_l[8];
        fp_t    sum_r[8];
    
    private :
        const primitive_list   &a;
        fp_t                    s[8];
};
}; /* namespace raptor_raytracer */
