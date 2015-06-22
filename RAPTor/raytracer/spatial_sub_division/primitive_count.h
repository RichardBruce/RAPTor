#pragma once

#include "common.h"
#include "triangle.h"

namespace raptor_raytracer
{
class primitive_count 
{
    public:
        primitive_count(const primitive_list &a, const fp_t sample[8], const axis n) : a(a), n(n) 
        {
            for (int i = 0; i < 8; i++)
            {
                this->sum_l[i] = 0;
                this->sum_r[i] = 0;
                this->s[i]     = sample[i];
            }
        }
        ~primitive_count() {  }
        
        /* Split into two primitive counters */
        primitive_count(primitive_count& x, split) : a(x.a), n(x.n)
        {
            for (int i = 0; i < 8; i++)
            {
                this->sum_l[i] = 0;
                this->sum_r[i] = 0;
                this->s[i]     = x.s[i];
            }
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
            switch (this->n)
            {
                case x_axis:
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
                    break;

                case y_axis:
                    for(size_t i = r.begin(); i != end; ++i)
                    {
                        for (int j = 0; j < 8; j++)
                        {
                            if ((*iter)->is_intersecting_y(s[j]))
                            {
                                this->sum_l[j]++;
                                this->sum_r[j]++;
                            }
                            else if ((*iter)->get_y0() < s[j])
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
                    break;

                case z_axis:
                    for(size_t i = r.begin(); i != end; ++i)
                    {
                        for (int j = 0; j < 8; j++)
                        {
                            if ((*iter)->is_intersecting_z(s[j]))
                            {
                                this->sum_l[j]++;
                                this->sum_r[j]++;
                            }
                            else if ((*iter)->get_z0() < s[j])
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
                    break;
                    
                default :
                    assert(false);
                    break;
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
        fp_t    s[8];
    
    private :
        const primitive_list   &a;
        const axis              n;
};
}; /* namespace raptor_raytracer */
