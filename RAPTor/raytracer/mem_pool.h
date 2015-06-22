#pragma once

#include "common.h"
#include "shape.h"


namespace raptor_raytracer
{
template <typename T, unsigned log_s = 16>
class mem_pool
{
    public :
        /* Constructors */
        mem_pool() : cur(0)
        {
            T *t = new T[1 << log_s];
            this->pools.push_back(t);
        }

        /* Destructor */
        ~mem_pool()
        {
            /* Clean up */
            for (unsigned i = 0; i < this->pools.size(); i++)
            {
                delete [] this->pools.back();
            }
        }
        
        /* Write access */
        /* Add to the back of the pool only */
        mem_pool & push_back(T &s)
        {
            /* Allocate a new pool */
            if (((1 << log_s) * this->pools.size()) <= this->cur)
            {
                T *t = new T[1 << log_s];
                this->pools.push_back(t);
                this->cur = 0;
            }
            
            /* Add */
            this->pools.back()[this->cur] = s;
            this->cur++;
            
            return *this;
        }
        
        /* Read access */
        T & back() const
        {
            return this->pools.back()[this->cur];
        }
        
        T & front() const
        {
            return this->pools.front()[0];
        }
        
        /* Read/write access */
        T & operator[](unsigned i) const
        {
            unsigned p = i & ~((1 << log_s) - 1);
            unsigned a = i &  ((1 << log_s) - 1);
            return this->pools[p][a];
        }
        
        /* Delete */
        mem_pool & pop_back(T &s)
        {
            /* Pop */
            this->cur--;
            
            /* Check for empty pool */
            if (this->cur > (1 << log_s))
            {
                this->cur = (1 << log_s) - 1;
                delete [] this->pools.back();
            }
            
            return *this;
        }
        
        /* Clear the pool */
        mem_pool & clear()
        {
            this->cur = 0;

            /* Clean up all dynamic memory */
            for (unsigned i = 0; i < this->pools.size(); i++)
            {
                delete [] this->pools.back();
            }
        }
        
        /* Reserve space for later use */
        T * reserve_space(unsigned s)
        {
            /* Allocate a new pool */
            if (((1 << log_s) * this->pools.size()) < (this->cur + s))
            {
                T *t = new T[1 << log_s];
                this->pools.push_back(t);
                this->cur = 0;
            }
            
            /* Leave the empty space */
            this->cur += s;
            
            return &this->pools.back()[this->cur - s];
        }


    private :
        vector<T *>     pools;
        unsigned        cur;
};
}; /* namespace raptor_raytracer */
