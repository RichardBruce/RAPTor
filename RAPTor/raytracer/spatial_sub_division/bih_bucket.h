#ifndef __BIH_BUCKET_H__
#define __BIH_BUCKET_H__

#include "common.h"
#include "triangle.h"


namespace raptor_raytracer
{
class bih_bucket
{
    public :
        /* Constructor */
        bih_bucket() : t(triangle::get_scene_upper_bounds()), b(triangle::get_scene_lower_bounds()), begin(0), end(0) { };
        
        /* Add list of primitive indices that are in this bucket */
        bih_bucket & add_indices(const int b, const int e)
        {
            this->begin = b;
            this->end   = e;
            
            /* If the node is empty */
            if (b == e)
            {
                this->t = triangle::get_scene_lower_bounds();
                this->b = triangle::get_scene_upper_bounds();
            }
            return *this;
        }
        
        /* Expand the bounds of the bucket */
        bih_bucket & expand(const point_t &top, const point_t &bot)
        {
            this->b.x = min(this->b.x, bot.x);
            this->b.y = min(this->b.y, bot.y);
            this->b.z = min(this->b.z, bot.z);

            this->t.x = max(this->t.x, top.x);
            this->t.y = max(this->t.y, top.y);
            this->t.z = max(this->t.z, top.z);

//            cout << "max: " << this->t.x << ", " << this->t.y << ", " << this->t.z << endl;
//            cout << "min: " << this->b.x << ", " << this->b.y << ", " << this->b.z << endl;
            return *this;
        }
        
        int size() const { return this->end - this->begin + 1; }
        
        /* Bounding box access functions for spatial subdivision */
        fp_t    lowest_x()          const   { return this->b.x;     }
        fp_t    lowest_y()          const   { return this->b.y;     }
        fp_t    lowest_z()          const   { return this->b.z;     }
        fp_t    highest_x()         const   { return this->t.x;     }
        fp_t    highest_y()         const   { return this->t.y;     }
        fp_t    highest_z()         const   { return this->t.z;     }
        
        /* Index access to swicth to normal builder */
        int get_begin()             const   { return this->begin;   }
        int get_end()               const   { return this->end;     }

    private : 
        point_t     t;      /*  */
        point_t     b;      /*  */
        int         begin;  /*  */
        int         end;    /*  */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __BIH_BUCKET_H__ */
