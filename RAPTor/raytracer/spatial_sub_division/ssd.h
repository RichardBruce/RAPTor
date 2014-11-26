#ifndef __SSD_H__
#define __SSD_H__

/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */

/* Raytracer headers */


namespace raptor_raytracer
{
/* Vritual class to be implemented by all types of spatial sub division */
class ssd : private boost::noncopyable
{
    public :
        /* Traversal functions */
#ifdef SIMD_PACKET_TRACING
        /* SIMD traversal */
        virtual void        frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const = 0;
        virtual void        frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned int size) const = 0;

        virtual void        find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const = 0;
        virtual vfp_t       found_nearer_object(const packet_ray *const r, const vfp_t &t) const = 0;
#endif /* #ifdef SIMD_PACKET_TRACING */

        /* Single ray traversal */
        virtual triangle*   find_nearest_object(const ray *const r, hit_description *const h) const = 0;
        virtual bool        found_nearer_object(const ray *const r, const float t) const = 0;

    private :
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __SSD_H__ */
