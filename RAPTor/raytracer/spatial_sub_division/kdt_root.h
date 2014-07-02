#ifndef __KDT_ROOT_H__
#define __KDT_ROOT_H__

#include "common.h"
#include "ray.h"
#include "kdt_node.h"

class kdt_root
{
    public :
        kdt_root(const kdt_node *const k, const point_t &t, const point_t &b) : k(k), t(t), b(b) { };
        ~kdt_root() 
        { 
            delete this->k;
        };
        
        /* Access functions */
        const kdt_node *const get_kd_tree() const { return this->k; }
        
    private :
        const kdt_node *const  k;
        const point_t  t, b;
};

#endif /* #ifndef __KDT_ROOT_H__ */
