#ifndef __GJK_TEST_ACCESS_H__
#define __GJK_TEST_ACCESS_H__

/* Common headers */
#include "quaternion_t.h"

/* Physics headers */
#include "gjk.h"
#include "simplex.h"
#include "vertex_group.h"


/* Class to access internals of gjk while testing */
class gjk_test_access
{
    public :
        gjk_test_access(const raptor_physics::vertex_group &vg_a, const raptor_physics::vertex_group &vg_b, raptor_physics::simplex *const sim_a, raptor_physics::simplex *const sim_b)
            : _gjk(vg_a, vg_b, sim_a, sim_b), _retain{0, 1, 2, 3} {  };

        /* Access to functions under test */
        bool find_minimum_distance(point_t *const dist, const point_t &fixed_rel_disp, const point_t &float_rel_disp, const quaternion_t &a_o, const quaternion_t &b_o)
        {
            return _gjk.find_minimum_distance(dist, fixed_rel_disp, float_rel_disp, a_o, b_o);
        }

        int find_closest_feature_to_origin(const point_t *const c_space, int *const verts, point_t *const dir) const
        {
            return _gjk.find_closest_feature_to_origin(c_space, verts, dir);
        }

        /* Access to set up state for tests */
        void set_simplex_size(const int size)
        {
            for (int i = 1; i <= size; ++i)
            {
                _gjk._a_simplex->retain_vertices(_retain, i);
                _gjk._a_simplex->add(0);
            }
        }

    private :
        raptor_physics::gjk _gjk;
        const int _retain[4];
};

#endif /* #ifndef __GJK_TEST_ACCESS_H__ */
