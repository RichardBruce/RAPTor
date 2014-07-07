#ifndef __PHYSICS_ENGINE_H__
#define __PHYSICS_ENGINE_H__

/* Standard headers */
#include <algorithm>
#include <deque>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"

/* Physics headers */
#include "collider.h"
#include "physics_object.h"
#include "tracking_info.h"

namespace raptor_physics
{
/* Forward declarations */
class force;
class simplex;

class physics_engine : private boost::noncopyable
{
    public :
        /* CTOR */
        /* Ownership is taken of the collider, it will be deleted */
        physics_engine(const collider *const c, const bool clean_objects = true)
         :  _objects(new std::unordered_map<int, physics_object*>),
            _moving_objects(new std::unordered_map<int, physics_object*>),
            _collision_cache(new std::map<physics_object*, tracking_info<physics_object>*>()),
            _collider_map(),
            _collider(c),
            _xb(new std::vector<bound_point*>()),
            _yb(new std::vector<bound_point*>()),
            _zb(new std::vector<bound_point*>()),
            _entry(0),
            _clean_objects(clean_objects) {  };
            
        /* DTOR */
        ~physics_engine()
        {
            /* Clean up the scene objects */
            if (_clean_objects)
            {
                for (auto& p : (*_objects))
                {
                    delete p.second;
                }
            }
            delete _objects;
            delete _moving_objects;

            /* Clean up tracking info */
            for (auto& p : (*_collision_cache))
            {
                delete p.second;
            }
            delete _collision_cache;

            /* Clean up pairwise colliders */
            for (auto& outer : _collider_map)
            {
                for (auto& inner : *(outer.second))
                {
                    delete inner.second;
                }
                delete outer.second;
            }
            
            /* Clean up the default collider */
            delete _collider;
            
            /* Bounds are cleared when vg meta is deleted */
            delete _xb;
            delete _yb;
            delete _zb;
        }
    
        physics_engine& apply_force(force *const f, const int i)
        {
            /* Only moving objects can have forces applied to them */
            auto mov_obj_iter = _moving_objects->find(i);
            if (mov_obj_iter != _moving_objects->end())
            {
                mov_obj_iter->second->register_force(f);
            }
            else
            {
                delete f;
            }

            return *this;
        }

        physics_engine& advance_time(const fp_t t);
        
        raptor_raytracer::primitive_list* scene_to_triangles() const
        {
            raptor_raytracer::primitive_list *ret = new raptor_raytracer::primitive_list();
            for (auto& p : (*_objects))
            {
                p.second->triangles(ret);
            }
            
            return ret;
        }

        /* Voids all collisions with a or b */
        /* This is used if vg_a and vg_b have collided and are now moving in different directions */
        physics_engine& void_all_collisions_with(const int i)
        {
            auto obj_iter = _objects->find(i);
            if (obj_iter == _objects->end())
            {
                return *this;
            }
            else
            {
                return void_all_collisions_with(obj_iter->second);
            }
        }

        physics_engine& void_all_collisions_with(physics_object *const vg)
        {
            for (auto &pv : (*_collision_cache))
            {
                pv.second->void_collision(vg);
            }

            return *this;
        }

        /* Collider access functions */
        const collider *const default_collider() const { return _collider; }
        physics_engine& default_collider(const collider *const c)
        {
            delete _collider;
            _collider = c;
            return *this;
        }

        const collider *const pair_collider(const unsigned int i, const unsigned int j) const
        {
            auto outer_iter = _collider_map.find(std::min(i, j));
            if (outer_iter == _collider_map.end())
            {
                BOOST_LOG_TRIVIAL(trace) << "Defaulted collider at outer level";
                return _collider;
            }

            auto inner_iter = outer_iter->second->find(std::max(i, j));
            if (inner_iter == outer_iter->second->end())
            {
                BOOST_LOG_TRIVIAL(trace) << "Defaulted collider at inner level";
                return _collider;
            }

            return inner_iter->second;
        }

        physics_engine& pair_collider(const collider *const c, const unsigned int i, const unsigned int j)
        {
            const int min = std::min(i, j);
            const int max = std::max(i, j);
            auto outer_iter = _collider_map.find(min);
            if (outer_iter == _collider_map.end())
            {
                const auto inserted = _collider_map.insert(std::make_pair(min, new std::map<unsigned int, const collider *>()));
                assert(inserted.second);
                outer_iter = inserted.first;
            }

            auto inner_iter = outer_iter->second->find(max);
            if (inner_iter == outer_iter->second->end())
            {
                const auto inserted = outer_iter->second->insert(std::make_pair(max, c));
                assert(inserted.second);
            }
            else
            {
                delete inner_iter->second;
                inner_iter->second = c;
            }

            return *this;
        }

        /* Object access functions */
        int number_of_objects()         const { return _objects->size();        }
        int number_of_moving_objects()  const { return _moving_objects->size(); }
        int next_object_id()            const { return _entry;                  }

        const physics_object *const get_object(const int i) const
        {
            auto obj_iter = _objects->find(i);
            if (obj_iter == _objects->end())
            {
                return nullptr;
            }
            else
            {
                return obj_iter->second;
            }
        }

        int add_object(physics_object *const v)
        {
            const auto inserted = _objects->insert(std::make_pair(_entry++, v));
            assert(inserted.second);

            return _entry;
        }

        int add_moving_object(physics_object *const v)
        {
            /* Add as an object */
            const auto obj_inserted = _objects->insert(std::make_pair(_entry, v));
            assert(obj_inserted.second);

            /* Add as a moving object */
            const auto inserted = _moving_objects->insert(std::make_pair(_entry++, v));
            assert(inserted.second);

            /* Check for collisions */
            collision_detect_versus(inserted.first, 0.0, true);

            return _entry;
        }
        
        physics_engine& remove_object(const int i)
        {
            /* Remove from objects */
            auto obj_iter = _objects->find(i);
            if (obj_iter != _objects->end())
            {
                delete obj_iter->second;
                _objects->erase(obj_iter);
            }

            /* Possibly remove from moving objects */
            auto mov_obj_iter = _moving_objects->find(i);
            if (mov_obj_iter != _moving_objects->end())
            {
                _moving_objects->erase(mov_obj_iter);
            }

            return *this;
        }

        /* Collision access functions */
        int number_of_collisions() const { return _collision_cache->size(); }

        const collision_info *const get_collision(const int i, const int j) const
        {
            /* Try to find objects */
            auto obj_iter_i = _objects->find(i);
            auto obj_iter_j = _objects->find(j);
            if ((obj_iter_i == _objects->end()) || (obj_iter_j == _objects->end()))
            {
                return nullptr;
            }

            /* Try to find tracking info for the first object */
            auto coll_iter = _collision_cache->find(obj_iter_i->second);
            if (coll_iter == _collision_cache->end())
            {
                return nullptr;
            }

            /* Try to get the collision info */
            return (*(coll_iter->second))[obj_iter_j->second];
        }
        

    private :
        /* typedef for convinence */
        friend physics_object;
        typedef std::pair<physics_object *, fp_t> bound_point;
        typedef std::unordered_map<int, physics_object*>::iterator obj_iter;
        typedef std::unordered_map<int, physics_object*>::const_iterator const_obj_iter;
        

        /* Functor to check what sort of a collision a collision cache element represents */
        class is_collision
        {
            public :
                is_collision(const collision_t t, const bool invert) : _t(t), _invert(invert) {  };

                bool operator()(const std::pair<physics_object*, tracking_info<physics_object>*> &p) const
                {
                    return _invert ^ (to_certain(p.second->get_first_collision_type()) == _t);
                }

            private :
                const collision_t   _t;
                const bool          _invert;
        };


        /* Update tracking info after a test */
        physics_engine& update_tracking_info(physics_object *vg_a, physics_object *vg_b, simplex *s, const simplex &other_s,
            const fp_t t, const collision_t type)
        {
            auto coll_iter = _collision_cache->find(vg_a);
            if (coll_iter != _collision_cache->end())
            {
                coll_iter->second->update(vg_b, s, other_s, t, type);
            }
            else
            {
                _collision_cache->insert({vg_a, new tracking_info<physics_object>(vg_b, s, other_s, t, type)});
            }

            return *this;
        }

        /* Update tracking info after a successful retest */
        physics_engine& update_tracking_info(physics_object *vg_a, physics_object *vg_b, simplex *s, const simplex &other_s)
        {
            auto tracking = _collision_cache->find(vg_a);

            /* If this happens then this is not a retest */
            assert(tracking != _collision_cache->end());

            tracking->second->successful_retest_update(vg_b, s, other_s);
            
            return *this;
        }

        /* Collision detection */
        bool            collide_and_cache(physics_object *const vg_a, physics_object *const vg_b, const fp_t t);
        bool            retest_and_cache(physics_object *const vg_a, physics_object *const vg_b, const fp_t t);
        physics_engine& collision_detect_versus(const const_obj_iter &v, fp_t t, const bool all);

        std::unordered_map<int, physics_object*>   *                        _objects;
        std::unordered_map<int, physics_object*>   *                        _moving_objects;
        std::map<physics_object*, tracking_info<physics_object>*>  *        _collision_cache;   /* Cache of object vs. object collisions. This is expected to be sparsely populated. */
        std::map<unsigned int, std::map<unsigned int, const collider*>*>    _collider_map;
        const collider                      *                               _collider;
        std::vector<bound_point*>           *                               _xb;
        std::vector<bound_point*>           *                               _yb;
        std::vector<bound_point*>           *                               _zb;
        int                                                                 _entry;
        const fp_t                                                          _max_d = 1.0;
        const bool                                                          _clean_objects;
};
}; /* namespace raptor_physics */

#endif /* #ifndef __PHYSICS_ENGINE_H__ */
