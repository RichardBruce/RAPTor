#pragma once

/* Common headers */
#include "point_t.h"

/* Physics headers */
#include "inertia_tensor.h"


namespace raptor_physics
{
/* Abstract base class for all forces */
class force
{
    public :
        /* CTOR */
        force(const point_t<> &at, const float t) : _at(at), _t(t) {  };

        /* Virtual DTOR for inheritance */
        virtual ~force() {  };

        /* Allow default copy and assign */

        /* Virtual functions for applying a force */
        virtual point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const = 0;
        virtual point_t<> get_torque(const inertia_tensor &i, const point_t<> &x, const point_t<> &w, const float dt) const
        {
            return cross_product(_at, get_force(i, x, w, dt));
        }

        /* Commit a time increment */
        bool commit(const float dt)
        {
            _t -= dt;
            return _t <= 0.0f;
        }

    protected :
        /* Get the remaining time the force should be applied for */
        float time_remaining() const { return _t; }

    private :
        const point_t<> _at;
        float           _t;
};


class const_force : public force
{
    public :
        const_force(const point_t<> &at, const point_t<> &f, const float t) : force(at, t), _f(f) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            return _f;
        };

    private :
        const point_t<> _f;
};


class linear_force : public force
{
    public :
        linear_force(const point_t<> &at, const point_t<> &m, const point_t<> &c, const float t) : force(at, t), _m(m), _c(c), _t_tot(t) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            const float t = dt + (_t_tot - time_remaining());
            return (_m * t) + _c;
        };
        
    private :
        const point_t<> _m;
        const point_t<> _c;
        const float     _t_tot;
};


class squared_force : public force
{
    public :
        squared_force(const point_t<> &at, const point_t<> &m, const point_t<> &c, const float t) : force(at, t), _m(m), _c(c), _t_tot(t) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            const float t = dt + (_t_tot - time_remaining());
            return (_m * t * t) + _c;
        };
        
    private :
        const point_t<> _m;
        const point_t<> _c;
        const float     _t_tot;
};


/* Distance based forces */
/* Be very careful not to allow objects near to, there is a singularity at this point that you wont like */
class attract_force : public force
{
    public :
        attract_force(const point_t<> &at, const point_t<> &to, const float f, const float c, const float t) 
        : force(at, t), _to(to), _f(f), _c(c) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            const point_t<> dir = _to - x;
            const float magn = magnitude(dir);
            return ((_f / magn) + _c) * (dir / magn);
        };
        
    private :
        const point_t<> _to;
        const float     _f;
        const float     _c;
};


/* Be very careful not to allow objects near to, there is a singularity at this point that you wont like */
class repel_force : public force
{
    public :
        repel_force(const point_t<> &at, const point_t<> &to, const float f, const float c, const float t) 
        : force(at, t), _to(to), _f(f), _c(c) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            const point_t<> dir = x - _to;
            const float magn = magnitude(dir);
            return ((_f / magn) + _c) * (dir / magn);
        };
        
    private :
        const point_t<> _to;
        const float     _f;
        const float     _c;
};


/* Sinusoidal forces */
class sin_force : public force
{
    public :
        sin_force(const point_t<> &at, const point_t<> &f, const point_t<> &c, const float freq, const float k, const float t)
            : force(at, t), _f(f), _c(c), _freq(freq * 2.0f * PI), _k(k), _t_tot(t) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            const float t = dt + (_t_tot - time_remaining());
            return (_f * std::sin((t * _freq) + _k)) + _c;
        };
        
    private :
        const point_t<> _f;
        const point_t<> _c;
        const float     _freq;
        const float     _k;
        const float     _t_tot;
};


class cos_force : public force
{
    public :
        cos_force(const point_t<> &at, const point_t<> &f, const point_t<> &c, const float freq, const float k, const float t)
            : force(at, t), _f(f), _c(c), _freq(freq * 2.0 * PI), _k(k), _t_tot(t) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            const float t = dt + (_t_tot - time_remaining());
            return (_f * std::cos((t * _freq) + _k)) + _c;
        };
        
    private :
        const point_t<> _f;
        const point_t<> _c;
        const float     _freq;
        const float     _k;
        const float     _t_tot;
};


/* Velocity based forces */
class viscous_force : public force
{
    public :
        viscous_force(const point_t<> &at, const float f, const float t) : force(at, t), _f(-f) {  };

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const 
        {
            return _f * v;
        };
        
    private :
        const float _f;
};


/* Class to aggregate a collection of forces */
class aggregate_force
{
    public :
        aggregate_force(const std::vector<force *> &f) : _f(f), _if(point_t<>(0.0f, 0.0f, 0.0f)), _it(point_t<>(0.0f, 0.0f, 0.0f)) { }

        /* Internal forces */
        aggregate_force& clear_internal_forces()
        {
            _if = point_t<>(0.0f, 0.0f, 0.0f);
            _it = point_t<>(0.0f, 0.0f, 0.0f);
            return *this;
        }

        aggregate_force& apply_internal_force(const point_t<> &at, const point_t<> &f)
        {
            _if += f;
            _it += cross_product(at, f);
            return *this;
        }

        point_t<> get_force(const inertia_tensor &i, const point_t<> &x, const point_t<> &v, const float dt) const
        {
            point_t<> agg(_if);
            for (auto *f : _f)
            {
                agg += f->get_force(i, x, v, dt);
            }
            return agg;
        }

        point_t<> get_torque(const inertia_tensor &i, const point_t<> &x, const point_t<> &w, const float dt) const
        {
            point_t<> agg(_it);
            for (auto *f : _f)
            {
                agg += f->get_torque(i, x, w, dt);
            }
            return agg;
        }

    private :
        const std::vector<force *> &_f;
        point_t<>                   _if;
        point_t<>                   _it;
};
}; /* namespace raptor_physics */
