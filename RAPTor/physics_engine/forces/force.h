#ifndef __FORCE_H__
#define __FORCE_H__

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
        force(const point_t &at, const fp_t t) : _at(at), _t(t) {  };

        /* Virtual DTOR for inheritance */
        virtual ~force() {  };

        /* Allow default copy and assign */

        /* Virtual functions for applying a force */
        virtual point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const = 0;
        virtual point_t get_torque(const inertia_tensor &i, const point_t &x, const point_t &w, const fp_t dt) const
        {
            return cross_product(_at, get_force(i, x, w, dt));
        }

        /* Commit a time increment */
        bool commit(const fp_t dt)
        {
            _t -= dt;
            return _t <= 0.0;
        }

    protected :
        /* Get the remaining time the force should be applied for */
        fp_t time_remaining() const { return _t; }

    private :
        const point_t   _at;
        fp_t            _t;
};


class const_force : public force
{
    public :
        const_force(const point_t &at, const point_t &f, const fp_t t) : force(at, t), _f(f) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            return _f;
        };

    private :
        const point_t _f;
};


class linear_force : public force
{
    public :
        linear_force(const point_t &at, const point_t &m, const point_t &c, const fp_t t) : force(at, t), _m(m), _c(c), _t_tot(t) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            const fp_t t = dt + (_t_tot - time_remaining());
            return (_m * t) + _c;
        };
        
    private :
        const point_t   _m;
        const point_t   _c;
        const fp_t      _t_tot;
};


class squared_force : public force
{
    public :
        squared_force(const point_t &at, const point_t &m, const point_t &c, const fp_t t) : force(at, t), _m(m), _c(c), _t_tot(t) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            const fp_t t = dt + (_t_tot - time_remaining());
            return (_m * t * t) + _c;
        };
        
    private :
        const point_t   _m;
        const point_t   _c;
        const fp_t      _t_tot;
};


/* Distance based forces */
/* Be very careful not to allow objects near to, there is a singularity at this point that you wont like */
class attract_force : public force
{
    public :
        attract_force(const point_t &at, const point_t &to, const fp_t f, const fp_t c, const fp_t t) 
        : force(at, t), _to(to), _f(f), _c(c) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            const point_t dir = _to - x;
            const fp_t magn = magnitude(dir);
            return ((_f / magn) + _c) * (dir / magn);
        };
        
    private :
        const point_t   _to;
        const fp_t      _f;
        const fp_t      _c;
};


/* Be very careful not to allow objects near to, there is a singularity at this point that you wont like */
class repel_force : public force
{
    public :
        repel_force(const point_t &at, const point_t &to, const fp_t f, const fp_t c, const fp_t t) 
        : force(at, t), _to(to), _f(f), _c(c) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            const point_t dir = x - _to;
            const fp_t magn = magnitude(dir);
            return ((_f / magn) + _c) * (dir / magn);
        };
        
    private :
        const point_t   _to;
        const fp_t      _f;
        const fp_t      _c;
};


/* Sinusoidal forces */
class sin_force : public force
{
    public :
        sin_force(const point_t &at, const point_t &f, const point_t &c, const fp_t freq, const fp_t k, const fp_t t)
            : force(at, t), _f(f), _c(c), _freq(freq * 2.0 * PI), _k(k), _t_tot(t) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            const fp_t t = dt + (_t_tot - time_remaining());
            return (_f * sin((t * _freq) + _k)) + _c;
        };
        
    private :
        const point_t   _f;
        const point_t   _c;
        const fp_t      _freq;
        const fp_t      _k;
        const fp_t      _t_tot;
};


class cos_force : public force
{
    public :
        cos_force(const point_t &at, const point_t &f, const point_t &c, const fp_t freq, const fp_t k, const fp_t t)
            : force(at, t), _f(f), _c(c), _freq(freq * 2.0 * PI), _k(k), _t_tot(t) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            const fp_t t = dt + (_t_tot - time_remaining());
            return (_f * cos((t * _freq) + _k)) + _c;
        };
        
    private :
        const point_t   _f;
        const point_t   _c;
        const fp_t      _freq;
        const fp_t      _k;
        const fp_t      _t_tot;
};


/* Velocity based forces */
class viscous_force : public force
{
    public :
        viscous_force(const point_t &at, const fp_t f, const fp_t t) : force(at, t), _f(-f) {  };

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const 
        {
            return _f * v;
        };
        
    private :
        const fp_t _f;
};


/* Class to aggregate a collection of forces */
class aggregate_force
{
    public :
        aggregate_force(const std::vector<force *> &f) : _f(f) { }

        point_t get_force(const inertia_tensor &i, const point_t &x, const point_t &v, const fp_t dt) const
        {
            point_t agg(0.0, 0.0, 0.0);
            for (auto *f : _f)
            {
                agg += f->get_force(i, x, v, dt);
            }
            return agg;
        }

        point_t get_torque(const inertia_tensor &i, const point_t &x, const point_t &w, const fp_t dt) const
        {
            point_t agg(0.0, 0.0, 0.0);
            for (auto *f : _f)
            {
                agg += f->get_torque(i, x, w, dt);
            }
            return agg;
        }

    private :
        const std::vector<force *> &_f;
};
}; /* namespace raptor_physics */

#endif /* #ifndef __FORCE_H__ */
