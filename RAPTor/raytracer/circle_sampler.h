#pragma once

/* Standard headers */
#include <random>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"

/* Ray tracer headers */
#include "sobol_numbers_2d.h"


namespace raptor_raytracer
{
class circle_sampler : private boost::noncopyable
{
    public:
        circle_sampler(const point_t<> &n, const point_t<> &r, const float s, const float a) : _rdist(0.0f, s), _tdist(0.0f, a), _n(othogonalise(n, normalise(r))), _r(r) {  }

        /* Take a completely random sample over the circle */
        point_t<> sample(const float r, const float t)
        {
            point_t<> pt(_r * r);
            rotate_about_origin(&pt, &_n, t);
            return pt;
        }

        void reset(const point_t<> &n, const point_t<> &r)
        {
            _r = r;
            _n = othogonalise(n, normalise(r));
        }

        static void seed() { _rand.seed(); }

    protected:
        static std::minstd_rand0                _rand;
        std::uniform_real_distribution<float>   _rdist;
        std::uniform_real_distribution<float>   _tdist;

    private:
        /* Want to make sure r and n are othogonal, its just a little bit cleaner */
        point_t<> othogonalise(const point_t<> &n, const point_t<> &r) const
        {
            const point_t<> p(cross_product(n, r));
            return cross_product(r, p);
        }

        point_t<> _n;
        point_t<> _r;
};

/* Random sampler for a circle */
class circle_sampler_random : public circle_sampler
{
    public:
        circle_sampler_random(const point_t<> &n, const point_t<> &r) : circle_sampler(n, r, 1.0f, 2.0f * PI) {  }

        void reset(const point_t<> &n, const point_t<> &r)
        {
            circle_sampler::reset(n, r);
        }

        /* Take a completely random sample over the circle */
        point_t<> sample()
        {
            return circle_sampler::sample(_rdist(_rand), _tdist(_rand));
        }
};

/* Sobol stratified sampler for a circle */
class circle_sampler_sobol : public circle_sampler
{
    public:
        circle_sampler_sobol(const point_t<> &n, const point_t<> &r, const int s) : circle_sampler(n, r, 1.0f / s, (2.0f * PI) / s), _ld(s) {  }

        void reset(const point_t<> &n, const point_t<> &r)
        {
            circle_sampler::reset(n, r);
            _ld.reset();
        }

        /* Take an evenly spreaded sample over the circle */
        point_t<> sample()
        {
            /* Pick low descrepency sector */
            float x;
            float y;
            _ld.next(x, y);

            /* Pick random point in sector */
            const float r_rand = _rdist(_rand);
            const float t_rand = _tdist(_rand);

            /* Convert the pair to a point */
            return circle_sampler::sample(x + r_rand, (y * 2.0f * PI) + t_rand);
        }

    private:
        sobol_numbers_2d<float> _ld;
};

/* Manually defined stratified sampler for a circle */
class circle_sampler_stratified : public circle_sampler
{
    public:
        circle_sampler_stratified(const point_t<> &n, const point_t<> &r, const int s) :
            circle_sampler(n, r, 1.0f, (2.0f * PI)),
            _r_samples(1),
            _t_samples(s),
            _r_sample(0),
            _t_sample(0)
        {
            const float root_2_inv = 1.0f / std::sqrt(2.0f);

            /* Initially only theta is split up, iteratively move those split to radial until the aspect ratio falls below 1 */
            float t_size = PI * (1.0f / _t_samples);
            float r_size = (std::sqrt((_r_samples + 2.0f) / _r_samples) - 1.0f) * root_2_inv;
            while ((r_size / t_size) > 1.0f)
            {
                ++_r_samples;
                _t_samples = s / static_cast<float>(_r_samples);
                t_size = PI * (1.0f / _t_samples);
                r_size = (std::sqrt((_r_samples + 2.0f) / _r_samples) - 1.0f) * root_2_inv;
            }

            /* Set up distribution for the first cells */
            reset();
        }

        void reset(const point_t<> &n, const point_t<> &r)
        {
            reset();
            circle_sampler::reset(n, r);
        }

        void reset()
        {
            _r_sample = 0;
            _t_sample = 0;
            _rdist.param(std::uniform_real_distribution<float>::param_type(0.0f, std::sqrt(1.0f / _r_samples)));
            _tdist.param(std::uniform_real_distribution<float>::param_type(0.0f,  (2.0f * PI) / _t_samples));
        }

        /* Take an evenly spreaded sample over the circle */
        point_t<> sample()
        {
            /* Pick cell */
            const float t = _t_sample * ((2.0f * PI) / _t_samples);

            /* Pick random sample */
            const float r_rand = _rdist(_rand);
            const float t_rand = _tdist(_rand);

            /* Advance sample */
            ++_t_sample;
            if (_t_sample == _t_samples)
            {
                _t_sample = 0;
                ++_r_sample;

                const float inner_r = std::sqrt(_r_sample / static_cast<float>(_r_samples));
                const float outer_r = std::sqrt((_r_sample + 1.0f) / static_cast<float>(_r_samples));
                _rdist.param(std::uniform_real_distribution<float>::param_type(inner_r, outer_r));
            }

            assert((_r_sample <= _r_samples) || !"Out of bounds sample");

            /* Pick a random offset */
            return circle_sampler::sample(r_rand, t + t_rand);
        }

        /* Getter to check on the sampling */
        int r_samples() const { return _r_samples;              }
        int t_samples() const { return _t_samples;              }
        int samples()   const { return _r_samples * _t_samples; }

    private:
        /* This is the fun part */
        
        /* We want to create cells of the same area, otherwise you would need to area weight the samples (not the worst thing ever, maybe one day we do that) */
        /* To do this we can sample angles evenly, but the radius must be sampled with the square root. Proof below, c is the fraction of the area we wish to have in this ring, rm is the radius to do this */
        /* pi * r^2 * c = pi * rm^2 */
        /* r^2 * c = rm^2 */
        /* r * c^0.5 = rm */

        /* We also want to keep the aspect ratio of the cells roughly around 1, otherwise we would end up with long thin or wide arcing cells that would sample far from their center in one dimension */
        /* This is a bit hackier, the aspect ratio if the cells will be smaller near the center and wider around the outside. So working with the middle rings of cells */

        int _r_samples;
        int _t_samples;
        int _r_sample;
        int _t_sample;
};

/* Manually defined stratified sampler for a circle */
/* Leave each ring at a fixed radius, but split it to keep the area and aspect ratio the same */
class circle_sampler_stratified_rings : public circle_sampler
{
    public:
        circle_sampler_stratified_rings(const point_t<> &n, const point_t<> &r, const int s) :
            circle_sampler(n, r, 1.0f, (2.0f * PI))
        {
            _rdist.param(std::uniform_real_distribution<float>::param_type(0.0f, 1.0f / _r_samples));
        }

        void reset(const point_t<> &n, const point_t<> &r)
        {
            reset();
            circle_sampler::reset(n, r);
        }

        void reset()
        {
            _r_sample = 0;
            _t_sample = 0;
            _t_samples = _t0_samples;
            _tdist.param(std::uniform_real_distribution<float>::param_type(0.0f,  (2.0f * PI) / _t_samples));
        }

        /* Take an evenly spreaded sample over the circle */
        point_t<> sample()
        {
            /* Pick random sample */
            const float r_rand = _rdist(_rand);
            const float t_rand = _tdist(_rand);
            const float r = _r_sample * _rdist.max();
            const float t = _t_sample * _tdist.max();

            /* Advance sample */
            ++_t_sample;
            if (_t_sample == _t_samples)
            {
                _t_sample = 0;
                _t_samples *= 3;
                ++_r_sample;
                _tdist.param(std::uniform_real_distribution<float>::param_type(0.0f,  _tdist.max() * (1.0f / 3.0f)));
            }

            /* Pick a random offset */
            return circle_sampler::sample(r + r_rand, t + t_rand);
        }

    private:
        /* Fun bit */
        /* If the arc length at the middle of the center ring is r*t then the arc length of the next ring 3r*t, then 5r*t then 7r*t.... */
        /* If the area of the center ring is 0.5*r*r*t then the area of the next ring are */
        /* 0.5*4*r*r*t  - 0.5*r*r*t     = 0.5*3*r*r*t   */
        /* 0.5*9*r*r*t  - 0.5*4*r*r*t   = 0.5*5*r*r*t   */
        /* 0.5*16*r*r*t - 0.5*9*r*r*t   = 0.5*7*r*r*t   */
        /* How good is that, they scale at the same rate. At each ring we split by the next odd number more and we control area and aspect ratio equally */

        int _t0_samples;
        int _t_samples;
        int _r_samples;
        int _r_sample;
        int _t_sample;
};
}; /* namespace raptor_raytracer */
