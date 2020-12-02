#pragma once

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"


class base_camera : private boost::noncopyable
{
    public:
        base_camera(const point_t<> &c, const point_t<> &x, const point_t<> &y, const point_t<> &z, const float speed) :
            _c(c), _x(x), _y(y), _z(z), _speed(speed)
        {  }

        virtual ~base_camera() {  }

        /* Access functions */
        const point_t<> &     camera_position()   const { return _c;       }
        const quaternion_t &orientation()       const { return _o;       }
        float               speed()             const { return _speed;   }

        point_t<> x_axis() const { return _o.rotate(_x);   }
        point_t<> y_axis() const { return _o.rotate(_y);   }
        point_t<> z_axis() const { return _o.rotate(_z);   }
        
        /* Dump function */
        void print_position_data() const
        {
            std::cout << "--cam " << _c       << std::endl;
            std::cout << "--dx  " << x_axis() << std::endl;
            std::cout << "--dy  " << y_axis() << std::endl;
            std::cout << "--dz  " << z_axis() << std::endl;
            std::cout << "_speed " << _speed  << std::endl;
        }

        /* Speed control */
        base_camera& speed_up()
        {
            _speed *= 2.0f;
            return *this;
        }
        
        base_camera& slow_down()
        {
            _speed *= 0.5f;
            return *this;
        }

        /* Camera movement */
        base_camera& move_to(const point_t<> &p)
        {
            _c = p;
            return *this;
        }
        
        base_camera& move_forward(const float d = 1.0f)
        {
            _c += z_axis() * _speed * d;
            return *this;
        }
        
        base_camera& move_up(const float d = 1.0f)
        {
            _c += y_axis() * _speed * d;
            return *this;
        }
        
        base_camera& move_right(const float d = 1.0f)
        {
            _c += x_axis() * _speed * d;
            return *this;
        }
        
        /* Camera rotate */        
        /* Rotate about _y */
        base_camera& pan(const float a)
        {
            _o *= quaternion_t(point_t<>(0.0f, 1.0f, 0.0f), _speed * a);
            normalise(&_o);
            return *this;
        }
    
        /* Rotate about _x */
        base_camera& tilt(const float a)
        {
            _o *= quaternion_t(point_t<>(1.0f, 0.0f, 0.0f), _speed * a);
            normalise(&_o);
            return *this;
        }

        /* Rotate about _z */
        base_camera& roll(const float a)
        {
            _o *= quaternion_t(point_t<>(0.0f, 0.0f, 1.0f), _speed * a);
            normalise(&_o);
            return *this;
        }
        
        /* Rotate about an arbitary axis */
        base_camera& rotate_about(const point_t<> &v, const float a)
        {
            _o *= quaternion_t(point_t<>(v.x, v.y, v.z), _speed * a);
            normalise(&_o);
            return *this;
        }

    private:
        point_t<>       _c;                          /* Camera position                                                  */
        quaternion_t    _o;                          /* Camera orientation                                               */
        point_t<>       _x;                          /* Horizontal axis                                                  */
        point_t<>       _y;                          /* Vertical axis                                                    */
        point_t<>       _z;                          /* Forward axis                                                     */
        float           _speed;                      /* Speed of movement                                                */
};
