#ifndef __REGRESSION_FIXTURE_H__
#define __REGRESSION_FIXTURE_H__

/* Standard headers */
#include <cstdlib>
#include <chrono>
#include <list>
#include <string>
#include <vector>

/* Common headers */
#include "logging.h"
#include "point_t.h"

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Raytracer headers */
#include "scene.h"
#include "camera.h"
#include "cfg_parser.h"
#include "mgf_parser.h"
#include "nff_parser.h"
#include "lwo_parser.h"
#include "obj_parser.h"
#include "ply_parser.h"
#include "vrml_parser.h"
#include "raytracer.h"

/* Test headers */
#include "regression_checker.h"

#ifndef VALGRIND_TESTS
const int test_iterations = 5;
#else
const int test_iterations = 1;
#endif

/* Test data */
struct regression_fixture : private boost::noncopyable
{
    public :
        /* CTOR */
        regression_fixture(const std::string &input_file, const model_format_t input_format, 
            const point_t &cam_p = point_t(0.0, 0.0, -10), const point_t &x_vec = point_t(1.0, 0.0, 0.0), const point_t &y_vec = point_t(0.0, 1.0, 0.0), const point_t &z_vec = point_t(0.0, 0.0, 1.0),
            const ext_colour_t &bg = ext_colour_t(0.0, 0.0, 0.0),
            const fp_t rx = 0.0, const fp_t ry = 0.0, const fp_t rz = 0.0, 
            const unsigned int xr = 640, const unsigned int yr = 480, const unsigned int xa = 1, const unsigned int ya = 1, 
            const std::string &view_point = "")
        {
            /* Reset scene bounds */
            triangle::reset_scene_bounding_box();
            
            const fp_t screen_width     = 10.0;
            const fp_t screen_height    = screen_width * ((fp_t)yr / (fp_t)xr);

            const std::string data_dir(getenv("RAPTOR_DATA"));
            const std::string input_path(data_dir + input_file);

            int last_slash;
            std::string path;
            std::ifstream input_stream;
            const auto t0(std::chrono::system_clock::now());
            switch (input_format)
            {
                case model_format_t::cfg :
                {
                    input_stream.open(input_path.c_str());
                    assert(input_stream.is_open());
                    const unsigned int path_end = input_path.find_last_of("/\\") + 1;
                    cfg_parser(input_path.substr(0, path_end), input_stream, _lights, _everything, _materials, &_cam);

                    /* Camera is not set in the scene so do it here */
                    _cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
                    break;
                }
                case model_format_t::mgf :
                    mgf_parser(input_path.c_str(), _lights, _everything, _materials);
                    
                    /* Camera is not set in the scene so do it here */
                    _cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 10, xr, yr, xa, ya);
                    break;

                case model_format_t::nff :
                    input_stream.open(input_path.c_str());
                    assert(input_stream.is_open());
                    nff_parser(input_stream, _lights, _everything, _materials, &_cam);
                    break;

                case model_format_t::lwo :
                    input_stream.open(input_path.c_str());
                    assert(input_stream.is_open());
                    last_slash  = input_path.find_last_of('/');
                    path        = input_path.substr(0, last_slash + 1);
                    lwo_parser(input_stream, path, _lights, _everything, _materials, &_cam);

                    /* Camera is not set in the scene so do it here */
                    _cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
                    break;
                    
                case model_format_t::obj :
                    input_stream.open(input_path.c_str());
                    assert(input_stream.is_open());
                    last_slash  = input_path.find_last_of('/');
                    path        = input_path.substr(0, last_slash + 1);
                    obj_parser(input_stream, path, _lights, _everything, _materials, &_cam);
                    
                    /* Camera is not set in the scene so do it here */
                    _cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
                    break;

                case model_format_t::ply :
                    input_stream.open(input_path.c_str());
                    assert(input_stream.is_open());
                    ply_parser(input_stream, _lights, _everything, _materials, &_cam);
                    
                    /* Camera is not set in the scene so do it here */
                    _cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
                    break;

                case model_format_t::vrml :
                    /* Deafult camera set up for vrml -- NOTE negative z axis */
                    _cam = new camera(cam_p, point_t((fp_t)1.0, (fp_t)0.0, (fp_t) 0.0), 
                                             point_t((fp_t)0.0, (fp_t)1.0, (fp_t) 0.0), 
                                             point_t((fp_t)0.0, (fp_t)0.0, (fp_t)-1.0), bg, screen_width, screen_height, 20, xr, yr, xa, ya);

                    input_stream.open(input_path.c_str());
                    assert(input_stream.is_open());
                    vrml_parser(input_stream, _lights, _everything, _materials, _cam, view_point);
                    break;

                default :
                    assert(false);
                    break;
            }
            
            const auto t1(std::chrono::system_clock::now());
            BOOST_LOG_TRIVIAL(fatal) << "PERF 3 - Parser Time ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
            
            /* Pan tilt and roll the camera */
            _cam->tilt(rx);
            _cam->pan(ry);
            _cam->roll(rz);
        }

        /* DTOR */
        ~regression_fixture()
        {
            scene_clean(&_everything, &_materials, _cam);
        }

        regression_fixture& add_light(const ext_colour_t &rgb, const point_t &c, const fp_t d, const fp_t r)
        {
            new_light(&_lights, rgb, c, d, r);

            return *this;
        }

        regression_fixture& add_spotlight(const ext_colour_t &rgb, const point_t &c, const point_t &a, const fp_t r, const fp_t d, const fp_t s_a, const fp_t s_b)
        {
            const point_t n(normalise(c - a));
            new_light(&_lights, rgb, c, n, d, s_a, s_b, r);

            return *this;
        }

        regression_fixture& add_directional_light(const ext_colour_t &rgb, const point_t &c, const point_t &a, const fp_t d)
        {
            const point_t n(normalise(c - a));
            new_light(&_lights, rgb, n, d);

            return *this;
        }

        regression_fixture& render()
        {
            /* Assert there must be some lights to light the scene */
            assert(!_lights.empty());

            /* Assert there must be some materials for the objects */
            assert(!_materials.empty());

            /* Assert there is an imagine to trace */
            assert(!_everything.empty());

            /* Log scene size */
            BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - # Primitives: " << _everything.size();
            BOOST_LOG_TRIVIAL(fatal) << "PERF 2 - # Lights: " << _lights.size();

            /* Render */
            int max_runtime = 0;
            int total_runtime = 0;
            for (int i = 0; i < test_iterations; ++i)
            {
                const auto t0(std::chrono::system_clock::now());
                ray_tracer(_lights, _everything, *_cam);
                const auto t1(std::chrono::system_clock::now());

                /* Track run time and outliers */
                const int runtime = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                total_runtime += runtime;
                max_runtime = std::max(max_runtime, runtime);
            }

            /* Log test duration */
            if (test_iterations > 1)
            {
                total_runtime -= max_runtime;
            }
            const int avg_runtime = total_runtime / (std::max(1, test_iterations - 1) * 1000);
            BOOST_LOG_TRIVIAL(fatal) << "PERF 4 - Render Time ms: " << avg_runtime;

            return *this;
        }

        const camera& get_camera() const
        {
            return *_cam;
        }

    private :
        camera              *   _cam;
        light_list              _lights;
        primitive_list          _everything;
        std::list<material *>   _materials;
};

#endif /* #ifndef __REGRESSION_FIXTURE_H__ */
