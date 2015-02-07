/* Standard headers */
#include <iomanip>
#include <iostream>
#include <sstream>

/* Common headers */
#include "logging.h"

/* Raytracer headers */
#include "bih.h"
#include "raytracer.h"

/* Physics headers */
#include "simulation_environment.h"


namespace raptor_physics
{
int simulation_environment::run()
{
    BOOST_LOG_TRIVIAL(info) << "Beginning Physics Simulation";

    _time_run = 0.0f;
    std::unique_ptr<unsigned char[]> screen_data(new unsigned char [_cam.x_resolution() * _cam.y_resolution() * 3]);

    std::ostringstream fps;
    fps << std::fixed << std::setprecision(2);
    
    int do_next = 0;
    bool paused = false;
    while ((do_next != 1) && (_po->pause_on_last() || (_po->frames_to_run() != 0)))
    {
        /* Process user input */
        do_next = _cam_event_handler->process_events();

        /* Update time */
        const clock_t time_now = clock();
        float time_step = std::max(_po->min_timestep(), (time_now - _sim_time) * clocks_per_sec_inv);
        if (_po->max_timestep() > 0.0f)
        {
            time_step = std::min(_po->max_timestep(), time_step);
        }
        _sim_time = time_now;
        _time_run += time_step;

        /* Dont update the physics if paused */
        if (!paused)
        {
            /* Simulate phyics */
            _pe->advance_time(time_step);
        }

        /* render */
        if (_po->render())
        {
            raptor_raytracer::primitive_list *tris = _pe->scene_to_triangles();
            std::unique_ptr<raptor_raytracer::bih> ssd(new raptor_raytracer::bih(*tris));
            raptor_raytracer::ray_tracer(ssd.get(), _lights, *tris, _cam);

            /* Clean up triangles */        
            for (auto t : (*tris))
            {
                delete t;
            }
            delete tris;

            _damped_fps = (0.5f * _damped_fps) + (0.5f / time_step);
            fps.str(std::string());
            fps << _damped_fps;            

            /* Display the output */
            _cam.clip_image_to_bgr(screen_data.get());

            const int draw_status = draw_screen(_window, _renderer, _texture, _font, fps.str(), screen_data.get());
            if (draw_status)
            {
                return draw_status;
            }
        }

        /* Decrement remaining frames */
        _po->frame_done();

        /* Check if the next frame should be paused */
        paused = _po->pause_on_last() && (_po->frames_to_run() == 0);

        BOOST_LOG_TRIVIAL(info) << "Frame done, remaining: " << _po->frames_to_run();
    }

    BOOST_LOG_TRIVIAL(info) << "Completed Physics Simulation";
    return 0;
}
}; /* namespace raptor_physics */
