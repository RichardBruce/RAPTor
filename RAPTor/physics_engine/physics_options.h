#pragma once

/* Standard headers */
#include <iostream>

/* Boost headers */
#include "boost/lexical_cast.hpp"

/* Common headers */
#include "common.h"


namespace raptor_physics
{
/* Class to hold options on how to run the physics simulation */
class physics_options
{
    public :
        /* CTOR */
        physics_options(const float max_ts = -1.0f, const float min_ts = 0.0f, const int frames_to_run = -1, const bool pause_on_last = false, const bool render = true)
            : _max_ts(max_ts), _min_ts(min_ts), _frames_to_run(frames_to_run), _pause_on_last(pause_on_last), _render(render), _error(false) { };

        physics_options(const char *const *const argv, const int argc)
            : _max_ts(-1.0f), _min_ts(0.0f), _frames_to_run(-1), _pause_on_last(false), _render(true), _error(false)
        {
            /* Parse input arguements */
            if (argc > 1)
            {
                for (int i = 1; i < argc; ++i)
                {
                    const char *cur_arg = argv[i];
                    if ((cur_arg[0] == '-') && (cur_arg[1] == '-'))
                    {
                        ++cur_arg;
                    }

                    /* Maximum frames per second */
                    if (strcmp(cur_arg, "-max_fps") == 0)
                    {
                        if ((argc - i) < 2)
                        {
                            std::cout << "Error: Incorrectly specified maximum fps." << std::endl;
                            _error = true;
                            return;
                        }
                        else
                        {
                            _max_ts = 1.0f / boost::lexical_cast<float>(argv[++i]);
                        }
                    }
                    /* Minimum frames per second */
                    else if (strcmp(cur_arg, "-min_fps") == 0)
                    {
                        if ((argc - i) < 2)
                        {
                            std::cout << "Error: Incorrectly specified minimum fps." << std::endl;
                            _error = true;
                            return;
                        }
                        else
                        {
                            _min_ts = 1.0f / boost::lexical_cast<float>(argv[++i]);
                        }
                    }
                    /* Number of frame to run */
                    else if (strcmp(cur_arg, "-frames_to_run") == 0)
                    {
                        if ((argc - i) < 2)
                        {
                            std::cout << "Error: Incorrectly specified number of frames to run." << std::endl;
                            _error = true;
                            return;
                        }
                        else
                        {
                            _frames_to_run = boost::lexical_cast<int>(argv[++i]);
                        }
                    }
                    /* Pause on last frame */
                    else if (strcmp(cur_arg, "-pause_on_last") == 0)
                    {
                        _pause_on_last = true;
                    }
                    /* Run the physics only and dont render */
                    else if (strcmp(cur_arg, "-dont_render") == 0)
                    {
                        _render = false;
                    }
                    /* Multi cast frames */
                    // else if (strcmp(cur_arg, "-server") == 0)
                    // {
                    //     BOOST_LOG_TRIVIAL(trace) << "about to construct server" << std::endl;
                    //     server.reset(new raptor_networking::msg_server("0.0.0.0", "0.0.0.0", raptor_networking::MULTI_CAST_ADDR, raptor_networking::CONTROL_SEND_PORT, raptor_networking::CONTROL_RECV_PORT, raptor_networking::MULTI_CAST_PORT, 0));
                    // }
                    else
                    {
                        std::cout << "Error: Unknown arguement: " << cur_arg << std::endl;
                        _error = true;
                        return;
                    }
                }
            }
        }

        /* Access functions */
        float   max_timestep()  const { return _max_ts;         }
        float   min_timestep()  const { return _min_ts;         }
        int     frames_to_run() const { return _frames_to_run;  }
        bool    pause_on_last() const { return _pause_on_last;  }
        bool    render()        const { return _render;         }
        bool    error()         const { return _error;          }

        physics_options& max_timestep(const float s)
        {
            _max_ts = s;
            return *this;
        }

        physics_options& min_timestep(const float s)
        {
            _min_ts = s;
            return *this;
        }

        physics_options& frames_to_run(const int f)
        {
            _frames_to_run = f;
            return *this;
        }

        /* Track frames run */
        physics_options& frame_done()
        {
            if (_frames_to_run > 0)
            {
                --_frames_to_run;
            }

            return *this;
        }
        

    private :
        float   _max_ts;
        float   _min_ts;
        int     _frames_to_run;
        bool    _pause_on_last;
        bool    _render;
        bool    _error;
};
}; /* namespace raptor_physics */
