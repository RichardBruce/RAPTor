/* Standard headers */
#include <cmath>
#include <map>
#include <string>

/* Event handlers */
#include "SDL.h"
#include "sdl_event_handler.h"
#include "raytracer_event_handler_factory.h"


namespace raptor_raytracer
{
/* Function to add handlers for the ray tracer camera mappings of SDL_Keycode */
/*  cam is the camera that will be updated. ownership is not taken of the camera */
/*  a pointer to an sdl_event_handler<camera> is returned */
void append_raytracer_camera_event_handler(std::map<SDL_Keycode, sdl_event_handler::key_handler_function> *const handler_map, camera *const cam, const std::string &output_file, const int jpg_quality, const image_format_t image_format)
{
    using handler_function = sdl_event_handler::key_handler_function;
    handler_map->emplace(SDLK_f, [cam, &output_file, jpg_quality, image_format](const SDL_Keycode key, const float)
        {
            static unsigned int snapshot_nr = 0;
            std::ostringstream file_name(std::ostringstream::out);

            /* Tone map */
            cam->tone_map(tone_mapping_mode_t::local_human_histogram);
            
            /* Output image */
            switch (image_format)
            {
                case image_format_t::tga :
                    file_name << output_file << "_" << snapshot_nr++ << ".tga";
                    cam->write_tga_file(file_name.str());
                    break;
                case image_format_t::jpg :
                    file_name << output_file << "_" << snapshot_nr++ << ".jpg";
                    cam->write_jpeg_file(file_name.str(), jpg_quality);
                    break;
                case image_format_t::png :
                    file_name << output_file << "_" << snapshot_nr++ << ".png";
                    cam->write_png_file(file_name.str());
                    break;
                default :
                    assert(!"Error unknown image format");
                    break;
            }

            return 0;
        });
}
} /* using raptor_raytracer */
