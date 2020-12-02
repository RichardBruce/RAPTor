#pragma once

/* Boost headers */
#include "boost/archive/text_oarchive.hpp"
#include "boost/archive/text_iarchive.hpp"

/* Common headers */
#include "base_camera.h"
#include "point_t.h"

/* Ray tracer headers */
#include "circle_sampler.h"
#include "ext_colour_t.h"
#include "texture_mapper.h"
#include "ray.h"

#ifdef SIMD_PACKET_TRACING
#include "packet_ray.h"
#endif


namespace raptor_raytracer
{
class camera;
}; /* namespace raptor_raytracer */

namespace boost {
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::camera *cam, const unsigned int file_version);

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::camera *cam, const unsigned int file_version);
} /* namespace serialization */
} /* namespace boost */

namespace raptor_raytracer
{
/* Enumerate the 3 light levels */
enum class light_level_t : char { scotopic = 0, mesopic = 1, photopic = 2 };

/* Enumerate the tone mappers */
enum class tone_mapping_mode_t : char { global_contrast = 1, local_histogram      = 2, local_human_histogram = 3, global_non_linear       = 4, 
                                       global_exposure = 5, global_avg_luminance = 6, global_max_luminance  = 7, global_bilateral_filter = 8, 
                                       global_ferwerda = 9, none = 0 };

/* File output functions */
void write_png_file(const std::string &file_name, unsigned char *png_data, const unsigned int x, const unsigned int y);
void read_png_file(const std::string &file_name, unsigned char *png_data, unsigned int *const x, unsigned int *const y);

/* Class representing a camera, shouldn't be copied */
class camera : public base_camera
{
    public :
        camera(const point_t<> &c, const point_t<> &x, const point_t<> &y, const point_t<> &z, const ext_colour_t &b, const float w, 
               const float h, const float t, const unsigned x_res, const unsigned y_res, const unsigned x_a_res, const unsigned y_a_res,
               const float aperture, const float focal_length) :
                base_camera(c, x, y, z, 1.0f), tm(nullptr), sampler(point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 1.0f, 0.0f)),
                image(new ext_colour_t [ (x_res * x_a_res) * (y_res * y_a_res) ]), scotopic_glare_filter(nullptr), mesopic_glare_filter(nullptr), photopic_glare_filter(nullptr), temporal_glare_filter(nullptr),
                u(point_t<>(0.0f, 0.0f, 0.0f)), l(point_t<>(0.0f, 0.0f, 0.0f)), b(b), x_m(-w), y_m(-h), x_inc((w * 2.0f)/static_cast<float>(x_res * x_a_res)), y_inc((h * 2.0f)/static_cast<float>(y_res * y_a_res)), 
                t(t), x_res(x_res * x_a_res), y_res(y_res * y_a_res), out_x_res(x_res), out_y_res(y_res), r_vec(point_t<>(0.0f, 0.0f, 0.0f)), r_angle(0.0f),
                r_pivot(point_t<>(0.0f, 0.0f, 0.0f)), time_step(0.0f), adatption_level(0.0f), aperture(aperture), focal_length(focal_length) { };

        camera(const std::vector<texture_mapper *>  *const tm, const point_t<> &u, const point_t<> &l, const point_t<> &c, 
               const point_t<> &x, const point_t<> &y, const point_t<> &z, const ext_colour_t &b, const float w, 
               const float h, const float t, const unsigned x_res, const unsigned y_res, 
               const point_t<> &r_vec = point_t<>(0.0f, 0.0f, 0.0f), const float r_angle = 0.0f, const point_t<> &r_pivot = point_t<>(0.0f, 0.0f, 0.0f),
               const unsigned x_a_res = 1, const unsigned y_a_res = 1, const float speed = 1.0f, const float time_step = 0.0f) :
            base_camera(c, x, y, z, speed), tm(tm), sampler(point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 1.0f, 0.0f)), 
            image(new ext_colour_t [ (x_res * x_a_res) * (y_res * y_a_res) ]), scotopic_glare_filter(nullptr), mesopic_glare_filter(nullptr), photopic_glare_filter(nullptr), temporal_glare_filter(nullptr),
            u(u), l(l), b(b), x_m(-w), y_m(-h), x_inc((w * 2.0f)/static_cast<float>(x_res * x_a_res)), y_inc((h * 2.0f)/static_cast<float>(y_res * y_a_res)), 
            t(t), x_res(x_res * x_a_res), y_res(y_res * y_a_res), out_x_res(x_res), out_y_res(y_res), r_vec(r_vec), r_angle(r_angle),
            r_pivot(r_pivot), time_step(time_step), adatption_level(0.0)
        {  };

        ~camera()
        {
            /* Delete the image data */
            delete [] this->image;
            
            /* Delete any glare filter images */            
            if (this->scotopic_glare_filter != nullptr)
            {
                delete [] this->scotopic_glare_filter;
            }
            
            if (this->mesopic_glare_filter != nullptr)
            {
                delete [] this->mesopic_glare_filter;
            }
            
            if (this->photopic_glare_filter != nullptr)
            {
                delete [] this->photopic_glare_filter;
            }
            
            if (this->temporal_glare_filter != nullptr)
            {
                delete [] this->temporal_glare_filter;
            }
        };
        
        /* Access function */
        unsigned x_resolution()     const   { return this->out_x_res;   }
        unsigned y_resolution()     const   { return this->out_y_res;   }
        unsigned x_number_of_rays() const   { return this->x_res;       }
        unsigned y_number_of_rays() const   { return this->y_res;       }
        
        /* Time control */
        camera& advance_time(const float t)
        {
            this->time_step = t;
            return *this;
        }
        
        /* Pixel to co-ordinate conversion */
        point_t<> pixel_to_co_ordinate(const int x, const int y, const int a_x = 0, const int a_y = 0) const
        {
            /* The camera can be anywhere so these co-ordinates are relative to it */
            /* Calaculate the direction that would pass through x,y */
            const float x_t = (static_cast<float>(x) * x_inc) + x_m;
            const float y_t = (static_cast<float>(y) * y_inc) + y_m;
            
            return (x_axis() * x_t) + (y_axis() * y_t) + (z_axis() * this->t);
        }

        int pixel_to_co_ordinate(std::vector<ray> *const rays, const int x, const int y, const int a_x = 0, const int a_y = 0, const int samples = 0)
        {
            /* Find the way the center ray would go */
            const float x_t = (static_cast<float>(x) * x_inc) + x_m;
            const float y_t = (static_cast<float>(y) * y_inc) + y_m;
            
            /* Work out where the rays should focus, if we dont need depth of focus send it back */
            const point_t<> screen((x_axis() * x_t) + (y_axis() * y_t) + (z_axis() * this->t));
            const point_t<> dir(normalise(screen));
            if ((samples <= 1) || (focal_length <= 0.0f) || (aperture <= 0.0f))
            {
                rays->resize(1);
                rays->at(0).set_up(camera_position(), dir.x, dir.y, dir.z);
                return 1;
            }

            const point_t<> focal_point(camera_position() + (dir * focal_length));
            // circle_sampler_stratified sampler(dir, y_axis() * aperture, samples);
            // rays->resize(sampler.samples());
            // for (int i = 0; i < sampler.samples(); ++i)
            rays->resize(samples);
            sampler.reset(dir, y_axis() * aperture);
            for (int i = 0; i < samples; ++i)
            {
                const point_t<> o(camera_position() + sampler.sample());
                const point_t<> adj_dir(normalise(focal_point - o));
                rays->at(i).set_up(o, adj_dir.x, adj_dir.y, adj_dir.z);
            }

            // return sampler.samples();
            return samples;
        }

        /* Convert a line from the origin into a pixel address */
        inline bool direction_to_pixel(const point_t<> &d, float *xy) const
        {
            /* Find the intersection the the line from the origin and the image plane */
            const float n_dot_d = dot_product(z_axis(), d);
            
            /* Direction perphendicular to the image plane */
            if (n_dot_d == 0.0f)
            {
                return false;
            }
            
            const point_t<> hit_point = d * (this->t / n_dot_d);
            
            /* Convert the intersection pont to a co-ordinate */
            const point_t<> plane_centre = z_axis() * t;
            const point_t<> diff = hit_point - plane_centre;
            
            xy[0] = (dot_product(x_axis(), diff) - this->x_m) / this->x_inc;
            xy[1] = (dot_product(y_axis(), diff) - this->y_m) / this->y_inc;
            
            return ((xy[0] <= this->x_res) && (xy[0] >= 0.0f) && 
                    (xy[1] <= this->y_res) && (xy[1] >= 0.0f));
        }
        
#ifdef SIMD_PACKET_TRACING
        void pixel_to_co_ordinate(packet_ray *const r, const int x, const int y) const
        {
            /* Assert packet is square */
            assert(std::fmod(std::sqrt(static_cast<float>(MAXIMUM_PACKET_SIZE)), 1.0f) == 0.0f);

            /* Assert the packet is MAXIMUM_PACKET_SIZE aligned */
            assert((x & (((unsigned int)std::sqrt(MAXIMUM_PACKET_SIZE) << 1) - 1)) == 0);
            assert((y & (((unsigned int)std::sqrt(MAXIMUM_PACKET_SIZE) << 1) - 1)) == 0);

            /* Create the packet data */
            for (unsigned int i = 0; i < MAXIMUM_PACKET_SIZE; i++)
            {
                /* Look up x, y addresses so any sub range forms a valid packet */
                const int tx = x + packet_ray_to_co_ordinate_lut.x_offset(i);
                const int ty = y + packet_ray_to_co_ordinate_lut.y_offset(i);
                
                /* Calculate the rays direction */
                vfp_t vx(tx, (tx + 1.0f), tx,         (tx + 1.0f));
                vfp_t vy(ty,  ty,        (ty + 1.0f), (ty + 1.0f));

                vfp_t vx_t((vx * vfp_t(x_inc)) + vfp_t(this->x_m));
                vfp_t vy_t((vy * vfp_t(y_inc)) + vfp_t(this->y_m));

                vfp_t vd_x(vfp_t(x_axis().x) * vx_t);
                vfp_t vd_y(vfp_t(x_axis().y) * vx_t);
                vfp_t vd_z(vfp_t(x_axis().z) * vx_t);
             
                vd_x += vfp_t(y_axis().x) * vy_t;
                vd_y += vfp_t(y_axis().y) * vy_t;
                vd_z += vfp_t(y_axis().z) * vy_t;

                vd_x += vfp_t(z_axis().x) * vfp_t(this->t);
                vd_y += vfp_t(z_axis().y) * vfp_t(this->t);
                vd_z += vfp_t(z_axis().z) * vfp_t(this->t);
             
                /* Normalise */
                vfp_t vec_len(inverse_sqrt((vd_x * vd_x) + (vd_y * vd_y) + (vd_z * vd_z)));
                vd_x *= vec_len;
                vd_y *= vec_len;
                vd_z *= vec_len;
             
                /* Initalise part of the packet */
                r[i].set_up(camera_position(), vd_x, vd_y, vd_z);
            }
        }
#endif /* #ifdef SIMD_PACKET_TRACING */
        
        /* Background shading */
        ext_colour_t shade(ray *const r) const
        {
            if (this->tm == nullptr)
            {
                return this->b;
            }
            else
            {
                /* Find the intersection point with the sky box and which plane was hit */
                point_t<> p;
                const unsigned int tm_nr = this->sky_box_intersection(*r, &p);
                r->set_dst(p);

                point_t<> n;
                switch(tm_nr)
                {
                    case 0 :
                        n = point_t<>(1.0, 0.0, 0.0);
                        break;
                    case 1 :
                        n = point_t<>(0.0, 1.0, 0.0);
                        break;
                    case 2 :
                        n = point_t<>(0.0, 0.0, 1.0);
                        break;
                    case 3 :
                        n = point_t<>(-1.0, 0.0, 0.0);
                        break;
                    case 4 :
                        n = point_t<>(0.0, -1.0, 0.0);
                        break;
                    case 5 :
                        n = point_t<>(0.0, 0.0, -1.0);
                        break;
                    default :
                        assert(!"Sky box texture mapper out of range");
                        break;
                }
                
                /* Look up the texture and texture map the pixel */
                ext_colour_t c;
                (*this->tm)[tm_nr]->texture_map(*r, &c, n, point_t<>(MAX_DIST, MAX_DIST, MAX_DIST));
                return c;
            }
        }

        /* Setting pixel colour */
        camera & set_pixel(const ext_colour_t &p, const int x, const int y)
        {
            this->image[x + (y * this->x_res)] = p;
            return *this;
        }

        /* Image output function */
        /* Downsample and set output clipped to rgb */
        void clip_image_to_rgb(unsigned char * c) const
        {
            /* Clip each pixel */
            const int x_pixel_samples = (int)(this->x_res / this->out_x_res);
            const int y_pixel_samples = (int)(this->y_res / this->out_y_res);
            const int pixel_samples   = x_pixel_samples * y_pixel_samples;
            for (int x = 0; x < (int)this->out_x_res; x++)
            {
                for (int y = 0; y < (int)this->out_y_res; y++)
                {
                    ext_colour_t a(0.0f, 0.0f, 0.0f);
                    const int pixel_addr = (x * x_pixel_samples) + (y * y_pixel_samples * this->x_res);
                    for (int a_x = 0; a_x < x_pixel_samples; a_x++)
                    {
                        for (int a_y = 0; a_y < y_pixel_samples; a_y++)
                        {
                            a.r += this->image[pixel_addr + (a_y * this->x_res) + a_x].r;
                            a.g += this->image[pixel_addr + (a_y * this->x_res) + a_x].g;
                            a.b += this->image[pixel_addr + (a_y * this->x_res) + a_x].b;
                        }
                    }

                    const int image_addr = (x + (y * this->out_x_res)) * 3;
                    c[image_addr    ] = (unsigned char)std::min(255.0f, std::max(0.0f, (a.r / pixel_samples)));
                    c[image_addr + 1] = (unsigned char)std::min(255.0f, std::max(0.0f, (a.g / pixel_samples)));
                    c[image_addr + 2] = (unsigned char)std::min(255.0f, std::max(0.0f, (a.b / pixel_samples)));
                }
            }

            return;
        }

        /* Downsample and set output clipped to bgr */
        void clip_image_to_bgr(unsigned char * c) const
        {
            /* Clip each pixel */
            const int x_pixel_samples = (int)(this->x_res / this->out_x_res);
            const int y_pixel_samples = (int)(this->y_res / this->out_y_res);
            const float pixel_samples_inv = 1.0f / (x_pixel_samples * y_pixel_samples);
            for (int x = 0; x < (int)this->out_x_res; x++)
            {
                for (int y = 0; y < (int)this->out_y_res; y++)
                {
                    ext_colour_t a(0.0f, 0.0f, 0.0f);
                    const int pixel_addr = (x * x_pixel_samples) + (y * y_pixel_samples * this->x_res);
                    for (int a_x = 0; a_x < x_pixel_samples; a_x++)
                    {
                        for (int a_y = 0; a_y < y_pixel_samples; a_y++)
                        {
                            a.r += this->image[pixel_addr + (a_y * this->x_res) + a_x].r;
                            a.g += this->image[pixel_addr + (a_y * this->x_res) + a_x].g;
                            a.b += this->image[pixel_addr + (a_y * this->x_res) + a_x].b;
                        }
                    }

                    const int image_addr = (x + (y * this->out_x_res)) * 3;
                    c[image_addr    ] = (unsigned char)std::min(255.0f, std::max(0.0f, (a.b * pixel_samples_inv)));
                    c[image_addr + 1] = (unsigned char)std::min(255.0f, std::max(0.0f, (a.g * pixel_samples_inv)));
                    c[image_addr + 2] = (unsigned char)std::min(255.0f, std::max(0.0f, (a.r * pixel_samples_inv)));
                }
            }

            return;
        }
        
        /* Write to tga file */
        const camera & write_tga_file(const std::string &file_name, unsigned char *o = nullptr) const;

        /* Write to png file */
        const camera & write_png_file(const std::string &file_name) const;

        /* Write to jpeg file */
        const camera & write_jpeg_file(const std::string &file_name, const int q, unsigned char *o = nullptr) const;


        /* Image colour correction */
        /* Exposure compensation of the image */
        camera & exposure_compensate(const float e)
        {
            /* Multiply each pixel by exposure */
            for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
            {
                this->image[i] *= e;
            }

            return *this;
        }
        
        /* Tone mapping of the image */
        camera & tone_map(const tone_mapping_mode_t tone_map, const float key = 0.18f, const float xw = (1.0f / 3.0f), const float yw = (1.0f / 3.0f), const bool gf = false, const bool ta = false, const bool ds = false, const bool sc = false);
        
        /* Gamma correction of the image */
        camera & gamma_correct(const float gamma);

    private :
        friend class boost::serialization::access;

        template<class Archive> friend void boost::serialization::save_construct_data(Archive & ar, const camera *cam, const unsigned int file_version);
        template<class Archive> friend void boost::serialization::load_construct_data(Archive & ar, camera *cam, const unsigned int file_version);

        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            /* Image must be loaded an element at a time because boost cant work out how long it is */
            const unsigned int image_size = x_res * y_res;
            for (unsigned int i = 0; i < image_size; i++)
            {
                ar & image[i];
            }
            // ar & c;
            // ar & x;
            // ar & y;
            // ar & z;
            // ar & speed;
            ar & time_step;
            ar & adatption_level;
        }

        camera(const std::vector<texture_mapper *>  *const tm, const point_t<> &u, const point_t<> &l, const ext_colour_t &b,
            const float x_m, const float y_m, const float x_inc, const float y_inc, const float t, 
            const unsigned x_res, const unsigned y_res, const unsigned out_x_res, const unsigned out_y_res, 
            const point_t<> &r_vec, const float r_angle, const point_t<> &r_pivot) : 
            base_camera(point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f), 1.0f),
            tm(tm), sampler(point_t<>(1.0f, 0.0f, 0.0f), y_axis() * aperture), image(new ext_colour_t[x_res * y_res]),
            scotopic_glare_filter(nullptr), mesopic_glare_filter(nullptr), photopic_glare_filter(nullptr), 
            temporal_glare_filter(nullptr), u(u), l(l), b(b), x_m(x_m), y_m(y_m), x_inc(x_inc), y_inc(y_inc), 
            t(t), x_res(x_res), y_res(y_res), out_x_res(out_x_res), out_y_res(out_y_res), r_vec(r_vec), 
            r_angle(r_angle), r_pivot(r_pivot)
        {  };

        unsigned int sky_box_intersection(const ray &r, point_t<> *p) const
        {
            /* Assert the ray originated inside the sky box */
            assert((r.get_x0() >= this->l.x) && (r.get_x0() <= this->u.x));
            assert((r.get_y0() >= this->l.y) && (r.get_y0() <= this->u.y));
            assert((r.get_z0() >= this->l.z) && (r.get_z0() <= this->u.z));
            
            /* Rotate the ray into the sky boxes co-ordinate system */
            ray rot_r = r.rotate(this->r_vec, this->r_pivot, this->r_angle);
            const point_t<> ogn = rot_r.get_ogn();
            const point_t<> dir = rot_r.get_dir();

            float x_dist = MAX_DIST;
            float y_dist = MAX_DIST;
            float z_dist = MAX_DIST;
            unsigned int hit_plane_x = 0;
            unsigned int hit_plane_y = 0;
            unsigned int hit_plane_z = 0;
            
            /* Calculate distances to all planes, a hit is garenteed */
            /* Only intersect with the plane in the rays forward direction */
            if (dir.x < 0.0f)
            {
                hit_plane_x = 3;
                x_dist = (this->l.x - ogn.x) / dir.x;
            }
            else if (dir.x > 0.0f)
            {
                x_dist = (this->u.x - ogn.x) / dir.x;
            }

            if (dir.y < 0.0f)
            {
                hit_plane_y = 3;
                y_dist = (this->l.y - ogn.y) / dir.y;
            }
            else if (dir.y > 0.0f)
            {
                y_dist = (this->u.y - ogn.y) / dir.y;
            }

            if (dir.z < 0.0f)
            {
                hit_plane_z = 3;
                z_dist = (this->l.z - ogn.z) / dir.z;
            }
            else if (dir.z > 0.0f)
            {
                z_dist = (this->u.z - ogn.z) / dir.z;
            }

            assert(x_dist > 0.0f);
            assert(y_dist > 0.0f);
            assert(z_dist > 0.0f);
            
            /* Find the closest intersection */
            unsigned int hit_plane;
            if (x_dist < y_dist)
            {
                if (z_dist < x_dist)
                {
                    hit_plane = hit_plane_z + 2;
                    (*p) = ogn + (dir * z_dist);
                }
                else
                {
                    hit_plane = hit_plane_x;
                    (*p) = ogn + (dir * x_dist);
                }
            }
            else
            {
                if (z_dist < y_dist)
                {
                    hit_plane = hit_plane_z + 2;
                    (*p) = ogn + (dir * z_dist);
                }
                else
                {
                    hit_plane = hit_plane_y + 1;
                    (*p) = ogn + (dir * y_dist);
                }
            }
            
            return hit_plane;
        }
        
        /* Glare filtering  */
        void generate_glare_filter(ext_colour_t **gf, const light_level_t ll);
        float generate_flare_lines(ext_colour_t *const f);
        
        void draw_circle(float *o, const int x, const int y, const float s, const float v);
        void generate_temporal_psf(const float y);

        template<typename T>
        void apply_point_spreading_function(T p, ext_colour_t *const f);
        void perform_glare_filter(const float Yi, const float Yw);

        /* Bi-lateral filtering based tone mapping */
        void bilateral_filter_tone_map(const float r_s, const float s_s, const float r_sa, const float s_sa);
       
        /* Ward's histogram based tone mapping function */
        float just_noticable_difference(const float La) const;
        void histogram_tone_map(const bool human);
        
        /* Colour space conversion */
        void convert_Yxy_to_rgb();
        void convert_xyz_to_rgb(ext_colour_t *const p, const int x, const int y) const;

        const std::vector<texture_mapper *>  * const    tm;                         /* Texture mapper for each face of the sky box                      */
        circle_sampler_random                           sampler;                    /* Sampler for depth of focus                                       */
        ext_colour_t                    *               image;                      /* The rendered image                                               */
        ext_colour_t                    *               scotopic_glare_filter;      /* Array of colours for glare filter images in scotopic lighting    */
        ext_colour_t                    *               mesopic_glare_filter;       /* Array of colours for glare filter images in mesopic lighting     */
        ext_colour_t                    *               photopic_glare_filter;      /* Array of colours for glare filter images in photopic lighting    */
        ext_colour_t                    *               temporal_glare_filter;      /* Array of colours for temporal glare filter images                */
        const point_t<>                                 u;                          /* Upper bounds of the sky box                                      */
        const point_t<>                                 l;                          /* Lower bounds of the sky box                                      */
        const ext_colour_t                              b;                          /* Background colour                                                */
        const float                                     x_m;                        /* Minimum X co-ordinate                                            */
        const float                                     y_m;                        /* Minimum y co-ordinate                                            */
        const float                                     x_inc;                      /* X increment per pixel                                            */
        const float                                     y_inc;                      /* Y increment per pixel                                            */
        const float                                     t;                          /* Distance to the image plane                                      */
        const unsigned                                  x_res;                      /* Internal X resolution                                            */
        const unsigned                                  y_res;                      /* Internal Y resolution                                            */
        const unsigned                                  out_x_res;                  /* Output X resolution                                              */
        const unsigned                                  out_y_res;                  /* Output Y resolution                                              */
        const point_t<>                                 r_vec;                      /* Sky box rotation axis                                            */
        const float                                     r_angle;                    /* Sky box rotation                                                 */
        const point_t<>                                 r_pivot;                    /* Sky box pivot point                                              */
        float                                           time_step;                  /* Time step between last and current frame in seconds              */
        float                                           adatption_level;            /* Current light level that has been adapted to                     */
        float                                           aperture;                   /* The radius of the aperture                                       */
        float                                           focal_length;               /* The focal length                                                 */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::camera *cam, const unsigned int file_version)
{
    ar << cam->tm;
    ar << cam->u;
    ar << cam->l;
    ar << cam->b;
    ar << cam->x_m;
    ar << cam->y_m;
    ar << cam->x_inc;
    ar << cam->y_inc;
    ar << cam->t;
    ar << cam->x_res;
    ar << cam->y_res;
    ar << cam->out_x_res;
    ar << cam->out_y_res;
    ar << cam->r_vec;
    ar << cam->r_angle;
    ar << cam->r_pivot;
}

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::camera *cam, const unsigned int file_version)
{
    /* Retreive the fields */
    std::vector<raptor_raytracer::texture_mapper *>  *tm;
    point_t<> u, l, r_vec, r_pivot;
    raptor_raytracer::ext_colour_t b;
    float x_m, y_m, x_inc, y_inc, t, r_angle;
    unsigned int x_res, y_res, out_x_res, out_y_res;

    ar >> tm;
    ar >> u;
    ar >> l;
    ar >> b;
    ar >> x_m;
    ar >> y_m;
    ar >> x_inc;
    ar >> y_inc;
    ar >> t;
    ar >> x_res;
    ar >> y_res;
    ar >> out_x_res;
    ar >> out_y_res;
    ar >> r_vec;
    ar >> r_angle;
    ar >> r_pivot;

    /* Use plaement new to create the class */
    ::new(cam)raptor_raytracer::camera(tm, u, l, b, x_m, y_m, x_inc, y_inc, t, x_res, y_res, out_x_res, out_y_res, r_vec, r_angle, r_pivot);
}
} /* namespace serialization */
} /* namespace boost */
