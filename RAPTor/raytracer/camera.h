#ifndef __CAMERA_H__
#define __CAMERA_H__

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/archive/text_oarchive.hpp"
#include "boost/archive/text_iarchive.hpp"

/* Ray tracer headers */
#include "point_t.h"
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
enum light_level_t { scotopic = 0, mesopic = 1, photopic = 2 };

/* Enumerate the tone mappers */
enum tone_mapping_mode_t { global_contrast = 1, local_histogram      = 2, local_human_histogram = 3, global_non_linear       = 4, 
                           global_exposure = 5, global_avg_luminance = 6, global_max_luminance  = 7, global_bilateral_filter = 8, 
                           global_ferwerda = 9, none = 0 };

/* File output functions */
void write_png_file(const string &file_name, unsigned char *png_data, const unsigned int x, const unsigned int y);
void read_png_file(const std::string &file_name, unsigned char *png_data, unsigned int *const x, unsigned int *const y);

/* Class representing a camera, shouldn't be copied */
class camera : private boost::noncopyable
{
    public :
        camera(const point_t &c, const vector_t &x, const vector_t    &y, const vector_t    &z, const ext_colour_t     &b, const     fp_t w, 
               const    fp_t  h, const     fp_t  t, const unsigned x_res, const unsigned y_res, const unsigned x_a_res = 1, const unsigned y_a_res = 1) :
                tm(nullptr), image(new ext_colour_t [ (x_res * x_a_res) * (y_res * y_a_res) ]), scotopic_glare_filter(nullptr), mesopic_glare_filter(nullptr), photopic_glare_filter(nullptr), temporal_glare_filter(nullptr),
                u(point_t((fp_t)0.0, (fp_t)0.0, (fp_t)0.0)), l(point_t((fp_t)0.0, (fp_t)0.0, (fp_t)0.0)), c(c), x(x), y(y), z(z), b(b), x_m(-w), y_m(-h), x_inc((w * (fp_t)2.0)/(fp_t)(x_res * x_a_res)), y_inc((h * (fp_t)2.0)/(fp_t)(y_res * y_a_res)), 
                t(t), x_res(x_res * x_a_res), y_res(y_res * y_a_res), out_x_res(x_res), out_y_res(y_res), r_vec(point_t(0.0, 0.0, 0.0)), r_angle(0.0),
                r_pivot(point_t(0.0, 0.0, 0.0)), speed(1.0), time_step(0.0), adatption_level(0.0)
        {
#ifdef LOG_DEPTH                
            this->depth_map = new fp_t[ this->x_res * this->y_res ];
#endif
        };

        camera(const vector<texture_mapper *>  * const tm, const point_t &u, const point_t &l, const point_t &c, 
               const vector_t &x, const vector_t    &y, const vector_t    &z, const ext_colour_t     &b, const     fp_t w, 
               const fp_t  h, const     fp_t  t, const unsigned x_res, const unsigned y_res, 
               const point_t &r_vec = point_t(0.0, 0.0, 0.0), const fp_t r_angle = 0.0, const point_t &r_pivot = point_t(0.0, 0.0, 0.0),
               const unsigned  x_a_res = 1, const unsigned y_a_res = 1, const fp_t speed = 1.0, const fp_t time_step = 0.0)
            : tm(tm), image(new ext_colour_t [ (x_res * x_a_res) * (y_res * y_a_res) ]), scotopic_glare_filter(nullptr), mesopic_glare_filter(nullptr), photopic_glare_filter(nullptr), temporal_glare_filter(nullptr),
            u(u), l(l), c(c), x(x), y(y), z(z), b(b), x_m(-w), y_m(-h), x_inc((w * (fp_t)2.0)/(fp_t)(x_res * x_a_res)), y_inc((h * (fp_t)2.0)/(fp_t)(y_res * y_a_res)), 
            t(t), x_res(x_res * x_a_res), y_res(y_res * y_a_res), out_x_res(x_res), out_y_res(y_res), r_vec(r_vec), r_angle(r_angle),
            r_pivot(r_pivot), speed(speed), time_step(time_step), adatption_level(0.0)
        {
#ifdef LOG_DEPTH                
            this->depth_map = new fp_t[ this->x_res * this->y_res ];
#endif
        };

        ~camera()
        {
            /* Delete the image data */
            delete [] this->image;
            
#ifdef LOG_DEPTH
            /* Delete the depth map */
            delete [] this->depth_map;
#endif

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
        point_t  camera_position()  const   { return this->c;           }
        unsigned x_resolution()     const   { return this->out_x_res;   }
        unsigned y_resolution()     const   { return this->out_y_res;   }
        unsigned x_number_of_rays() const   { return this->x_res;       }
        unsigned y_number_of_rays() const   { return this->y_res;       }
        
        /* Time control */
        camera& advance_time(const fp_t t)
        {
            this->time_step = t;
            return *this;
        }

        /* Speed control */
        camera& speed_up()
        {
            this->speed *= (fp_t)2.0;
            return *this;
        }
        
        camera& slow_down()
        {
            this->speed /= (fp_t)2.0;
            return *this;
        }

        /* Camera movement */
        camera& move_to(const point_t &p)
        {
            this->c = p;
            return *this;
        }
        
        camera& move_forward(const fp_t d = 1.0)
        {
            this->c += this->z * speed * d;
            return *this;
        }
        
        camera& move_up(const fp_t d = 1.0)
        {
            this->c += this->y * speed * d;
            return *this;
        }
        
        camera& move_right(const fp_t d = 1.0)
        {
            this->c += this->x * speed * d;
            return *this;
        }
        
        /* Camera rotate */
        camera& angle_to(const point_t &x, const point_t &y, const point_t &z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            return *this;
        }
        
        /* Rotate about y */
        camera& pan(const fp_t a)
        {
            rotate_about_origin(&this->x, &this->y, speed * a);
            rotate_about_origin(&this->z, &this->y, speed * a);
            return *this;
        }
    
        /* Rotate about x */
        camera& tilt(const fp_t a)
        {
            rotate_about_origin(&this->y, &this->x, speed * a);
            rotate_about_origin(&this->z, &this->x, speed * a);
            return *this;
        }

        /* Rotate about z */
        camera& roll(const fp_t a)
        {
            rotate_about_origin(&this->x, &this->z, speed * a);
            rotate_about_origin(&this->y, &this->z, speed * a);
            return *this;
        }
        
        /* Rotate about an arbitary axis */
        camera& rotate_about(const point_t &v, const fp_t a)
        {
            rotate_about_origin(&this->x, &v, a);
            rotate_about_origin(&this->y, &v, a);
            rotate_about_origin(&this->z, &v, a);
            return *this;
        }
        
        /* Pixel to co-ordinate conversion */
        point_t pixel_to_co_ordinate(const int x, const int y, const int a_x = 0, const int a_y = 0) const
        {
            /* The camera can be anywhere so these co-ordinates are relative to it */
            /* Calaculate the direction that would pass through x,y */
            const fp_t x_t = ((fp_t)x * x_inc) + x_m;
            const fp_t y_t = ((fp_t)y * y_inc) + y_m;
            
            return (this->x * x_t) + (this->y * y_t) + (this->z * this->t);
        }

        /* Convert a line from the origin into a pixel address */
        inline bool direction_to_pixel(const point_t &d, fp_t *xy) const
        {
            /* Find the intersection the the line from the origin and the image plane */
            const fp_t n_dot_d      = dot_product(this->z, d);
            //const fp_t n_dot_dist   = dot_product(this->z, (this->z * this->t));
            
            /* Direction perphendicular to the image plane */
            if (n_dot_d == (fp_t)0.0)
            {
                return false;
            }
            
            const point_t hit_point = d * (this->t / n_dot_d);
            
            /* Convert the intersection pont to a co-ordinate */
            const point_t plane_centre = this->z * t;
            const point_t diff = hit_point - plane_centre;
            
            xy[0] = (dot_product(this->x, diff) - this->x_m) / this->x_inc;
            xy[1] = (dot_product(this->y, diff) - this->y_m) / this->y_inc;
            
            return ((xy[0] <= this->x_res) && (xy[0] >= (fp_t)0.0) && 
                    (xy[1] <= this->y_res) && (xy[1] >= (fp_t)0.0));
        }
        
#ifdef SIMD_PACKET_TRACING
        void pixel_to_co_ordinate(packet_ray *const r, const int x, const int y) const
        {
            /* Assert packet is square */
            assert(fmod(sqrt((fp_t)MAXIMUM_PACKET_SIZE), (fp_t)1.0) == (fp_t)0.0);

            /* Assert the packet is MAXIMUM_PACKET_SIZE aligned */
            assert((x & (((unsigned int)sqrt(MAXIMUM_PACKET_SIZE) << 1) - 1)) == 0);
            assert((y & (((unsigned int)sqrt(MAXIMUM_PACKET_SIZE) << 1) - 1)) == 0);

            /* Create the packet data */
            for (unsigned int i = 0; i < MAXIMUM_PACKET_SIZE; i++)
            {
                /* Look up x, y addresses so any sub range forms a valid packet */
                const int tx = x + packet_ray_to_co_ordinate_lut.x_offset(i);
                const int ty = y + packet_ray_to_co_ordinate_lut.y_offset(i);
                
                /* Calculate the rays direction */
                vfp_t vx(tx, (tx + 1), tx,      (tx + 1));
                vfp_t vy(ty,  ty,     (ty + 1), (ty + 1));

                vfp_t vx_t = (vx * vfp_t(x_inc)) + vfp_t(this->x_m);
                vfp_t vy_t = (vy * vfp_t(y_inc)) + vfp_t(this->y_m); 

                vfp_t vd_x(vfp_t(this->x.x) * vx_t);
                vfp_t vd_y(vfp_t(this->x.y) * vx_t);
                vfp_t vd_z(vfp_t(this->x.z) * vx_t);
             
                vd_x += vfp_t(this->y.x) * vy_t;
                vd_y += vfp_t(this->y.y) * vy_t;
                vd_z += vfp_t(this->y.z) * vy_t;

                vd_x += vfp_t(this->z.x) * vfp_t(this->t);
                vd_y += vfp_t(this->z.y) * vfp_t(this->t);
                vd_z += vfp_t(this->z.z) * vfp_t(this->t);
             
                /* Normalise */
                vfp_t vec_len = inverse_sqrt((vd_x * vd_x) + (vd_y * vd_y) + (vd_z * vd_z));
                vd_x *= vec_len;
                vd_y *= vec_len;
                vd_z *= vec_len;
             
                /* Initalise part of the packet */
                r[i].set_up(this->c, vd_x, vd_y, vd_z);
            }
        }
#endif /* #ifdef SIMD_PACKET_TRACING */
        
        /* Background shading */
        ext_colour_t shade(const ray &r) const
        {
            if (this->tm == nullptr)
            {
                return this->b;
            }
            else
            {
                /* Find the intersection point with the sky box and which plane was hit */
                point_t p;
                unsigned int tm_nr = this->sky_box_intersection(r, &p);
                
                /* Look up the texture and texture map the pixel */
                ext_colour_t c;
                (*this->tm)[tm_nr]->texture_map(p, r.get_dir(), &c, point_t(MAX_DIST, MAX_DIST, MAX_DIST));
                return c;
            }
        }
        
        /* Dump function */
        void print_position_data() const
        {
            cout << "--cam " << this->c.x   << " "  << this->c.y << " " << this->c.z << endl;
            cout << "--dx  " << this->x.x   << " "  << this->x.y << " " << this->x.z << endl;
            cout << "--dy  " << this->y.x   << " "  << this->y.y << " " << this->y.z << endl;
            cout << "--dz  " << this->z.x   << " "  << this->z.y << " " << this->z.z << endl;
            cout << "speed " << this->speed                                          << endl;
        }
        
        /* Setting pixel colour */
        camera & set_pixel(const ext_colour_t &p, const int x, const int y)
        {
            this->image[x + (y * this->x_res)] = p;
            return *this;
        }

        /* Setting depth */
        camera & set_pixel(const fp_t p, const int x, const int y)
        {
#ifdef LOG_DEPTH
            this->depth_map[x + (y * this->x_res)] = p;
#endif
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
                    ext_colour_t a((fp_t)0.0, (fp_t)0.0, (fp_t)0.0);
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
                    c[image_addr    ] = (unsigned char)min((fp_t)255.0, max((fp_t)0.0, (a.r / pixel_samples)));
                    c[image_addr + 1] = (unsigned char)min((fp_t)255.0, max((fp_t)0.0, (a.g / pixel_samples)));
                    c[image_addr + 2] = (unsigned char)min((fp_t)255.0, max((fp_t)0.0, (a.b / pixel_samples)));
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
            const fp_t pixel_samples_inv = 1.0 / (x_pixel_samples * y_pixel_samples);
            for (int x = 0; x < (int)this->out_x_res; x++)
            {
                for (int y = 0; y < (int)this->out_y_res; y++)
                {
                    ext_colour_t a((fp_t)0.0, (fp_t)0.0, (fp_t)0.0);
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
                    c[image_addr    ] = (unsigned char)min((fp_t)255.0, max((fp_t)0.0, (a.b * pixel_samples_inv)));
                    c[image_addr + 1] = (unsigned char)min((fp_t)255.0, max((fp_t)0.0, (a.g * pixel_samples_inv)));
                    c[image_addr + 2] = (unsigned char)min((fp_t)255.0, max((fp_t)0.0, (a.r * pixel_samples_inv)));
                }
            }

            return;
        }
        
#ifdef LOG_DEPTH
        /* Depth map */
        camera & write_depth_map(const string &file_name);
#endif

        /* Write to tga file */
        const camera & write_tga_file(const string &file_name, unsigned char *o = nullptr) const;

        /* Write to png file */
        const camera & write_png_file(const string &file_name) const;

        /* Write to jpeg file */
        const camera & write_jpeg_file(const string &file_name, const int q, unsigned char *o = nullptr) const;


        /* Image colour correction */
        /* Exposure compensation of the image */
        camera & exposure_compensate(const fp_t e)
        {
            /* Multiply each pixel by exposure */
            for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
            {
                this->image[i] *= e;
            }

            return *this;
        }
        
        /* Tone mapping of the image */
        camera & tone_map(const tone_mapping_mode_t tone_map, const fp_t key = 0.18, const fp_t xw = (1.0/3.0), const fp_t yw = (1.0/3.0), const bool gf = false, const bool ta = false, const bool ds = false, const bool sc = false);
        
        /* Gamma correction of the image */
        camera & gamma_correct(const fp_t gamma);

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
#ifdef LOG_DEPTH                
            ar & depth_map;
#endif
            ar & c;
            ar & x;
            ar & y;
            ar & z;
            ar & speed;
            ar & time_step;
            ar & adatption_level;
        }

        camera(const vector<texture_mapper *>  *const tm, const point_t &u, const point_t &l, const ext_colour_t &b,
            const fp_t x_m, const fp_t y_m, const fp_t x_inc, const fp_t y_inc, const fp_t t, 
            const unsigned x_res, const unsigned y_res, const unsigned out_x_res, const unsigned out_y_res, 
            const point_t &r_vec, const fp_t r_angle, const point_t &r_pivot)
            : tm(tm), image(new ext_colour_t[x_res * y_res]), 
#ifdef LOG_DEPTH                
            depth_map(nullptr),
#endif
            scotopic_glare_filter(nullptr), mesopic_glare_filter(nullptr), photopic_glare_filter(nullptr), 
            temporal_glare_filter(nullptr), u(u), l(l), b(b), x_m(x_m), y_m(y_m), x_inc(x_inc), y_inc(y_inc), 
            t(t), x_res(x_res), y_res(y_res), out_x_res(out_x_res), out_y_res(out_y_res), r_vec(r_vec), 
            r_angle(r_angle), r_pivot(r_pivot)
        {  };

        unsigned int sky_box_intersection(const ray &r, point_t *p) const
        {
            /* Assert the ray originated inside the sky box */
            assert((r.get_x0() >= this->l.x) && (r.get_x0() <= this->u.x));
            assert((r.get_y0() >= this->l.y) && (r.get_y0() <= this->u.y));
            assert((r.get_z0() >= this->l.z) && (r.get_z0() <= this->u.z));
            
            /* Rotate the ray into the sky boxes co-ordinate system */
            ray rot_r = r.rotate(this->r_vec, this->r_pivot, this->r_angle);
            const point_t ogn = rot_r.get_ogn();
            const point_t dir = rot_r.get_dir();

            fp_t x_dist = MAX_DIST;
            fp_t y_dist = MAX_DIST;
            fp_t z_dist = MAX_DIST;
            unsigned int hit_plane_x = 0;
            unsigned int hit_plane_y = 0;
            unsigned int hit_plane_z = 0;
            
            /* Calculate distances to all planes, a hit is garenteed */
            /* Only intersect with the plane in the rays forward direction */
            if (dir.x < 0.0)
            {
                hit_plane_x = 3;
                x_dist = (this->l.x - ogn.x) / dir.x;
            }
            else if (dir.x > 0.0)
            {
                x_dist = (this->u.x - ogn.x) / dir.x;
            }

            if (dir.y < 0.0)
            {
                hit_plane_y = 3;
                y_dist = (this->l.y - ogn.y) / dir.y;
            }
            else if (dir.y > 0.0)
            {
                y_dist = (this->u.y - ogn.y) / dir.y;
            }

            if (dir.z < 0.0)
            {
                hit_plane_z = 3;
                z_dist = (this->l.z - ogn.z) / dir.z;
            }
            else if (dir.z > 0.0)
            {
                z_dist = (this->u.z - ogn.z) / dir.z;
            }

            assert(x_dist > (fp_t)0.0);
            assert(y_dist > (fp_t)0.0);
            assert(z_dist > (fp_t)0.0);
            
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
        fp_t generate_flare_lines(ext_colour_t *const f);
        
        void draw_circle(fp_t *o, const int x, const int y, const fp_t s, const fp_t v);
        void generate_temporal_psf(const fp_t y);

        template<typename T>
        void apply_point_spreading_function(T p, ext_colour_t *const f);
        void perform_glare_filter(const fp_t Yi, const fp_t Yw);

        /* Bi-lateral filtering based tone mapping */
        void bilateral_filter_tone_map(const fp_t r_s, const fp_t s_s, const fp_t r_sa, const fp_t s_sa);
       
        /* Ward's histogram based tone mapping function */
        fp_t just_noticable_difference(const fp_t La) const;
        void histogram_tone_map(const bool human);
        
        /* Colour space conversion */
        void convert_Yxy_to_rgb();
        void convert_xyz_to_rgb(ext_colour_t *const p, const int x, const int y) const;

        const vector<texture_mapper *>  * const     tm;                         /* Texture mapper for each face of the sky box                      */
        ext_colour_t                    *           image;                      /* The rendered image                                               */
        ext_colour_t                    *           scotopic_glare_filter;      /* Array of colours for glare filter images in scotopic lighting    */
        ext_colour_t                    *           mesopic_glare_filter;       /* Array of colours for glare filter images in mesopic lighting     */
        ext_colour_t                    *           photopic_glare_filter;      /* Array of colours for glare filter images in photopic lighting    */
        ext_colour_t                    *           temporal_glare_filter;      /* Array of colours for temporal glare filter images                */
#ifdef LOG_DEPTH                
        fp_t                            *           depth_map;                  /* Per pixel ray depths for 3D reconstruction                       */
#endif
        const point_t                               u;                          /* Upper bounds of the sky box                                      */
        const point_t                               l;                          /* Lower bounds of the sky box                                      */
        point_t                                     c;                          /* Camera position                                                  */
        vector_t                                    x;                          /* Horizontal axis                                                  */
        vector_t                                    y;                          /* Vertical axis                                                    */
        vector_t                                    z;                          /* Forward axis                                                     */
        const ext_colour_t                          b;                          /* Background colour                                                */
        const fp_t                                  x_m;                        /* Minimum X co-ordinate                                            */
        const fp_t                                  y_m;                        /* Minimum y co-ordinate                                            */
        const fp_t                                  x_inc;                      /* X increment per pixel                                            */
        const fp_t                                  y_inc;                      /* Y increment per pixel                                            */
        const fp_t                                  t;                          /* Distance to the image plane                                      */
        const unsigned                              x_res;                      /* Internal X resolution                                            */
        const unsigned                              y_res;                      /* Internal Y resolution                                            */
        const unsigned                              out_x_res;                  /* Output X resolution                                              */
        const unsigned                              out_y_res;                  /* Output Y resolution                                              */
        const point_t                               r_vec;                      /* Sky box rotation axis                                            */
        const fp_t                                  r_angle;                    /* Sky box rotation                                                 */
        const point_t                               r_pivot;                    /* Sky box pivot point                                              */
        fp_t                                        speed;                      /* Speed of movement                                                */
        fp_t                                        time_step;                  /* Time step between last and current frame in seconds              */
        fp_t                                        adatption_level;            /* Current light level that has been adapted to                     */
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
    vector<raptor_raytracer::texture_mapper *>  *tm;
    point_t u, l, r_vec, r_pivot;
    raptor_raytracer::ext_colour_t b;
    fp_t x_m, y_m, x_inc, y_inc, t, r_angle;
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

#endif /* #ifndef __CAMERA_H__ */
