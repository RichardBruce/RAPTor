#ifndef __PACKET_RAY_H__
#define __PACKET_RAY_H__

#include "simd.h"
#include "ray.h"

#ifdef SIMD_PACKET_TRACING


namespace raptor_raytracer
{
/* Forward declarations */
class triangle;

class packet_ray
{
    public :
        /* Constructor if we havent intersected anything yet */
        packet_ray(const point_t &o, const vfp_t &dx, const vfp_t &dy, const vfp_t &dz, fp_t m=1.0, int c=1) : 
            magn(m), componant(c) 
            { 
                this->ogn[0] = o.x;
                this->ogn[1] = o.y;
                this->ogn[2] = o.z;
                this->dir[0] = dx;
                this->dir[1] = dy;
                this->dir[2] = dz;
            };
        
        /* Empty constructor */
        packet_ray() {  };
        
        void set_up(const point_t &o, const vfp_t &dx, const vfp_t &dy, const vfp_t &dz, const fp_t m=1.0, const int c=1)
        { 
            magn        = vfp_t(m);
            componant   = vfp_t((fp_t)c);

            this->ogn[0] = o.x;
            this->ogn[1] = o.y;
            this->ogn[2] = o.z;
            this->dir[0] = dx;
            this->dir[1] = dy;
            this->dir[2] = dz;
            
            return;
        }
        
        void set_up(const vfp_t &ox, const vfp_t &oy, const vfp_t &oz, const vfp_t &dx, const vfp_t &dy, const vfp_t &dz, const fp_t m=1.0, const int c=1)
        { 
            magn        = vfp_t(m);
            componant   = vfp_t((fp_t)c);

            this->ogn[0] = ox;
            this->ogn[1] = oy;
            this->ogn[2] = oz;
            this->dir[0] = dx;
            this->dir[1] = dy;
            this->dir[2] = dz;
            
            return;
        }
        
        packet_ray & calculate_destination(const vfp_t &d)
        {
            this->length = d;
            this->dst[0] = this->ogn[0] + (this->dir[0] * d);
            this->dst[1] = this->ogn[1] + (this->dir[1] * d);
            this->dst[2] = this->ogn[2] + (this->dir[2] * d);
            return *this;
        }
        
        vfp_t pack(const ray *const *const r)
        {
            this->ogn[0]    = vfp_t(r[0]->get_x0(), r[1]->get_x0(), r[2]->get_x0(), r[3]->get_x0());
            this->ogn[1]    = vfp_t(r[0]->get_y0(), r[1]->get_y0(), r[2]->get_y0(), r[3]->get_y0());
            this->ogn[2]    = vfp_t(r[0]->get_z0(), r[1]->get_z0(), r[2]->get_z0(), r[3]->get_z0());

            this->dir[0]    = vfp_t(r[0]->get_x_grad(), r[1]->get_x_grad(), r[2]->get_x_grad(), r[3]->get_x_grad());
            this->dir[1]    = vfp_t(r[0]->get_y_grad(), r[1]->get_y_grad(), r[2]->get_y_grad(), r[3]->get_y_grad());
            this->dir[2]    = vfp_t(r[0]->get_z_grad(), r[1]->get_z_grad(), r[2]->get_z_grad(), r[3]->get_z_grad());

            this->dst[0]    = vfp_t(r[0]->get_x1(), r[1]->get_x1(), r[2]->get_x1(), r[3]->get_x1());
            this->dst[1]    = vfp_t(r[0]->get_y1(), r[1]->get_y1(), r[2]->get_y1(), r[3]->get_y1());
            this->dst[2]    = vfp_t(r[0]->get_z1(), r[1]->get_z1(), r[2]->get_z1(), r[3]->get_z1());
            
            this->magn      = vfp_t(r[0]->get_magnitude(), r[1]->get_magnitude(), r[2]->get_magnitude(), r[3]->get_magnitude());
            this->componant = vfp_t((fp_t)r[0]->get_componant(), (fp_t)r[1]->get_componant(), (fp_t)r[2]->get_componant(), (fp_t)r[3]->get_componant());

            return vfp_t(r[0]->get_length(), r[1]->get_length(), r[2]->get_length(), r[3]->get_length());
        }
        
        
        /* Access functions */
        /* It is a very bad idea to use this function */
        ray extract(const unsigned i) const
        {
            assert(i < SIMD_WIDTH);
            
            return ray(point_t(this->ogn[0][i], this->ogn[1][i], this->ogn[2][i]), 
                       point_t(this->dst[0][i], this->dst[1][i], this->dst[2][i]), 
                       point_t(this->dir[0][i], this->dir[1][i], this->dir[2][i]), 
                       this->length[i], this->magn[i], (int)this->componant[i]);
        }
        
        const packet_ray & extract(ray *const r) const
        {
            /* Extract simd data */
            const fp_t *ogn_x = this->ogn[0];
            const fp_t *ogn_y = this->ogn[1];
            const fp_t *ogn_z = this->ogn[2];

            const fp_t *dir_x = this->dir[0];
            const fp_t *dir_y = this->dir[1];
            const fp_t *dir_z = this->dir[2];

            const fp_t *dst_x = this->dst[0];
            const fp_t *dst_y = this->dst[1];
            const fp_t *dst_z = this->dst[2];

            const fp_t *len   = this->length;
            
            const fp_t *magn  = this->magn;
            const fp_t *comp  = this->componant;
            
            r[0].set_up(point_t(ogn_x[0], ogn_y[0], ogn_z[0]), 
                        point_t(dst_x[0], dst_y[0], dst_z[0]), 
                        point_t(dir_x[0], dir_y[0], dir_z[0]), 
                        len[0], magn[0], (int)comp[0]);
                       
            r[1].set_up(point_t(ogn_x[1], ogn_y[1], ogn_z[1]), 
                        point_t(dst_x[1], dst_y[1], dst_z[1]), 
                        point_t(dir_x[1], dir_y[1], dir_z[1]), 
                        len[1], magn[1], (int)comp[1]);
                       
            r[2].set_up(point_t(ogn_x[2], ogn_y[2], ogn_z[2]), 
                        point_t(dst_x[2], dst_y[2], dst_z[2]), 
                        point_t(dir_x[2], dir_y[2], dir_z[2]), 
                        len[2], magn[2], (int)comp[2]);
                       
            r[3].set_up(point_t(ogn_x[3], ogn_y[3], ogn_z[3]), 
                        point_t(dst_x[3], dst_y[3], dst_z[3]), 
                        point_t(dir_x[3], dir_y[3], dir_z[3]), 
                        len[3], magn[3], (int)comp[3]);
            return *this;
        }
        
        vfp_t get_x0()                          const   { return this->ogn[0];  	};
        vfp_t get_y0()                          const   { return this->ogn[1];  	};
        vfp_t get_z0()                          const   { return this->ogn[2];  	};
        vfp_t get_x_grad()                      const   { return this->dir[0];  	};
        vfp_t get_y_grad()                      const   { return this->dir[1];  	};
        vfp_t get_z_grad()                      const   { return this->dir[2];  	};
        vfp_t get_ogn(const unsigned int xyz)   const   { return this->ogn[xyz];	};
        vfp_t get_dir(const unsigned int xyz)   const   { return this->dir[xyz];	};
        vfp_t get_dst(const unsigned int xyz)   const   { return this->dst[xyz];	};
        
    private : 
        vfp_t   ogn[3];
        vfp_t   dst[3];
        vfp_t   dir[3];
        vfp_t   length;
        vfp_t   mask;
        vfp_t   magn;
        vfp_t   componant;
};


/* A struct to hold the hit information for a packet ray */
struct packet_hit_description
{
    packet_hit_description(const vfp_t &d = MAX_DIST) : u((fp_t)0.0), v((fp_t)0.0), d(d) {  };
    
    hit_description operator[](int i)   const   { return hit_description(d[i], miss, u[i], v[i]);   };

    const packet_hit_description & extract(hit_description *const h) const
    {
        /* Extract simd data */
        const fp_t *fp_u = this->u;
        const fp_t *fp_v = this->v;
        const fp_t *fp_d = this->d;

        h[0] = hit_description(fp_d[0], miss, fp_u[0], fp_v[0]);
        h[1] = hit_description(fp_d[1], miss, fp_u[1], fp_v[1]);
        h[2] = hit_description(fp_d[2], miss, fp_u[2], fp_v[2]);
        h[3] = hit_description(fp_d[3], miss, fp_u[3], fp_v[3]);
        return *this;
    }

    vfp_t       u;      /* Barycentric u co-ordinate    */
    vfp_t       v;      /* Barycentric u co-ordinate    */
    vfp_t       d;      /* Distance                     */
};


/**********************************************************
 packet_ray_to_pixel provides a lut for ray address to 
 pixel address conversion.
 
 The ray address should be looked up based on the pixel
 address.
**********************************************************/
class packet_ray_to_pixel
{
    public :
        packet_ray_to_pixel()
        {
            const unsigned int pkt_width = (unsigned int)std::sqrt((fp_t)(MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH));

            /* Foreach ray in the biggest packet */
            for (unsigned int i = 0; i < (unsigned int)(MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH); i++)
            {
                /* Generate x, y addresses of the ray number i in the packet */
                unsigned int x = 0;
                unsigned int y = 0;
                
                unsigned int addr = i;
                unsigned int mask = 0x1;
                for (unsigned int j = 0; j < pkt_width; j++)
                {
                    x += (addr & mask);
                    addr >>= 1;
                    
                    y += (addr & mask);
                    mask <<= 1;
                }
                
                /* convert x, y address to linear address and set entry to i */
                this->lut[i] = (y * pkt_width) + x;
            }        
        }
        
        
        /* Look up */
        inline unsigned int operator[](unsigned int i)  const   { return this->lut[i];  }
        
        inline operator const unsigned int *const()     const   { return &this->lut[0]; }

    private :
        unsigned int lut[(MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH)];
};

extern const packet_ray_to_pixel packet_ray_to_pixel_lut;


/**********************************************************
 packet_ray_to_co_ordinate provides a lut for packet 
 number to (x,y) co-rodinate conversion.
 
 (x,y) co-rodinates are looked up based of the packet
 address 'i'. This address should be added to packet 0
 (x,y) co-rodinates.
**********************************************************/
class packet_ray_to_co_ordinate
{
    public :
        packet_ray_to_co_ordinate()
        {
            const unsigned int pkt_width = (unsigned int)std::sqrt((fp_t)MAXIMUM_PACKET_SIZE) >> 1;

            /* Foreach the biggest packet size */
            for (unsigned int i = 0; i < MAXIMUM_PACKET_SIZE; i++)
            {
                /* Generate x, y addresses so any sub range forms a valid packet */
                unsigned int x = 0;
                unsigned int y = 0;
                
                unsigned int addr = i << 1;
                unsigned int mask = 0x2;
                for (unsigned int j = 0; j < pkt_width; j++)
                {
                    x += (addr & mask);
                    addr >>= 1;
                    
                    y += (addr & mask);
                    mask <<= 1;
                }
                
                this->x_lut[i] = x;
                this->y_lut[i] = y;
            }       
        }
        
        
        /* Look up */
        unsigned int x_offset(unsigned int i) const { return this->x_lut[i];    }
        unsigned int y_offset(unsigned int i) const { return this->y_lut[i];    }

    private :
        unsigned int x_lut[MAXIMUM_PACKET_SIZE];
        unsigned int y_lut[MAXIMUM_PACKET_SIZE];
};

extern const packet_ray_to_co_ordinate packet_ray_to_co_ordinate_lut;

}; /* namespace raptor_raytracer */
#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifndef __PACKET_RAY_H__ */
