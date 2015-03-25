#ifndef __FRUSTRUM_H__
#define __FRUSTRUM_H__

/* Ray tracer headers */
#include "simd.h"
#include "ray.h"
#include "packet_ray.h"

#ifdef SIMD_PACKET_TRACING

namespace raptor_raytracer
{
/* Frustrum class to represent an array of rays by proxy */
class frustrum
{
    public :
        frustrum(const packet_ray *r, const unsigned s)
        {
            /* REVISIT -- This over estimates the spread of the frustrum */
            /*            Use scenes exit point for gradient             */
            /* SIMD max mins */
            vfp_t min_x_ogn = r[0].get_x0();
            vfp_t min_y_ogn = r[0].get_y0();
            vfp_t min_z_ogn = r[0].get_z0();
            vfp_t min_x_dir = r[0].get_x_grad();
            vfp_t min_y_dir = r[0].get_y_grad();
            vfp_t min_z_dir = r[0].get_z_grad();

            vfp_t max_x_ogn = r[0].get_x0();
            vfp_t max_y_ogn = r[0].get_y0();
            vfp_t max_z_ogn = r[0].get_z0();
            vfp_t max_x_dir = r[0].get_x_grad();
            vfp_t max_y_dir = r[0].get_y_grad();
            vfp_t max_z_dir = r[0].get_z_grad();
            for (unsigned i = 1; i < s; i++)
            {
                min_x_ogn = min(min_x_ogn, r[i].get_x0());
                min_y_ogn = min(min_y_ogn, r[i].get_y0()); 
                min_z_ogn = min(min_z_ogn, r[i].get_z0());
                min_x_dir = min(min_x_dir, r[i].get_x_grad());
                min_y_dir = min(min_y_dir, r[i].get_y_grad());
                min_z_dir = min(min_z_dir, r[i].get_z_grad());

                max_x_ogn = max(max_x_ogn, r[i].get_x0());
                max_y_ogn = max(max_y_ogn, r[i].get_y0());
                max_z_ogn = max(max_z_ogn, r[i].get_z0());
                max_x_dir = max(max_x_dir, r[i].get_x_grad());
                max_y_dir = max(max_y_dir, r[i].get_y_grad());
                max_z_dir = max(max_z_dir, r[i].get_z_grad());
            }
            
            /* Max and min accross vector */
            _x_data[0] = horizontal_min(min_x_ogn);
            _y_data[0] = horizontal_min(min_y_ogn);
            _z_data[0] = horizontal_min(min_z_ogn);
            _x_data[2] = 1.0f / horizontal_min(min_x_dir);
            _y_data[2] = 1.0f / horizontal_min(min_y_dir);
            _z_data[2] = 1.0f / horizontal_min(min_z_dir);
            
            _x_data[1] = horizontal_max(max_x_ogn);
            _y_data[1] = horizontal_max(max_y_ogn);
            _z_data[1] = horizontal_max(max_z_ogn);
            _x_data[3] = 1.0f / horizontal_max(max_x_dir);
            _y_data[3] = 1.0f / horizontal_max(max_y_dir);
            _z_data[3] = 1.0f / horizontal_max(max_z_dir);
            
            this->mm_ogn[0] = vfp_t(_x_data[0], _x_data[0], _x_data[1], _x_data[1]);
            this->mm_ogn[1] = vfp_t(_y_data[0], _y_data[0], _y_data[1], _y_data[1]);
            this->mm_ogn[2] = vfp_t(_z_data[0], _z_data[0], _z_data[1], _z_data[1]);
            
            this->mm_dir[0] = vfp_t(_x_data[2], _x_data[2], _x_data[3], _x_data[3]);
            this->mm_dir[1] = vfp_t(_y_data[2], _y_data[2], _y_data[3], _y_data[3]);
            this->mm_dir[2] = vfp_t(_z_data[2], _z_data[2], _z_data[3], _z_data[3]);
            
            /* Pick a major axis -- NOTE positive direction, _data[3], is inverse so flip the comparison */
            if (fabs(_x_data[3]) < fabs(_y_data[3]))
            {
                if (fabs(_x_data[3]) < fabs(_z_data[3]))
                {
                    this->n = 0 + ((int)(_x_data[3] < 0.0f) << 2);
                }
                else
                {
                    this->n = 2 + ((int)(_z_data[3] < 0.0f) << 2);
                }
            }
            else
            {
                if (fabs(_y_data[3]) < fabs(_z_data[3]))
                {
                    this->n = 1 + ((int)(_y_data[3] < 0.0f) << 2);
                }
                else
                {
                    this->n = 2 + ((int)(_z_data[3] < 0.0f) << 2);
                }
            }

            return;
        }
        
        frustrum(const packet_ray *r, const point_t &o, const unsigned s)
        {
            /* Max and min directions */
            vfp_t min_x_dir = r[0].get_x_grad();
            vfp_t min_y_dir = r[0].get_y_grad();
            vfp_t min_z_dir = r[0].get_z_grad();

            vfp_t max_x_dir = r[0].get_x_grad();
            vfp_t max_y_dir = r[0].get_y_grad();
            vfp_t max_z_dir = r[0].get_z_grad();
            for (unsigned int i = 1; i < s; i++)
            {
                min_x_dir = min(min_x_dir, r[i].get_x_grad());
                min_y_dir = min(min_y_dir, r[i].get_y_grad());
                min_z_dir = min(min_z_dir, r[i].get_z_grad());

                max_x_dir = max(max_x_dir, r[i].get_x_grad());
                max_y_dir = max(max_y_dir, r[i].get_y_grad());
                max_z_dir = max(max_z_dir, r[i].get_z_grad());
            }
            
            /* Max and min accross vector and invert the direction */
            _x_data[0] = o.x;
            _y_data[0] = o.y;
            _z_data[0] = o.z;
            _x_data[3] = 1.0f / -horizontal_min(min_x_dir);
            _y_data[3] = 1.0f / -horizontal_min(min_y_dir);
            _z_data[3] = 1.0f / -horizontal_min(min_z_dir);
            
            _x_data[1] = o.x;
            _y_data[1] = o.y;
            _z_data[1] = o.z;
            _x_data[2] = 1.0f / -horizontal_max(max_x_dir);
            _y_data[2] = 1.0f / -horizontal_max(max_y_dir);
            _z_data[2] = 1.0f / -horizontal_max(max_z_dir);
            
            this->mm_ogn[0] = vfp_t(o.x);
            this->mm_ogn[1] = vfp_t(o.y);
            this->mm_ogn[2] = vfp_t(o.z);
            
            this->mm_dir[0] = vfp_t(_x_data[2], _x_data[2], _x_data[3], _x_data[3]);
            this->mm_dir[1] = vfp_t(_y_data[2], _y_data[2], _y_data[3], _y_data[3]);
            this->mm_dir[2] = vfp_t(_z_data[2], _z_data[2], _z_data[3], _z_data[3]);
            
            /* Pick a major axis -- NOTE positive direction _data[3] is inverse so flip the comparison */
            if (fabs(_x_data[3]) < fabs(_y_data[3]))
            {
                if (fabs(_x_data[3]) < fabs(_z_data[3]))
                {
                    this->n = 0 + ((int)(_x_data[3] < 0.0f) << 2);
                }
                else
                {
                    this->n = 2 + ((int)(_z_data[3] < 0.0f) << 2);
                }
            }
            else
            {
                if (fabs(_y_data[3]) < fabs(_z_data[3]))
                {
                    this->n = 1 + ((int)(_y_data[3] < 0.0f) << 2);
                }
                else
                {
                    this->n = 2 + ((int)(_z_data[3] < 0.0f) << 2);
                }
            }

            return;
        }


        /* Adapt the frustrum to a leaf node with upper vertex u and lower vertex l */
        void adapt_to_leaf(const packet_ray *r, const point_t &u, const point_t &l, const unsigned *c, const unsigned s)
        {
            /* I0 - prevalent beam axis */
            const int I0 = this->n & 0x3;
            const int I1 = mod_3_lut[I0 + 1];
            const int I2 = mod_3_lut[I0 + 2];

            /* Width of the leaf node */
            float bminf;
            float bmaxf;
            switch (I0)
            {
                case 0 :
                    bminf = l.x;
                    bmaxf = u.x;
                    break;
                case 1 :
                    bminf = l.y;
                    bmaxf = u.y;
                    break;
                case 2 :
                    bminf = l.z;
                    bmaxf = u.z;
                    break;
                default :
                    assert(false);
                    break;
            }

            /* Avoid flat leaves */
            if (bminf == bmaxf) 
            { 
                bminf -= 0.5f;
                bmaxf += 0.5f;
            } 

            const vfp_t bmin = vfp_t(bminf);
            const vfp_t bmax = vfp_t(bmaxf); 

//            int negmaxabsdir    = this->n & (1<<2);     // 1 if dir is negative along maxabsdir

            /* Data for packet 0 */
            const vfp_t di      = inverse(r[c[0]].get_dir(I0));
            const vfp_t org0[]  = { r[c[0]].get_x0(),    r[c[0]].get_y0(),    r[c[0]].get_z0() };
            const vfp_t dir0[]  = { r[c[0]].get_dir(I1), r[c[0]].get_dir(I2) }; 
            const vfp_t tentr   = (bmin - org0[I0]) * di;
            const vfp_t texit   = (bmax - org0[I0]) * di; 

            /* Entry and exit co-ordinates */
            vfp_t r0min[2], r0max[2], r1min[2], r1max[2];
            r0min[0] = org0[I1] + tentr * dir0[0]; 
            r0min[1] = org0[I2] + tentr * dir0[1]; 
            r1min[0] = org0[I1] + texit * dir0[0]; 
            r1min[1] = org0[I2] + texit * dir0[1];

            r0max[0] = r0min[0]; 
            r0max[1] = r0min[1]; 
            r1max[0] = r1min[0]; 
            r1max[1] = r1min[1];

//            vfp_t orgmin(org0[I0]);
//            vfp_t orgmax(org0[I0]); 

            /* Loop for all packets */
            for (unsigned int ri = 1; ri < s; ri++) 
            {
                const vfp_t di      = inverse(r[c[ri]].get_dir(I0)); 
                const vfp_t org0[]  = { r[c[ri]].get_x0(),    r[c[ri]].get_y0(),    r[c[ri]].get_z0() };
                const vfp_t dir0[]  = { r[c[ri]].get_dir(I1), r[c[ri]].get_dir(I2) }; 
                const vfp_t tentr   = (bmin - org0[I0]) * di;
                const vfp_t texit   = (bmax - org0[I0]) * di; 

//                orgmin = min(orgmin, org0[I0]); 
//                orgmax = max(orgmax, org0[I0]);
                
                const vfp_t i1_entry = org0[I1] + tentr * dir0[0];
                const vfp_t i1_exit  = org0[I1] + texit * dir0[0];
                const vfp_t i2_entry = org0[I2] + tentr * dir0[1];
                const vfp_t i2_exit  = org0[I2] + texit * dir0[1];

                r0min[0] = min(r0min[0], i1_entry); 
                r0max[0] = max(r0max[0], i1_entry); 
                r0min[1] = min(r0min[1], i2_entry); 
                r0max[1] = max(r0max[1], i2_entry); 
                r1min[0] = min(r1min[0], i1_exit); 
                r1max[0] = max(r1max[0], i1_exit); 
                r1min[1] = min(r1min[1], i2_exit); 
                r1max[1] = max(r1max[1], i2_exit); 
            } 

            transpose(r0min[0], r0min[1], r1min[0], r1min[1]); 
            const vfp_t emin = min(min(r0min[0], r0min[1]), min(r1min[0], r1min[1])); 

            transpose(r0max[0], r0max[1], r1max[0], r1max[1]); 
            const vfp_t emax = max(max(r0max[0], r0max[1]), max(r1max[0], r1max[1])); 

            const vfp_t ext1 = shuffle<2, 0, 2, 0>(emin, emax); // I1 values 
            
            /* Triangle culling pre-computes */
            const vfp_t mix1    = shuffle<3, 2, 1, 0>(emin, emax); 
            const vfp_t mix2    = shuffle<1, 0, 3, 2>(emin, emax); 
            const vfp_t xminmax = shuffle<0, 0, 0, 0>(bmin, bmax); 
            const vfp_t xmaxmin = shuffle<0, 0, 0, 0>(bmax, bmin); 

            const vfp_t term0 = inverse(bmax - bmin); 
            this->q2 = mix1 - mix2;
            this->q1 = mix1 * (xminmax - xmaxmin) - xminmax * this->q2; 

            this->q2 *= term0; 
            this->q1 *= term0; 

            /* Rectangles on the front and rear face forming the frustrum */
            vfp_t entr[3];
            entr[I0] = bmin; 
            entr[I1] = shuffle<0, 2, 2, 0>(ext1, ext1);
            entr[I2] = shuffle<1, 1, 1, 1>(emin, emax);

            vfp_t extr[3];
            extr[I0] = bmax; 
            extr[I1] = shuffle<1, 3, 3, 1>(ext1, ext1);
            extr[I2] = shuffle<3, 3, 3, 3>(emin, emax);

            /* Frustrum origin and direction */
            this->ogn[0] = entr[0];
            this->ogn[1] = entr[1];
            this->ogn[2] = entr[2];
            this->dir[0] = extr[0] - entr[0];
            this->dir[1] = extr[1] - entr[1];
            this->dir[2] = extr[2] - entr[2];
            
//            vfp_t in_dir = inverse(this->dir[I0]); 
//            orgmin = vfp_t(min(min(orgmin[0], orgmin[1]), min(orgmin[2], orgmin[3]))); 
//            orgmax = vfp_t(max(max(orgmax[0], orgmax[1]), max(orgmax[2], orgmax[3]))); 
//    
//            this->tfar  = vfp_t(MAX_DIST); 
//            if (negmaxabsdir) 
//            {
//                this->torg0 = (orgmax - bmin) * in_dir; 
//                this->torg1 = (orgmin - bmin) * in_dir; 
//            }
//            else
//            { 
//                this->torg0 = (orgmin - bmin) * in_dir; 
//                this->torg1 = (orgmax - bmin) * in_dir; 
//            } 
        }

        /* Frustrum-triangle culling */
        bool cull(const point_t &a, const point_t &b, const point_t &c) const
        {
            /* I0 - prevalent beam axis */
            const int I0 = this->n & 0x3;
            const int I1 = mod_3_lut[I0 + 1];
            const int I2 = mod_3_lut[I0 + 2];

            /* Dot products with node pre-computes */
            const vfp_t t1 = this->q1; 
            const vfp_t t2 = this->q2; 
            const vfp_t d0 = t1 + (vfp_t(a[I0]) * t2) + vfp_t(a[I1], a[I2], -a[I1], -a[I2]);
            const vfp_t d1 = t1 + (vfp_t(b[I0]) * t2) + vfp_t(b[I1], b[I2], -b[I1], -b[I2]);
            const vfp_t d2 = t1 + (vfp_t(c[I0]) * t2) + vfp_t(c[I1], c[I2], -c[I1], -c[I2]); 

            /* Exclude the triangle if all vertices are on one side of the beam */
            return (move_mask(d0 & d1 & d2) != 0);
        }
       
        /* Access functions */
        float get_min_x0()                  const { return _x_data[0];               }
        float get_min_y0()                  const { return _y_data[0];               }
        float get_min_z0()                  const { return _z_data[0];               }
        float get_max_x0()                  const { return _x_data[1];               }
        float get_max_y0()                  const { return _y_data[1];               }
        float get_max_z0()                  const { return _z_data[1];               }
       
        float get_min_x_grad()              const { return 1.0f / _x_data[2];        }
        float get_min_y_grad()              const { return 1.0f / _y_data[2];        }
        float get_min_z_grad()              const { return 1.0f / _z_data[2];        }
        float get_max_x_grad()              const { return 1.0f / _x_data[3];        }
        float get_max_y_grad()              const { return 1.0f / _y_data[3];        }
        float get_max_z_grad()              const { return 1.0f / _z_data[3];        }
        
        float get_min_x_igrad()             const { return _x_data[2];               }
        float get_min_y_igrad()             const { return _y_data[2];               }
        float get_min_z_igrad()             const { return _z_data[2];               }
        float get_max_x_igrad()             const { return _x_data[3];               }
        float get_max_y_igrad()             const { return _y_data[3];               }
        float get_max_z_igrad()             const { return _z_data[3];               }
        
        vfp_t get_ogn(unsigned int i)       const { return this->ogn[i];                }
        vfp_t get_dir(unsigned int i)       const { return this->dir[i];                }
      
        vfp_t get_mm_ogn(unsigned int i)    const { return this->mm_ogn[i];             }
        vfp_t get_mm_idir(unsigned int i)   const { return this->mm_dir[i];             }
        vfp_t get_mm_dir(unsigned int i)    const { return inverse(this->mm_dir[i]);    }

//        vfp_t get_tfar()                    const { return this->tfar;              }
//        vfp_t get_torg0()                   const { return this->torg0;             }
//        vfp_t get_torg1()                   const { return this->torg1;             }
//      
//        void set_tfar(const vfp_t t)              { this->tfar  = t;                }
//        void set_torg0(const vfp_t t)             { this->torg0 = t;                }
//        void set_torg1(const vfp_t t)             { this->torg1 = t;                }
//        
//        bool neg_dir()                      const { return ((this->n & 0x4) != 0); }
      
    private : 
        vfp_t       ogn[3];         /* Entry point of the frustrum corner rays              */
        vfp_t       dir[3];         /* Direction of the frustrum corner rays                */
        vfp_t       q1;             /* Frustrum constant per leaf node for triangle culling */
        vfp_t       q2;             /* Frustrum constant per leaf node for triangle culling */
//        vfp_t       tfar;
//        vfp_t       torg0;
//        vfp_t       torg1;
        vfp_t       mm_ogn[3];      /* Max and min origin packed together                   */
        vfp_t       mm_dir[3];      /* Max and min direction packed together                */
        float       _x_data[4];
        float       _y_data[4];
        float       _z_data[4];
        // point_t     n_ogn;          /* Most negative origin                                 */
        // point_t     p_ogn;          /* Most positive origin                                 */
        // point_t     n_dir;          /* Most negative direction                              */
        // point_t     p_dir;          /* Most positive direction                              */
        int         n;              /* Major direction of the frustrum                      */
};

}; /* namespace raptor_raytracer */
#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifndef __FRUSTRUM_H__ */
