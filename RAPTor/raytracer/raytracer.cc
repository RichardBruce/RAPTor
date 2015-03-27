/* Standard headers */

/* Boost headers */

/* Common headers */

/* Ray tracer headers */
#include "raytracer.h"
#include "secondary_ray_data.h"
#include "ssd.h"


namespace raptor_raytracer
{
#ifdef THREADED_RAY_TRACE
    /* Start the thread scheduler */
    tbb::task_scheduler_init init(tbb::task_scheduler_init::automatic);
#endif

void ray_trace_engine::ray_trace(ray &r, ext_colour_t *const c) const
{
    /* Does the ray intersect any objects */
    /* If the ray intersects multiple objects find the closest */
    const triangle *intersecting_object;
    hit_description hit_type;
    
    intersecting_object = _ssd->find_nearest_object(&r, &hit_type);

    /* If there was an intersection set the rays endpoint and call that objects shader */
    if (hit_type.d < MAX_DIST)
    {
        r.calculate_destination(hit_type.d);
        
        point_t vt;
        this->shader_nr = 0;
        ext_colour_t refl_colour[MAX_SECONDARY_RAYS];
        ext_colour_t refr_colour[MAX_SECONDARY_RAYS];
        secondary_ray_data refl;
        secondary_ray_data refr;
        refl.colours(&refl_colour[0]);
        refr.colours(&refr_colour[0]);
        const point_t vn(intersecting_object->generate_rays(*this, r, &vt, &hit_type, &refl, &refr));
        
        /* Trace shadow rays */
        for (unsigned int i = 0; i < this->lights.size(); ++i)
        {
            /* Clear the count of un-occluded rays */
            float made_it = 0.0f;

            const int ray_addr = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH));
            const int shader   = (i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH));
            for (int l = 0; l < this->nr_pending_shadows[shader]; ++l)
            {
                if (!_ssd->found_nearer_object(&this->pending_shadows[ray_addr + l], this->pending_shadows[ray_addr + l].get_length()))
                {
                    ++made_it;
                }
            }

            /* Collect the illumintation data */
            if (this->nr_pending_shadows[shader] > 0.0f)
            {
                this->pending_shadows[ray_addr].set_magnitude(made_it / this->nr_pending_shadows[shader]);
            }
        }

        /* Call the shader */
        intersecting_object->shade(*this, r, vn, vt, hit_type, c);
        
        /* Process secondary rays */
        for (int i = 0; i < static_cast<int>(refl.number()); ++i)
        {
            this->ray_trace(refl.rays()[i], &refl.colours()[i]);
        }
        
        for (int i = 0; i < static_cast<int>(refr.number()); ++i)
        {
            this->ray_trace(refr.rays()[i], &refr.colours()[i]);
        }

        intersecting_object->combind_secondary_rays(*this, c, refl, refr);
    }
    /* Otherwise colour with the background colour */
    else
    {
        *c = this->c.shade(&r);
    }
}


#ifdef SIMD_PACKET_TRACING
void ray_trace_engine::shoot_shadow_packet(packet_ray *const r, ray *const *const sr, vfp_t *const t, unsigned int *r_to_s, int *m, const int s, const int l) const
{
    /* Coherency check */
    bool    coherant = (s > 4);
    int     size[MAXIMUM_PACKET_SIZE];

    const int pkt_x_dir = move_mask(r[0].get_x_grad());
    const int pkt_y_dir = move_mask(r[0].get_y_grad());
    const int pkt_z_dir = move_mask(r[0].get_z_grad());
    for (int j = 0; j < s; ++j)
    {
        const int x_dir = move_mask(r[j].get_x_grad());
        const int y_dir = move_mask(r[j].get_y_grad());
        const int z_dir = move_mask(r[j].get_z_grad());

        if (((x_dir != 0) && (x_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((y_dir != 0) && (y_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((z_dir != 0) && (z_dir != ((1 << SIMD_WIDTH) - 1))))
        {
            coherant = false;
            size[j] = 0;
        }
        else if ((x_dir != pkt_x_dir) || (y_dir != pkt_y_dir) || (z_dir != pkt_z_dir))
        {
            coherant = false;
            size[j] = 1;
        }
        else
        {
            size[j] = 1;
        }
    }

    /* Shoot the most co-herant packet */
    if (!coherant)
    {
        /* Trace single rays */
        for (int j = 0; j < s; ++j)
        {
            if (size[j] == 0)
            {
                for (int k = 0; k < SIMD_WIDTH; ++k)
                {
                    int addr = (j << LOG2_SIMD_WIDTH) + k;
                    if(!_ssd->found_nearer_object(sr[addr], sr[addr]->get_length()))
                    {
                        m[r_to_s[addr]]++;
                    }  
                }
            }
            else
            {
                vfp_t closer(_ssd->found_nearer_object(&r[j], t[j]));
                for (int k = 0; k < SIMD_WIDTH; ++k)
                {
                    int addr = (j << LOG2_SIMD_WIDTH) + k;
                    m[r_to_s[addr]] += (int)(closer[k] == 0.0f);
                }
            }
        }
    }
    else
    {
        vfp_t closer[MAXIMUM_PACKET_SIZE];
        memset(closer, 0, MAXIMUM_PACKET_SIZE * sizeof(vfp_t));

        _ssd->frustrum_found_nearer_object(&r[0], &t[0], &closer[0], s);
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            m[r_to_s[k]] += static_cast<int>(closer[k >> LOG2_SIMD_WIDTH][k & 0x3] == 0.0f);
        }
    }
    
    return;
}


void ray_trace_engine::ray_trace(packet_ray *const r, ext_colour_t *const c, const unsigned int *const ray_to_colour_lut, const int s) const
{
    const triangle *intersecting_object[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    packet_hit_description h[MAXIMUM_PACKET_SIZE];

    /* Check co-herency */
    bool coherant = (s > 4);
    int size[MAXIMUM_PACKET_SIZE];
    const int pkt_x_dir = move_mask(r[0].get_x_grad());
    const int pkt_y_dir = move_mask(r[0].get_y_grad());
    const int pkt_z_dir = move_mask(r[0].get_z_grad());
    
    for (int i = 0; i < s; ++i)
    {
        const int x_dir = move_mask(r[i].get_x_grad());
        const int y_dir = move_mask(r[i].get_y_grad());
        const int z_dir = move_mask(r[i].get_z_grad());
    
        if (((x_dir != 0) && (x_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((y_dir != 0) && (y_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((z_dir != 0) && (z_dir != ((1 << SIMD_WIDTH) - 1))))
        {
            coherant = false;
            size[i] = 0;
        }
        else if ((x_dir != pkt_x_dir) || (y_dir != pkt_y_dir) || (z_dir != pkt_z_dir))
        {
            coherant = false;
            size[i] = 1;
        }
        else
        {
            size[i] = 1;
        }
    }
    
    /* Shoot the most co-herant packet */
    if (!coherant)
    {
        /* Trace single rays */
        for (int i = 0; i < s; ++i)
        {
            if (size[i] == 0)
            {
                for (unsigned int j = 0; j < SIMD_WIDTH; ++j)
                {
                    ray             ray_0 = r[i].extract(j);
                    hit_description hit_0 = h[i][j];
                    intersecting_object[(i * SIMD_WIDTH) + j]  = _ssd->find_nearest_object(&ray_0, &hit_0);
                    h[i].d[j] = hit_0.d;
                    h[i].u[j] = hit_0.u;
                    h[i].v[j] = hit_0.v;
                }
            }
            else
            {
                _ssd->find_nearest_object(&r[i], &intersecting_object[i << LOG2_SIMD_WIDTH], &h[i]);
            }
        }
    }
    else
    {
        _ssd->frustrum_find_nearest_object(r, &intersecting_object[0], &h[0], s);
    }

    /* Generate secondary rays */
    point_t vn[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
    point_t vt[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
    ray ray_p[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
    hit_description hit_p[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];

    ext_colour_t refl_colour[MAX_SECONDARY_RAYS * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ext_colour_t refr_colour[MAX_SECONDARY_RAYS * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    secondary_ray_data refl[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    secondary_ray_data refr[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    
    int addr = 0;
    for (int i = 0; i < s; ++i)
    {
        r[i].calculate_destination(h[i].d);
        r[i].extract(&ray_p[i << LOG2_SIMD_WIDTH]);
        h[i].extract(&hit_p[i << LOG2_SIMD_WIDTH]);

        for (int j = 0; j < SIMD_WIDTH; ++j)
        {
            if (hit_p[addr].d < MAX_DIST)
            {
                this->shader_nr = addr;
                int ray_addr = addr * MAX_SECONDARY_RAYS;
                vn[addr] = intersecting_object[addr]->generate_rays(*this, ray_p[addr], &vt[addr], &hit_p[addr], &refl[ray_addr], &refr[ray_addr]);
            }
            else
            {
                for (unsigned int l = 0; l < this->lights.size(); ++l)
                {
                    const unsigned int nr_addr          = (l * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + addr;
                    this->nr_pending_shadows[nr_addr]   = 0.0f;
                }
            }
            
            ++addr;
        }
    }

    /* Trace the shadow rays to each light */
    packet_ray      next_packet[MAXIMUM_PACKET_SIZE];
    int             made_it[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ray *           rays_this_packet[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    unsigned int    ray_to_shader[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    vfp_t           t[MAXIMUM_PACKET_SIZE];
    for (unsigned int i = 0; i < this->lights.size(); ++i)
    {
        memset(made_it, 0, (MAXIMUM_PACKET_SIZE * SIMD_WIDTH) * sizeof(int));

        /* Pack the rays into a packet */
        int packed          = 0;
        int nr_of_packets   = 0;
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            int ray_addr = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + (k * SHADOW_ARRAY_SIZE);
            
            for (int l = 0; l < this->nr_pending_shadows[(i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + k]; ++l)
            {
                rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &this->pending_shadows[ray_addr];
                ray_to_shader[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = k;
                ++packed;
                ++ray_addr;
                
                if (packed == SIMD_WIDTH)
                {
                    packed = 0;
                    t[nr_of_packets] = next_packet[nr_of_packets].pack(&rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH)]);
                    ++nr_of_packets;
                    
                    if (nr_of_packets == MAXIMUM_PACKET_SIZE)
                    {
                        this->shoot_shadow_packet(next_packet, rays_this_packet, t, ray_to_shader, made_it, nr_of_packets, i);
                        nr_of_packets = 0;
                    }
                }
            }
        }
        
        this->shoot_shadow_packet(next_packet, rays_this_packet, t, ray_to_shader, made_it, nr_of_packets, i);
    
        /* Shoot rays mod SIMD_WIDTH alone */
        for (int k = 0; k < packed; ++k)
        {
            int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + k;
            if(!_ssd->found_nearer_object(rays_this_packet[addr], rays_this_packet[addr]->get_length()))
            {
                made_it[ray_to_shader[addr]]++;
            }
        }

        
        /* Collect the illumintation data */
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            const int addr   = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + (k * SHADOW_ARRAY_SIZE);
            const int shader = (i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + k;
            
            if (this->nr_pending_shadows[shader] > 0.0f)
            {
                this->pending_shadows[addr].set_magnitude(made_it[k] / this->nr_pending_shadows[shader]);
            }
        }
    }
    

    /* If there was an intersection set the rays endpoint and call that objects shader */
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        const unsigned int addr = ray_to_colour_lut[i];
        this->shader_nr = i;

        if (hit_p[i].d < MAX_DIST)
        {
            intersecting_object[i]->shade(*this, ray_p[i], vn[i], vt[i], hit_p[i], &c[addr]);
        }
        /* Otherwise colour with the background colour */
        else
        {
            c[addr] = this->c.shade(&ray_p[i]);
        }
    }

    /* Recurse for reflections */
    /* Pack the rays into a packet */
    int packed          = 0;
    int nr_of_packets   = 0;
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        int ray_addr = (i * MAX_SECONDARY_RAYS);
        refl[i].colours(&refl_colour[ray_addr]);

        for (int j = 0; j < static_cast<int>(refl[i].number()); ++j)
        {
            rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &refl[i].rays()[j];
            ray_to_shader[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = ray_addr;
            ++packed;
            ++ray_addr;
            
            if (packed == SIMD_WIDTH)
            {
                packed = 0;
                next_packet[nr_of_packets].pack(&rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH)]);
                ++nr_of_packets;
                
                if (nr_of_packets == MAXIMUM_PACKET_SIZE)
                {
                    this->ray_trace(&next_packet[0], &refl_colour[0], &ray_to_shader[0], nr_of_packets);
                    nr_of_packets = 0;
                }
            }
        }
    }

    if (nr_of_packets > 0)
    {
        this->ray_trace(&next_packet[0], &refl_colour[0], &ray_to_shader[0], nr_of_packets);
    }
    
    /* Shoot rays mod SIMD_WIDTH alone */
    for (int i = 0; i < packed; ++i)
    {
        int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + i;
        this->ray_trace(*(rays_this_packet[addr]), &refl_colour[ray_to_shader[addr]]);        
    }
    
    
    /* Recurse for refractions */
    /* Pack the rays into a packet */
    packed          = 0;
    nr_of_packets   = 0;
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        int ray_addr = (i * MAX_SECONDARY_RAYS);
        refr[i].colours(&refr_colour[ray_addr]);

        for (int j = 0; j < static_cast<int>(refr[i].number()); ++j)
        {
            rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &refr[i].rays()[j];
            ray_to_shader[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = ray_addr;
            ++packed;
            ++ray_addr;
            
            if (packed == SIMD_WIDTH)
            {
                packed = 0;
                next_packet[nr_of_packets].pack(&rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH)]);
                ++nr_of_packets;
                
                if (nr_of_packets == MAXIMUM_PACKET_SIZE)
                {
                    this->ray_trace(&next_packet[0], &refr_colour[0], &ray_to_shader[0], nr_of_packets);
                    nr_of_packets = 0;
                }
            }
        }
    }

    if (nr_of_packets > 0)
    {
        this->ray_trace(&next_packet[0], &refr_colour[0], &ray_to_shader[0], nr_of_packets);
    }
    
    /* Shoot rays mod SIMD_WIDTH alone */
    for (int i = 0; i < packed; ++i)
    {
        int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + i;
        this->ray_trace(*(rays_this_packet[addr]), &refr_colour[ray_to_shader[addr]]);        
    }

    /* Combine secondary rays in the shaders */
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        this->shader_nr = i;
        const unsigned int addr = ray_to_colour_lut[i];

        if (hit_p[i].d < MAX_DIST)
        {
            intersecting_object[i]->combind_secondary_rays(*this, &c[addr], refl[i], refr[i]);
        }
    }

    /* Done */
    return;
}


void ray_trace_engine::ray_trace_one_packet(const int x, const int y) const
{
    /* Create a packet of rays through the screen */
    packet_ray r[MAXIMUM_PACKET_SIZE];
    this->c.pixel_to_co_ordinate(r, x, y);
    
    /* Ray trace the packet */
    ext_colour_t pixel_colour[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    this->ray_trace(r, &pixel_colour[0], packet_ray_to_pixel_lut, MAXIMUM_PACKET_SIZE);
    
    /* Saturate the colour and output */
    for (int i = 0; i < (std::sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH)); ++i)
    {
        for (int j = 0; j < (std::sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH)); ++j)
        {
            /* Save output */
            this->c.set_pixel(pixel_colour[j + (i * (unsigned)std::sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH))], (x + j), (y + i));
        }
    }
}
#endif /* #ifdef SIMD_PACKET_TRACING */


inline void ray_trace_engine::ray_trace_one_pixel(const int x, const int y) const
{
    /* Convert to co-ordinate system */
    point_t ray_dir = this->c.pixel_to_co_ordinate(x, y);
    normalise(&ray_dir);

    /* Create a ray with this information */
    ray ray_0(this->c.camera_position(), ray_dir.x, ray_dir.y, ray_dir.z);

    /* Work on the pixel as a float and then saturate back to an unsigned char */
    ext_colour_t pixel_colour;
    this->ray_trace(ray_0, &pixel_colour);

    /* Saturate colours and save output */
    this->c.set_pixel(pixel_colour, x, y);
}


#ifdef THREADED_RAY_TRACE
void ray_trace_engine::operator() (const tbb::blocked_range2d<unsigned>& r) const
{
#ifdef SIMD_PACKET_TRACING
    for (unsigned y = r.rows().begin(); y != r.rows().end(); ++y)
    {     
        for(unsigned x = r.cols().begin(); x != r.cols().end(); ++x)
        {
            this->ray_trace_one_packet((x * PACKET_WIDTH), (y * PACKET_WIDTH));
        }                     
    }

#else
    /* Trace the square block specified in r */
    for (unsigned y = r.rows().begin(); y != r.rows().end(); ++y)
    {     
        for(unsigned x = r.cols().begin(); x != r.cols().end(); ++x)
        {
            this->ray_trace_one_pixel(x, y);
        }                     
    }
#endif /* #ifdef SIMD_PACKET_TRACING */
}
#endif /* #ifdef THREADED_RAY_TRACE */

/* Ray tracer main function */
void ray_tracer(const ssd *const sub_division, const light_list &lights, const primitive_list &everything, camera &c)
{
    /* Make the screen 20 wide and 20 high ie/ -10 to 10 */
#ifdef THREADED_RAY_TRACE
#ifdef SIMD_PACKET_TRACING
    tbb::parallel_for(tbb:blocked_range2d<unsigned>(0, (unsigned)c.y_number_of_rays()/PACKET_WIDTH, 4, 0, (unsigned)c.x_number_of_rays()/PACKET_WIDTH, 4), ray_trace_engine(lights, c, sub_division));

#else
    /* Thread using blocked_range2d to specify the size and triangle of block to trace */
    tbb::parallel_for(tbb:blocked_range2d<unsigned>(0, c.y_number_of_rays(), 32, 0, c.x_number_of_rays(), 32), ray_trace_engine(lights, c, sub_division));

#endif /* #ifdef SIMD_PACKET_TRACING */
#else
    /* Instantiate the ray trace engine */
    ray_trace_engine engine(lights, c, sub_division);
    
    /* Trace a ray through each pixel of the screen working bottom left to top right */
#ifdef SIMD_PACKET_TRACING
    for(unsigned y = 0; y < c.y_number_of_rays(); y += PACKET_WIDTH)
    {
        for(unsigned x = 0; x < c.x_number_of_rays(); x += PACKET_WIDTH)
        {
            engine.ray_trace_one_packet(x, y);
        }
    }
#else
    for(unsigned y = 0; y < c.y_number_of_rays(); ++y)
    {
        for(unsigned x = 0; x < c.x_number_of_rays(); ++x)
        {
            engine.ray_trace_one_pixel(x, y);
        }
    }
#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifdef THREADED_RAY_TRACE */
}
}; /* namespace raptor_raytracer */
