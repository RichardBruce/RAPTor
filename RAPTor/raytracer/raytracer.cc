#include "common.h"
#include "raytracer.h"

#include "scene.h"

#include "line.h"
#include "triangle.h"

#include "ray.h"
#include "packet_ray.h"
#include "frustrum.h"

#include "kdt_node.h"
#include "kd_tree_builder.h"

#include "bih_node.h"
#include "bih_builder.h"

namespace raptor_raytracer
{
/* Global performance counters for the spatial sub division */
#ifdef SPATIAL_SUBDIVISION_STATISTICS
/* Dynamic properties */
unsigned nr         = 0;    /* Number of rays shot */
unsigned nit        = 0;    /* Number of intersection tests */
unsigned ritm       = 0;    /* Ratio of intersection tests performed to minimum required tests */
unsigned nts        = 0;    /* Average number of nodes accessed per ray */
unsigned nets       = 0;    /* Average number of elementary nodes accessed per ray */
unsigned neets      = 0;    /* Average number of empty elementary nodes accessed per ray */

/* Static properties */
unsigned ng         = 0;    /* Number of generic nodes */
unsigned ne         = 0;    /* Number of elementary nodes */
unsigned nee        = 0;    /* Number of empty elementary nodes */
unsigned ner        = 0;    /* Maximum size of an elementary node */
unsigned ave_ob     = 0;    /* Averafe size of an elementary node */
unsigned max_depth  = 0;    /* Maximum depth of the tree */
#endif

#ifdef THREADED_RAY_TRACE
    /* Start the thread scheduler */
    task_scheduler_init init(task_scheduler_init::automatic);
#endif

/**********************************************************
 
**********************************************************/
void ray_trace_engine::ray_trace(ray &r, ext_colour_t *const c) const
{
    /* Does the ray intersect any objects */
    /* If the ray intersects multiple objects find the closest */
    const triangle *intersecting_object;
    hit_description hit_type;
    
    intersecting_object = ssd->find_nearest_object(&r, &hit_type);

    /* If there was an intersection set the rays endpoint and call that objects shader */
    if (hit_type.d < MAX_DIST)
    {
        r.calculate_destination(hit_type.d);
        
        this->shader_nr = 0;
        float           nr_reflections = 0.0f;
        float           nr_refractions = 0.0f;
        ray             reflection_rays[REFLECTION_ARRAY_SIZE];
        ray             refraction_rays[REFLECTION_ARRAY_SIZE];
        ext_colour_t    reflection_colour[REFLECTION_ARRAY_SIZE];
        ext_colour_t    refraction_colour[REFLECTION_ARRAY_SIZE];

        const line n = intersecting_object->generate_rays(*this, r, &hit_type, &reflection_rays[0], &refraction_rays[0], &nr_reflections, &nr_refractions);
        
        /* Trace shadow rays */
        for (unsigned int i = 0; i < this->lights.size(); ++i)
        {
            /* Clear the count of un-occluded rays */
            float made_it = 0.0f;

            const int ray_addr = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH));
            const int shader   = (i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH));
            for (int l = 0; l < this->nr_pending_shadows[shader]; ++l)
            {
                if (!this->ssd->found_nearer_object(&this->pending_shadows[ray_addr + l], this->pending_shadows[ray_addr + l].get_length()))
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
        intersecting_object->shade(*this, r, n, hit_type, c);
        
        /* Process secondary rays */
        for (int i = 0; i < nr_reflections; ++i)
        {
            this->ray_trace(reflection_rays[i], &reflection_colour[i]);
        }
        
        for (int i = 0; i < nr_refractions; ++i)
        {
            this->ray_trace(refraction_rays[i], &refraction_colour[i]);
        }

        intersecting_object->combind_secondary_rays(*this, (*c), &reflection_rays[0], &refraction_rays[0], &reflection_colour[0], &refraction_colour[0], &nr_reflections, &nr_refractions);
    }
    /* Otherwise colour with the background colour */
    else
    {
        *c = this->c.shade(r);
    }
}


#ifdef SIMD_PACKET_TRACING
/**********************************************************
 
**********************************************************/
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
                    if(!this->ssd->found_nearer_object(sr[addr], sr[addr]->get_length()))
                    {
                        m[r_to_s[addr]]++;
                    }  
                }
            }
            else
            {
                vfp_t closer(this->ssd->found_nearer_object(&r[j], t[j]));
                for (int k = 0; k < SIMD_WIDTH; ++k)
                {
                    int addr = (j << LOG2_SIMD_WIDTH) + k;
                    m[r_to_s[addr]] += (int)(closer[k] == 0.0);
                }
            }
        }
    }
    else
    {
        vfp_t closer[MAXIMUM_PACKET_SIZE];
        memset(closer, 0, MAXIMUM_PACKET_SIZE * sizeof(vfp_t));

        this->ssd->frustrum_found_nearer_object(&r[0], &t[0], &closer[0], s);
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            m[r_to_s[k]] += static_cast<int>(closer[k >> LOG2_SIMD_WIDTH][k & 0x3] == 0.0f);
        }
    }
    
    return;
}



/**********************************************************
 
**********************************************************/
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
                    intersecting_object[(i * SIMD_WIDTH) + j]  = this->ssd->find_nearest_object(&ray_0, &hit_0);
                    h[i].d[j] = hit_0.d;
                    h[i].u[j] = hit_0.u;
                    h[i].v[j] = hit_0.v;
                }
            }
            else
            {
                this->ssd->kdt_find_nearest_object(&r[i], &intersecting_object[i << LOG2_SIMD_WIDTH], &h[i]);
            }
        }
    }
    else
    {
        this->ssd->frustrum_find_nearest_object(r, &intersecting_object[0], &h[0], s);
    }

    /* Generate secondary rays, currently limited to shadow rays */
    line normals[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
    ray ray_p[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
    hit_description hit_p[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];

    float           nr_reflections[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    float           nr_refractions[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ray             reflection_rays[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ray             refraction_rays[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ext_colour_t    reflection_colour[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ext_colour_t    refraction_colour[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    
    memset(nr_reflections, 0, (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(float)));
    memset(nr_refractions, 0, (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(float)));
    
//    memset(this->nr_pending_shadows, 0, this->lights.size() * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(float)));
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
                int ray_addr = addr * REFLECTION_ARRAY_SIZE;
                normals[addr] = intersecting_object[addr]->generate_rays(*this, ray_p[addr], &hit_p[addr], &reflection_rays[ray_addr], &refraction_rays[ray_addr], &nr_reflections[addr], &nr_refractions[addr]);
            }
            else
            {
                for (unsigned int l = 0; l < this->lights.size(); ++l)
                {
                    const unsigned int nr_addr          = (l * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + addr;
                    this->nr_pending_shadows[nr_addr]   = 0.0f;
                }
                nr_reflections[addr] = 0.0f;
                nr_refractions[addr] = 0.0f;
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
            if(!this->ssd->found_nearer_object(rays_this_packet[addr], rays_this_packet[addr]->get_length()))
            {
                made_it[ray_to_shader[addr]]++;
            }
        }

        
        /* Collect the illumintation data */
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            const int addr   = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + (k * SHADOW_ARRAY_SIZE);
            const int shader = (i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + k;
            
            if (this->nr_pending_shadows[shader] > 0.0)
            {
                assert(made_it[k] >= 0);
                assert(made_it[k] <= 256);
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
            intersecting_object[i]->shade(*this, ray_p[i], normals[i], hit_p[i], &c[addr]);
        }
        /* Otherwise colour with the background colour */
        else
        {
            c[addr] = this->c.shade(ray_p[i]);
        }
    }
    

    /* Recurse for reflections */
    /* Pack the rays into a packet */
    int packed          = 0;
    int nr_of_packets   = 0;
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        int ray_addr = (i * REFLECTION_ARRAY_SIZE);
        for (int j = 0; j < (int)nr_reflections[i]; ++j)
        {
            rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &reflection_rays[ray_addr];
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
                    this->ray_trace(&next_packet[0], &reflection_colour[0], &ray_to_shader[0], nr_of_packets);
                    nr_of_packets = 0;
                }
            }
        }
    }

    if (nr_of_packets > 0)
    {
        this->ray_trace(&next_packet[0], &reflection_colour[0], &ray_to_shader[0], nr_of_packets);
    }
    
    /* Shoot rays mod SIMD_WIDTH alone */
    for (int i = 0; i < packed; ++i)
    {
        int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + i;
        this->ray_trace(*(rays_this_packet[addr]), &reflection_colour[ray_to_shader[addr]]);        
    }
    
    
    /* Recurse for refractions */
    /* Pack the rays into a packet */
    packed          = 0;
    nr_of_packets   = 0;
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        int ray_addr = (i * REFLECTION_ARRAY_SIZE);
        for (int j = 0; j < (int)nr_refractions[i]; ++j)
        {
            rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &refraction_rays[ray_addr];
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
                    this->ray_trace(&next_packet[0], &reflection_colour[0], &ray_to_shader[0], nr_of_packets);
                    nr_of_packets = 0;
                }
            }
        }
    }

    if (nr_of_packets > 0)
    {
        this->ray_trace(&next_packet[0], &refraction_colour[0], &ray_to_shader[0], nr_of_packets);
    }
    
    /* Shoot rays mod SIMD_WIDTH alone */
    for (int i = 0; i < packed; ++i)
    {
        int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + i;
        this->ray_trace(*(rays_this_packet[addr]), &refraction_colour[ray_to_shader[addr]]);        
    }

    /* Combine secondary rays in the shaders */
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        this->shader_nr = i;
        const unsigned int addr = ray_to_colour_lut[i];

        if (hit_p[i].d < MAX_DIST)
        {
            int ray_addr = i * REFLECTION_ARRAY_SIZE;
            intersecting_object[i]->combind_secondary_rays(*this, c[addr], &reflection_rays[ray_addr], &refraction_rays[ray_addr], &reflection_colour[ray_addr], &refraction_colour[ray_addr], &nr_reflections[i], &nr_refractions[i]);
        }
    }

    /* Done */
    return;
}


/**********************************************************
 
**********************************************************/
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


/**********************************************************
 
**********************************************************/
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
/**********************************************************
 
**********************************************************/
void ray_trace_engine::operator() (const blocked_range2d<unsigned>& r) const
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


/**********************************************************
 ray_tracer is the main ray tracing function. 
 
 The whole screen is ray traced based on the X/Y resolution, 
 minimum X/Y values and the X/Y increments. The scene is 
 passed in 'everything' and the light sources passrd in 
 'lights'. 'eye' specifies the launch point of the rays.
 'x_vec', 'y_vec' and 'z_vec'  specify the axis for ray 
 launch. The generated picture is put in camera.
**********************************************************/
template<class SpatialSubDivision>
void ray_tracer(const SpatialSubDivision *const ssd, const light_list &lights, const primitive_list &everything, camera &c)
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    std::cout << "Static properties of the tree :" << std::endl;
    std::cout << "Scene primitves                                             (nsp) : " << everything.size()                                     << std::endl;
    std::cout << "Tree primitves                                              (ntp) : " << ave_ob                                                << std::endl;
    std::cout << "Number of generic nodes                                     (ng ) : " << ng                                                    << std::endl;
    std::cout << "Number of elementary nodes                                  (ne ) : " << ne                                                    << std::endl;
    std::cout << "Number of empty elementary nodes                            (nee) : " << nee                                                   << std::endl;
    std::cout << "Maximum size of an elementary node                          (ner) : " << ner                                                   << std::endl;
    std::cout << "Average size of an elementary node                          (nea) : " << (static_cast<float>(ave_ob) / static_cast<float>(ne)) << std::endl;
    std::cout << "Maximum depth of the tree                                   (d  ) : " << max_depth                                             << std::endl << std::endl;
    
    ng          = 0;
    ne          = 0;
    nee         = 0;
    ner         = 0;
    ave_ob      = 0;
    max_depth   = 0;
#endif /* #ifdef SPATIAL_SUBDIVISION_STATISTICS */
    
    /* Make the screen 20 wide and 20 high ie/ -10 to 10 */
#ifdef THREADED_RAY_TRACE
#ifdef SIMD_PACKET_TRACING
    parallel_for(blocked_range2d<unsigned>(0, (unsigned)c.y_number_of_rays()/PACKET_WIDTH, 4, 0, (unsigned)c.x_number_of_rays()/PACKET_WIDTH, 4), ray_trace_engine(lights, c, ssd));

#else
    /* Thread using blocked_range2d to specify the size and triangle of block to trace */
    parallel_for(blocked_range2d<unsigned>(0, c.y_number_of_rays(), 32, 0, c.x_number_of_rays(), 32), ray_trace_engine(lights, c, ssd));

#endif /* #ifdef SIMD_PACKET_TRACING */
#else
    /* Instantiate the ray trace engine */
    ray_trace_engine engine(lights, c, ssd);
    
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


    /* Output dynamc statistics about the scene */
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    std::cout << "Dynamic properties of the KD-tree :"  << std::endl;
    std::cout << "Number of rays shot                                       (nr   ) : " << nr                                                           << std::endl;
    std::cout << "Intersection tests performed                              (nit  ) : " << nit                                                          << std::endl;
    std::cout << "Intersection tests performed : to minimum required tests  (ritm ) : " << (static_cast<float>(nit)      / static_cast<float>(ritm))    << std::endl;
    std::cout << "Average number of nodes accessed per ray                  (nts  ) : " << (static_cast<float>(nts)      / static_cast<float>(nr))      << std::endl;
    std::cout << "Average number of elementary nodes accessed per ray       (nets ) : " << (static_cast<float>(nets)     / static_cast<float>(nr))      << std::endl;
    std::cout << "Average number of empty elementary nodes accessed per ray (neets) : " << (static_cast<float>(neets)    / static_cast<float>(nr))      << std::endl;
#endif
}
}; /* namespace raptor_raytracer */
