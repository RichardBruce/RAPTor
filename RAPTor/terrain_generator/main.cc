/* Standard headers */
#include <vector>

/* Common headers */
#include "logging.h"
#include "perlin_noise_2d.h"
#include "simplex_noise_2d.h"

/* Raytracer headers */
#include "parser_common.h"
#include "coloured_mapper_shader.h"
#include "phong_shader.h"
#include "planar_mapper.h"

/* Physics headers */
#include "vertex_group.h"
#include "physics_object.h"
#include "physics_options.h"
#include "physics_engine.h"
#include "simulation_environment.h"

/* Terrain generator headers */
#include "height_map.h"
#include "grid_cell.h"
// #include "convex_decomposition.h"

using raptor_terrain::grid_cell;
using raptor_terrain::neighbour_t;


bool try_merge_grid_cell(grid_cell **const proc_grid_cells, grid_cell **const raw_grid_cells, grid_cell **const stack, const int add_idx)
{
    /* Check for the edge of the map */
    const float min_cos = 0.9999f;
    if (add_idx == -1)
    {
        return false;
    }

    /* Check if we can merge */
    const grid_cell *const cell = raw_grid_cells[add_idx];
    if ((cell != nullptr) && stack[0]->can_merge(*cell, min_cos))
    {
        std::swap(proc_grid_cells[add_idx], raw_grid_cells[add_idx]);
        stack[1] = proc_grid_cells[add_idx];
        return true;
    }

    return false;
}


neighbour_t track_anti_clockwise_edge(grid_cell **const grid_cells, const grid_cell **at, const neighbour_t facing, const bool update_at)
{
    /* Try to turn right */
    const neighbour_t right_dir = turn_right(facing);
    const int right_neighbour = (*at)->neighbour(right_dir);
    if ((right_neighbour != -1) && (grid_cells[right_neighbour] != nullptr))
    {
        return right_dir;
    }

    /* Try to move straight */
    const int straight_neighbour = (*at)->neighbour(facing);
    if (straight_neighbour != -1)
    {
        const grid_cell *front_cell = grid_cells[straight_neighbour];
        if (front_cell != nullptr)
        {
            if (update_at)
            {
                (*at) = front_cell;
            }
            return facing;
        }
    }

    /* Try to turn left */
    const neighbour_t left_dir = turn_left(facing);
    const int left_neighbour = (*at)->neighbour(left_dir);
    if ((left_neighbour != -1) && (grid_cells[left_neighbour] != nullptr))
    {
        return left_dir;
    }

    /* Try to turn around */
    const neighbour_t back_dir = turn_around(facing);
    const int back_neighbour = (*at)->neighbour(back_dir);
    if ((back_neighbour != -1) && (grid_cells[back_neighbour] != nullptr))
    {
        return back_dir;
    }

    /* Stuck, stay were we are, but change direction */
    return back_dir;
}


int main()
{
    const int x = 128;
    const int y = 128;

    const int cell_x = x - 1;
    const int cell_y = y - 1;
    const int nr_grid_cells = cell_x * cell_y;

    const float xs = 25.0f;
    const float ys = 25.0f;

    /* Enviroment set up */
    raptor_physics::physics_options po(0.04f, 0.04f, -1, false, true);
    raptor_physics::physics_engine pe(new raptor_physics::rigid_body_collider(0.5f, 0.75f, 0.75f), false);
    pe.default_collider(new raptor_physics::rigid_body_collider(0.5f, 0.3f, 0.3f));
    raptor_physics::simulation_environment se(&pe, &po);
    se.load_screen("./load_screen.png");

    std::unique_ptr<raptor_raytracer::material> m[16];
    m[ 0].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), 1.0f));
    m[ 1].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(128.0f,   0.0f,   0.0f), 1.0f));
    m[ 2].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f,   0.0f,   0.0f), 1.0f));
    m[ 3].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(  0.0f, 128.0f,   0.0f), 1.0f));
    m[ 4].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(  0.0f, 255.0f,   0.0f), 1.0f));
    m[ 5].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(  0.0f,   0.0f, 128.0f), 1.0f));
    m[ 6].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(  0.0f,   0.0f, 255.0f), 1.0f));
    m[ 7].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(128.0f, 128.0f,   0.0f), 1.0f));
    m[ 8].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 255.0f,   0.0f), 1.0f));
    m[ 9].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(128.0f,   0.0f, 128.0f), 1.0f));
    m[10].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f,   0.0f, 255.0f), 1.0f));
    m[11].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(  0.0f, 128.0f, 128.0f), 1.0f));
    m[12].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(  0.0f, 255.0f, 255.0f), 1.0f));
    m[13].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(128.0f, 255.0f, 255.0f), 1.0f));
    m[14].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 128.0f, 255.0f), 1.0f));
    m[15].reset(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 128.0f), 1.0f));

    float *img;
    unsigned int img_width;
    unsigned int img_height;
    const raptor_raytracer::ext_colour_t black(0.0f, 0.0f, 0.0f);
    unsigned int cpp = raptor_raytracer::read_jpeg(&img, "./grass-texture-2.jpg", &img_height, &img_width);
    raptor_raytracer::planar_mapper *tm = new raptor_raytracer::planar_mapper(boost::shared_array<float>(img), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 1.0f, 0.0f), point_t<>(10.0f, 0.0f, 10.0f), cpp, img_width, img_height, raptor_raytracer::texture_wrapping_mode_t::tile, raptor_raytracer::texture_wrapping_mode_t::tile);
    std::unique_ptr<raptor_raytracer::material> grass(new raptor_raytracer::coloured_mapper_shader(black, raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), black, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, nullptr, tm, nullptr, nullptr));

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), point_t<>( 200.0f,  1500.0f,  200.0f));
    // se.add_light(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), point_t<>( 200.0f, -1500.0f,  200.0f));


    /* Generate height map */
    // perlin_noise_2d noise(57);
    // raptor_terrain::height_map<perlin_noise_2d> hm(noise, 100.0f, 0.05f, 0.4f, 3, x, y);
    simplex_noise_2d noise;
    raptor_terrain::height_map<simplex_noise_2d> hm(noise, 100.0f, 0.01f, 0.4f, 3, x, y);
    std::unique_ptr<point_t<> []> verts(hm.generate(xs, ys));


    // std::vector< raptor_terrain::Vec3<raptor_terrain::Real> > points;
    // for (int i = 0; i < (x * y); ++i)
    // {
    //     points.emplace_back(verts[i].x, verts[i].y, verts[i].z);
    // }

    // std::vector< raptor_terrain::Vec3<long> > triangles;
    // for (int i = 0; i < (x - 1); ++i)
    // {
    //      Calculate row offsets for vertices 
    //     const int vert_row = i * y;
    //     const int vert_row_p1 = vert_row + y;

    //     for (int j = 0; j < (y - 1); ++j)
    //     {
    //         triangles.emplace_back(vert_row + j, vert_row + j + 1, vert_row_p1 + j + 1);
    //         triangles.emplace_back(vert_row_p1 + j, vert_row + j, vert_row_p1 + j + 1);
    //     }
    // }

    // raptor_terrain::HeapManager * heapManager = raptor_terrain::createHeapManager(65536*(1000));
    // raptor_terrain::HACD * const myHACD = raptor_terrain::CreateHACD(heapManager);
    // myHACD->SetPoints(&points[0]);
    // myHACD->SetNPoints(points.size());
    // myHACD->SetTriangles(&triangles[0]);
    // myHACD->SetNTriangles(triangles.size());
    // myHACD->SetCompacityWeight(0.0001);
    // myHACD->SetVolumeWeight(0.0);
    // myHACD->SetConnectDist(0.001);               // if two connected components are seperated by distance < ccConnectDist
    //                                                     // then create a virtual edge between them so the can be merged during 
    //                                                     // the simplification process
          
    // myHACD->SetNClusters(16);                     // minimum number of clusters
    // myHACD->SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
    // myHACD->SetConcavity(1.0);                     // maximum concavity
    // myHACD->SetSmallClusterThreshold(0.075);                 // threshold to detect small clusters
    // myHACD->SetNTargetTrianglesDecimatedMesh(2000); // # triangles in the decimated mesh
    // myHACD->SetCallBack(&CallBack);
    // myHACD->SetAddExtraDistPoints(false);   
    // myHACD->SetAddFacesPoints(true); 
    
    // myHACD->Compute();
    // const size_t nClusters = myHACD->GetNClusters();

    // BOOST_LOG_TRIVIAL(trace) << "Number of convex hulls: " << nClusters;
    // for(size_t c = 0; c < nClusters; ++c)
    // {
    //     std::cout << std::endl << "Convex-Hull " << c << std::endl;
    //     size_t nPoints = myHACD->GetNPointsCH(c);
    //     size_t nTriangles = myHACD->GetNTrianglesCH(c);
    //     std::unique_ptr<raptor_terrain::Vec3<raptor_terrain::Real> []> pointsCH(new raptor_terrain::Vec3<raptor_terrain::Real>[nPoints]);
    //     std::unique_ptr<raptor_terrain::Vec3<long> []> trianglesCH(new raptor_terrain::Vec3<long>[nTriangles]);
    //     myHACD->GetCH(c, pointsCH.get(), trianglesCH.get());
    //     BOOST_LOG_TRIVIAL(trace) << "Number of vertices: " << nPoints;
    //     BOOST_LOG_TRIVIAL(trace) << "Number of triangles: " << nTriangles;

    //     std::vector<point_t<>> patch_verts;
    //     for(size_t v = 0; v < nPoints; ++v)
    //     {
    //         patch_verts.emplace_back(pointsCH[v].X(), pointsCH[v].Y(), pointsCH[v].Z());
    //     }

    //     std::vector<int> *const tris = new std::vector<int>();
    //     for(size_t f = 0; f < nTriangles; ++f)
    //     {
    //         tris->push_back(trianglesCH[f].X());
    //         tris->push_back(trianglesCH[f].Y());
    //         tris->push_back(trianglesCH[f].Z());
    //     }


    //     /* Add static objects */
    //     raptor_physics::vertex_group *const vg = new raptor_physics::vertex_group(patch_verts, tris, m[c & 0xf].get());
    //     raptor_physics::physics_object *const phy_obj = new raptor_physics::physics_object(vg, point_t<>(0.0f, 0.0f, 0.0f), std::numeric_limits<float>::infinity());
    //     se.add_object(phy_obj);
    // }

    // DestroyHACD(myHACD);
    // releaseHeapManager(heapManager);
    
    // // return 1;
    
    // /* Run physics simulation */
    // se.run();

    // return 0;

    /* Build grid cells */
    std::unique_ptr<grid_cell *[]> raw_grid_cells(raptor_terrain::vertices_to_grid_cells(verts.get(), y, cell_x, cell_y));


    /* Merge grid plates that arent needed */
    std::vector<std::vector<int>> facets;
    std::unique_ptr<grid_cell *[]> proc_grid_cells(new grid_cell *[nr_grid_cells]);
    std::unique_ptr<grid_cell *[]> stack(new grid_cell *[nr_grid_cells]);
    memset(proc_grid_cells.get(), 0, nr_grid_cells * sizeof(grid_cell *));
    for (int i = 0; i < nr_grid_cells; ++i)
    {
        if (raw_grid_cells[i] != nullptr)
        {
            BOOST_LOG_TRIVIAL(trace) << "Checking for merges starting at: " << i;
            std::swap(proc_grid_cells[i], raw_grid_cells[i]);

            int current_cell = 0;
            stack[0] = proc_grid_cells[i];
            while (true)
            {
                /* Move north while ever possible */
                while (try_merge_grid_cell(proc_grid_cells.get(), raw_grid_cells.get(), &stack[current_cell], stack[current_cell]->north()))
                {
                    ++current_cell;
                }

                /* Try to move one west, then go back to trying to move north */
                if (try_merge_grid_cell(proc_grid_cells.get(), raw_grid_cells.get(), &stack[current_cell], stack[current_cell]->west()))
                {
                    ++current_cell;
                    continue;
                }

                /* Try to move one east, then go back to trying to move north */
                if (try_merge_grid_cell(proc_grid_cells.get(), raw_grid_cells.get(), &stack[current_cell], stack[current_cell]->east()))
                {
                    ++current_cell;
                    continue;
                }

                /* Try to move one south, then go back to trying to move north */
                if (try_merge_grid_cell(proc_grid_cells.get(), raw_grid_cells.get(), &stack[current_cell], stack[current_cell]->south()))
                {
                    ++current_cell;
                    continue;
                }

                /* Try to back track */
                if (current_cell > 0)
                {
                    --current_cell;
                }
                else
                {
                    /* No progress is possible so the group is complete */
                    BOOST_LOG_TRIVIAL(trace) << "Merging complete";
                    break;
                }
            }

            /* Start in the top left and work anti-clockwise */
            BOOST_LOG_TRIVIAL(trace) << "About to begin edge tracing from: " << i;
            const grid_cell *cur_cell = proc_grid_cells[i];
            const grid_cell *last_cell = proc_grid_cells[i];

            neighbour_t facing = track_anti_clockwise_edge(proc_grid_cells.get(), &cur_cell, neighbour_t::SOUTH, false);
            neighbour_t previous_facing = facing;
            const neighbour_t first_move = facing;

            facets.emplace_back(1, cur_cell->corner_leaving(turn_right(facing), facing));
            while (true)
            {
                /* If possible turn right, else go straight, else turn left, finally go back */
                /* Find the next cell tracking an anit clockwise edge */
                neighbour_t turning;
                float total_distance = (turn_left(previous_facing) == facing) ? 1.0f : 0.0f;
                do
                {
                    total_distance += 1.0f;
                    turning = track_anti_clockwise_edge(proc_grid_cells.get(), &cur_cell, facing, true);

                    assert(cur_cell != nullptr);
                } while (facing == turning);

                /* Subtract the edge from the turning cell if turning right */
                if (turn_right(facing) == turning)
                {
                    total_distance -= 1.0f;
                }

                /* Adjust geometry given that last_cell to cur_cell is "flat" */
                float distance = (turn_right(previous_facing) == facing) ? 0.0f : 1.0f;
                const int in_vertex = cur_cell->corner_entering(facing, turning);
                const float slope = (verts[in_vertex].y - verts[facets.back().back()].y) / total_distance;
                while (last_cell != cur_cell)
                {
                    const int adj_idx = last_cell->furthest_index(facing);
                    verts[adj_idx].y = verts[facets.back().back()].y + (slope * distance);
                    distance += 1.0f;

                    /* Move to next cell */
                    last_cell = proc_grid_cells[last_cell->neighbour(facing)];
                }

                const int out_vertex = cur_cell->corner_leaving(facing, turning);
                if (in_vertex != out_vertex)
                {
                    facets.back().push_back(in_vertex);
                }

                /* Check for termination */
                if ((cur_cell == proc_grid_cells[i]) && (turning == first_move))
                {
                    break;
                }

                /* Add vertices to current facet */
                facets.back().push_back(out_vertex);

                /* Update cell if not turning around, in which case we are on the same cell */
                if (turn_around(facing) != turning)
                {
                    cur_cell = proc_grid_cells[cur_cell->neighbour(turning)];
                }
                assert(cur_cell != nullptr);

                /* Move again */
                previous_facing = facing;
                facing = turning;
            }

            /* Check the number of vertices */
            assert(((facets.back().size() & 0x1) == 0) || !"Error: We are dealing with squares so the facets must have an even number of vertices");

            /* Clean up */
            for (int i = 0; i < nr_grid_cells; ++i)
            {
                if (proc_grid_cells[i] != nullptr)
                {
                    delete proc_grid_cells[i];
                    proc_grid_cells[i] = nullptr;
                }
            }
        }
    }

    /* Bash vertex group from patch */
    BOOST_LOG_TRIVIAL(trace) << "About to build physics objects";
    for (auto &facet : facets)
    {
        /* Collect vertices */
        BOOST_LOG_TRIVIAL(trace) << "Collecting vertices";
        std::vector<point_t<>> *const facet_verts = new std::vector<point_t<>>();
        for (int f : facet)
        {
            assert(f >= 0 || !"Vertex index is out of range");
            assert(f < (x * y) || !"Error: Vertex index is out of range");
            facet_verts->push_back(verts[f]);
        }

        /* Bash into triangles */
        std::vector<int> tris;
        raptor_raytracer::face_to_triangle_edges(&tris, *facet_verts);
        assert((tris.size() == ((facet_verts->size() - 2) * 3)) || !"Error: There is a triangle missing");

        /* Add static objects */
        const std::vector<raptor_physics::polygon> polys{ raptor_physics::polygon(facet_verts, tris) };
        raptor_physics::vertex_group *const vg = new raptor_physics::vertex_group(facet_verts, polys, m[0].get());
        raptor_physics::physics_object *const phy_obj = new raptor_physics::physics_object(vg, point_t<>(0.0f, 0.0f, 0.0f), std::numeric_limits<float>::infinity());
        se.add_object(phy_obj);
    }

    // return 1;

    /* Run physics simulation */
    se.run();

    return 0;
}