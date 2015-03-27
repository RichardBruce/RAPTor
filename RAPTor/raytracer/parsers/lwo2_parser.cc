/* Standard headers */
#include <cstdint>

/* Common headers */
#include "common.h"
#include "logging.h"

/* Ray tracer headers */
#include "parser_common.h"
#include "lwo_parser.h"
#include "lwo_chunks.h"
#include "lwo_clip.h"
#include "lwo_surf.h"
#include "vertex_map.h"
#include "normal_calculator.h"
#include "camera.h"


namespace raptor_raytracer
{
void parse_surf(const lwo_chunks &chunks, std::list<material *> &m, const std::string &p, const std::map<std::string, std::uint16_t> &tag_map, std::vector<std::unique_ptr<lwo_surf>> *const surfs)
{
    /* Parse CLIP */
    std::map<std::uint32_t, lwo_clip *> clips;
    for (int i = 0; i < chunks.size_of_clips(); ++i)
    {
        lwo_clip *const clip = new lwo_clip(p, chunks.clip(i), chunks.clip_len(i));
        const bool clip_parsed = clip->parse();
        assert(clip_parsed);

        clips.emplace(clip->clip_index(), clip);
    }


    /* Parse SURF */
    for (int i = 0; i < chunks.size_of_surfs(); ++i)
    {
        const char *surf = chunks.surf(i);
        const std::uint32_t surf_len = chunks.surf_len(i);
        BOOST_LOG_TRIVIAL(trace) << "Parsing material: " << surf << ". Length: " << surf_len;

        auto tag_iter = tag_map.find(surf);
        assert((tag_iter != tag_map.end()) || !"Current surface not found in tags");
        
        std::uint32_t srf_len = strlen(surf) + 1;
        srf_len     += srf_len & 0x1;
        surf        += srf_len;

        /* Parent surface */
        assert((*surf) == 0x00);   /* The source surface must be null */
        std::uint32_t source_len = strlen(surf) + 1;
        source_len  += source_len & 0x1;
        surf        += source_len;
        BOOST_LOG_TRIVIAL(trace) << "Name length: " << srf_len << " Source length: " << source_len;
        
        (*surfs)[tag_iter->second].reset(new lwo_surf());
        const bool surf_parsed = (*surfs)[tag_iter->second]->parse(clips, surf, surf_len - srf_len - source_len, tag_iter->second);
        assert(surf_parsed);
    }

    /* Copy to materials */
    for (std::uint32_t i = 0; i < tag_map.size(); ++i)
    {
        if (((*surfs)[i] != nullptr) && ((*surfs)[i]->material() != nullptr))
        {
            m.push_back((*surfs)[i]->material());
        }
    }

    /* Clean up */
    for (auto &c : clips)
    {
        delete c.second;
    }
}

void parse_pols(const lwo_chunks &chunks, light_list &l, primitive_list &e, const std::map<std::string, std::uint16_t> &tag_map, const std::vector<std::unique_ptr<lwo_surf>> &surfs, const std::vector<point_t> &all_points, const char *const buffer)
{
    /* Check this is the POLS chunk */
    const char *at = chunks.pols();
    const std::uint32_t pols_bytes = chunks.pols_len();
    check_for_chunk(&at, "FACE", 4, __LINE__);

    /* Find PTAGS */
    const char *ptags_at = chunks.ptag();
    check_for_chunk(&ptags_at, "SURF", 4, __LINE__);

    /* Find VMAP */
    vertex_map vmap;
    BOOST_LOG_TRIVIAL(trace) << "Parsing VMAP";
    for (int i = 0; i < chunks.size_of_vmaps(); ++i)
    {
        vmap.add(chunks.vmap(i), chunks.vmap_len(i));
    }

    /* Find VMAD */
    vertex_map vmad;
    BOOST_LOG_TRIVIAL(trace) << "Parsing VMAD";
    for (int i = 0; i < chunks.size_of_vmads(); ++i)
    {
        vmad.add_discontinuous(chunks.vmad(i), chunks.vmad_len(i));
    }

    /* Gather all the polygons */
    normal_calculator nc(all_points);
    std::uint32_t pol = 0;
    std::vector<int> pol_vert;
    std::uint32_t num_of_surfs = tag_map.size();
    while (at < (chunks.pols() + pols_bytes))
    {
        /* Parse the material to use */
        const std::uint32_t ptag_pol = parse_vx(&ptags_at);
        const std::uint16_t mat_num = from_byte_stream<std::uint16_t>(&ptags_at);
        assert((ptag_pol == pol) || !"Error: ptag is not for this polygon");

        /* Range check the material */
        if (static_cast<std::uint32_t>(mat_num) > num_of_surfs)
        {
            BOOST_LOG_TRIVIAL(error) << "Material " << std::hex << mat_num << " out of range at " << (std::uint32_t)(at - buffer) << std::dec;
            assert(false);
        }
        assert(surfs[mat_num] != nullptr);

        /* Parse vertices */
        const std::uint16_t vert_this_pol = from_byte_stream<std::uint16_t>(&at);
        for (std::uint32_t j = 0; j < vert_this_pol; j++)
        {
            const std::uint32_t vert_num = parse_vx(&at);
            pol_vert.push_back(vert_num);
            assert(vert_num < all_points.size());
        }

        /* Remember the polygon */
        nc.add_point_usage(pol_vert, surfs[mat_num]->smoothing_threshold(), mat_num, pol);
        
        /* Clean up */
        pol_vert.clear();
        ++pol;
    }

    /* Calculate normals for smoothed polygons */
    nc.calculate();

    /* Build polyongs */
    std::vector<point_t> pol_pnts;
    std::vector<point_t> pol_norm;
    std::vector<point_t> pol_vmap;
    for (int i = 0; i < nc.number_of_polygons(); ++i)
    {
        /* Check we have enough points */
        if (nc.number_of_points(i) < 3)
        {
            continue;
        }

        /* Look for VMAD or VMAP */
        const std::unique_ptr<lwo_surf> &surf = surfs[nc.group(i)];
        const std::string &vmap_name = surf->vmap_name();
        if (!vmap_name.empty())
        {
            for (int j = 0; j < nc.number_of_points(i); ++j)
            {
                const int pnt = nc.global_point(i, j);
                auto vmap_iter = vmap.find(vmap_name, pnt);
                auto vmad_iter = vmad.find(vmap_name, i, pnt);
                if (vmad_iter != nullptr)
                {
                    pol_vmap.push_back(point_t(vmad_iter[0], vmad_iter[1], 0.0f));
                }
                else if (vmap_iter != nullptr)
                {
                    pol_vmap.push_back(point_t(vmap_iter[0], vmap_iter[1], 0.0f));
                }
            }
        }

        /* Get normals */
        auto *const norms = nc.normals(&pol_norm, i);
        nc.points_on_polygon(&pol_pnts, i);

        /* Create the polygon */
        assert(pol_vmap.empty() || (pol_vmap.size() == pol_pnts.size()));
        assert((norms == nullptr) || (pol_norm.size() == pol_pnts.size()));
        if (pol_vmap.empty())
        {
            face_to_triangles(&e, &l, pol_pnts, surf->material(), false, norms);
        }
        else
        {
            face_to_triangles(&e, &l, pol_pnts, surf->material(), false, norms, &pol_vmap);
        }
        
        /* Clean up */
        pol_pnts.clear();
        pol_norm.clear();
        pol_vmap.clear();
    }
}

void lwo2_parser(
    const char *const       begin,
    const char              *at,
    std::string             &p,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c,
    const int               len)
{
    METHOD_LOG;

    /* Grab global data */
    lwo_chunks chunks(at, at + len);
    const bool populated = chunks.populate_globals();
    assert(populated || !"Error: Missing global chunks");

    /* Parse TAGS */
    std::uint16_t tag_id = 0;
    std::uint32_t tag_idx = 0;
    const char *tags = chunks.tags();
    std::map<std::string, std::uint16_t> tag_map;
    while (tag_idx < chunks.tags_len())
    {
        BOOST_LOG_TRIVIAL(trace) << "Adding tag: " << &tags[tag_idx] << " at tag id: " << tag_id;
        tag_map.emplace(std::string(&tags[tag_idx]), tag_id++);
        tag_idx += strlen(&tags[tag_idx]) + 1;
        tag_idx += tag_idx & 0x1;
    }

    /* Parse the SURF chunks */
    std::vector<std::unique_ptr<lwo_surf>> surfs(tag_map.size());
    parse_surf(chunks, m, p, tag_map, &surfs);

    /* Work through layrs */
    std::vector<point_t> all_points;
    while (!chunks.eof())
    {
        if (!chunks.populate_layer())
        {
            BOOST_LOG_TRIVIAL(warning) << "LAYR is missing chunks, skipping";
            continue;
        }

        /* Parse PNTS */
        const char *pnts            = chunks.pnts();
        std::uint32_t nr_of_verts   = chunks.pnts_len() / 12;
        for (std::uint32_t i = 0; i < nr_of_verts; i++)
        {
            point_t pnt;
            pnt.x = from_byte_stream<float>(&pnts);
            pnt.y = from_byte_stream<float>(&pnts);
            pnt.z = from_byte_stream<float>(&pnts);
            all_points.push_back(pnt);
        }

        /* Parse POLS */
        parse_pols(chunks, l, e, tag_map, surfs, all_points, begin);
        
        /* Clean up */
        all_points.clear();
    }
}
}; /* namespace raptor_raytracer */
