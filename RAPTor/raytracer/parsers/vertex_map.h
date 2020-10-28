#pragma once

/* Standard headers */
#include <map>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */

/* Raytracer headers */
#include "parser_common.h"


namespace raptor_raytracer
{
/* Class to track chunks in current layer */
class vertex_map : private boost::noncopyable
{
    public :
        /* Allow default CTOR and DTOR */

        bool add(const char *data, const int size)
        {
            /* No need to parse PICK, it doesnt affect rendering */
            const char *vmap_at = data;
            if (strncmp(vmap_at, "PICK", 4) == 0)
            {
                return false;
            }
            check_for_chunk(&vmap_at, "TXUV", 4, __LINE__);
        
            /* Get dimension and name */
            const std::uint16_t vmap_dim = raptor_parsers::from_byte_stream<std::uint16_t>(&vmap_at);
            const char *vmap_name = parse_string(&vmap_at);
            BOOST_LOG_TRIVIAL(trace) << "VMAP of dimensions: " << vmap_dim << " called: " << vmap_name;

            /* Get data */
            std::unique_ptr<map_data> vertex_map(new map_data());
            while (vmap_at < (data + size))
            {
                /* Add mapping from vertex to end of vmap points */
                const std::uint32_t vmap_pnt = parse_vx(&vmap_at);
                vertex_map->mappings.emplace(vmap_pnt, vertex_map->values.size());

                /* Add vmap points */
                for (std::uint16_t i = 0; i < vmap_dim; ++i)
                {
                    vertex_map->values.push_back(raptor_parsers::from_byte_stream<float>(&vmap_at));
                }
            }

            _vertex_maps.emplace(vmap_name, std::move(vertex_map));

            return true;
        }

        bool add_discontinuous(const char *data, const int size)
        {
            const char *vmad_at = data;
            check_for_chunk(&vmad_at, "TXUV", 4, __LINE__);

            /* Get dimension and name */
            const std::uint16_t vmad_dim = raptor_parsers::from_byte_stream<std::uint16_t>(&vmad_at);
            const char *vmad_name = parse_string(&vmad_at);
            BOOST_LOG_TRIVIAL(trace) << "VMAD of dimensions: " << vmad_dim << " called: " << vmad_name;

            /* Get data */
            std::unique_ptr<map_data> vertex_map(new map_data());
            while (vmad_at < (data + size))
            {
                /* Add mapping from vertex to end of vmad points */
                const std::uint32_t vmad_pnt = parse_vx(&vmad_at);
                const std::uint32_t vmad_pol = parse_vx(&vmad_at);
                vertex_map->mappings.emplace((static_cast<std::uint64_t>(vmad_pol) << 32) | vmad_pnt, vertex_map->values.size());

                /* Add vmad points */
                for (std::uint16_t i = 0; i < vmad_dim; ++i)
                {
                    vertex_map->values.push_back(raptor_parsers::from_byte_stream<float>(&vmad_at));
                }
            }

            _vertex_maps.emplace(vmad_name, std::move(vertex_map));

            return true;
        }

        float * find(const std::string &map_name, const int vertex)
        {
            /* Look up the map */
            auto data_iter = _vertex_maps.find(map_name);
            if (data_iter == _vertex_maps.end())
            {
                BOOST_LOG_TRIVIAL(trace) << "Cant find VMAP with name: " << map_name;
                return nullptr;
            }

            /* Look up the vertex */
            auto mappings = &data_iter->second->mappings;
            auto vertex_iter = mappings->find(vertex);
            if (vertex_iter == mappings->end())
            {
                return nullptr;
            }

            auto &values = data_iter->second->values;
            return &values[vertex_iter->second];
        }

        float * find(const std::string &map_name, const int poly, const int vertex)
        {
            /* Look up the map */
            auto data_iter = _vertex_maps.find(map_name);
            if (data_iter == _vertex_maps.end())
            {
                BOOST_LOG_TRIVIAL(trace) << "Cant find VMAD with name: " << map_name;
                return nullptr;
            }

            /* Look up the poly/vertex */
            const std::uint64_t id = (static_cast<std::uint64_t>(poly) << 32) | vertex;
            auto mappings = &data_iter->second->mappings;
            auto vertex_iter = mappings->find(id);
            if (vertex_iter == mappings->end())
            {
                return nullptr;
            }

            auto &values = data_iter->second->values;
            return &values[vertex_iter->second];
        }

    private :
        struct map_data
        {
            std::vector<float>              values;
            std::map<std::uint64_t, int>    mappings;
        };

        std::map<std::string, std::unique_ptr<map_data>>    _vertex_maps;
};
}; /* namespace raptor_raytracer */
