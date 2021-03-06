#include "common.h"
#include "parser_common.h"
#include "polygon_to_triangles.h"

#include "camera.h"

#include "off_parser.h"

/* Shaders */
#include "phong_shader.h"


namespace raptor_raytracer
{
void off_parser(
    std::ifstream           &off_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  *c)
{
    /* Find the size of the file */
    off_file.seekg(0, std::ios::end);
    size_t len = off_file.tellg();
    off_file.seekg(0, std::ios::beg);
    
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    off_file.read(buffer, len);
    const char *at = &buffer[0];

    /* Build a default material */
    auto mat = new phong_shader(ext_colour_t(0.0f, 0.0f, 0.0f), ext_colour_t(170.0f, 170.0f, 170.0f), ext_colour_t(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f, 0.0f);
    m.push_back(mat);

    /* Check header */
    const std::string header(raptor_parsers::get_this_string(&at));
    BOOST_LOG_TRIVIAL(trace) << "File header: " << header;
    if ((header != "OFF") && (header != "OFF\r"))
    {
        BOOST_LOG_TRIVIAL(error) << "Unknown header: " << header;
        assert(false);
        return;
    }

    /* Get vertex and face counts */
    // raptor_parsers::find_next_line(&at);
    const unsigned int nr_v = raptor_parsers::get_this_unsigned(&at);
    const unsigned int nr_f = raptor_parsers::get_next_unsigned(&at);
    const unsigned int nr_e = raptor_parsers::get_next_unsigned(&at);
    BOOST_LOG_TRIVIAL(trace) << "Scene has vertices: " << nr_v << ", faces: " << nr_f << ", edges: " << nr_e;
    if (nr_e != 0)
    {
        BOOST_LOG_TRIVIAL(warning) << "Expected number of edges to be 0";
    }
    raptor_parsers::find_next_line(&at);

    /* Get vertices */
    point_t<> vert;
    std::vector<point_t<>> vertices;
    vertices.reserve(nr_v);
    for (unsigned int i = 0; i < nr_v; ++i)
    {
        vert.x = raptor_parsers::get_this_float(&at);
        vert.y = raptor_parsers::get_next_float(&at);
        vert.z = raptor_parsers::get_next_float(&at);
        vertices.push_back(vert);

        raptor_parsers::find_next_line(&at);
    }

    /* Get faces */
    std::vector<point_t<>> face;
    for (unsigned int i = 0; i < nr_f; ++i)
    {
        const unsigned int nr_verts = raptor_parsers::get_this_unsigned(&at);
        for (unsigned int j = 0; j < nr_verts; ++j)
        {
            const unsigned int vert_idx = raptor_parsers::get_next_unsigned(&at);
            face.push_back(vertices[vert_idx]);
        }
        raptor_parsers::find_next_line(&at);

        /* Create the polygon */
        if (face.size() == 3)
        {
            new_triangle(&e, nullptr, mat, face[0], face[1], face[2], false);
        }
        else if (face.size() > 3)
        {
            face_to_triangles(&e, &l, face, mat, false, nullptr, nullptr);
        }

        /* Clean up */
        face.clear();
    }

    /* Clean up */
    delete [] buffer;
}
}; /* namespace raptor_raytracer */
