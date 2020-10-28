#ifdef STAND_ALONE
#define BOOST_TEST_MODULE mesh_decimation test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Standard headers */

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Convex decomposition headers */
#include "mesh_decimation.h"


namespace raptor_mesh_decimation
{
namespace test
{
/* Test data */
struct mesh_decimation_fixture : private boost::noncopyable
{
    mesh_decimation_fixture()
    {

    }

    void add_vertices(std::initializer_list<point_t> v)
    {
        for (const auto &p : v)
        {
            vertices.emplace_back(p);
        }
    }

    void add_face(vertex *const v0, vertex *const v1, vertex *const v2)
    {
        faces.emplace_back(v0, v1, v2);
        v0->add_face(faces.back());
        v1->add_face(faces.back());
        v2->add_face(faces.back());
    }

    std::vector<vertex> vertices;
    std::vector<face>   faces;
    point_t             v_new;
};

BOOST_FIXTURE_TEST_SUITE( mesh_decimation_tests, mesh_decimation_fixture );

BOOST_AUTO_TEST_CASE( merge_error_triangle_test )
{
    add_vertices({ point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f) });
    add_face(&vertices[0], &vertices[1], &vertices[2]);


    std::cout << "merge cost " << vertices[0].merge_error(vertices[1], v_new) << std::endl;
    std::cout << "new vertex " << v_new << std::endl;
    // BOOST_CHECK(proj_vertices[0].index() == 0);
}

BOOST_AUTO_TEST_CASE( merge_error_square_test )
{
    add_vertices({ point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f) });
    add_face(&vertices[0], &vertices[1], &vertices[2]);
    add_face(&vertices[0], &vertices[2], &vertices[3]);

    std::cout << "merge cost " << vertices[0].merge_error(vertices[1], v_new) << std::endl;
    std::cout << "new vertex " << v_new << std::endl;
    // BOOST_CHECK(proj_vertices[0].index() == 0);
}

BOOST_AUTO_TEST_CASE( merge_error_square_with_center_test )
{
    add_vertices({ point_t(0.0f, 0.0f, 0.0f), point_t(-1.0f, -1.0f, 0.0f), point_t(1.0f, -1.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f), point_t(-1.0f, 1.0f, 0.0f) });
    add_face(&vertices[0], &vertices[1], &vertices[2]);
    add_face(&vertices[0], &vertices[2], &vertices[3]);
    add_face(&vertices[0], &vertices[3], &vertices[4]);
    add_face(&vertices[0], &vertices[4], &vertices[1]);

    std::cout << "merge cost " << vertices[0].merge_error(vertices[1], v_new) << std::endl;
    std::cout << "new vertex " << v_new << std::endl;
    // BOOST_CHECK(proj_vertices[0].index() == 0);
}

// BOOST_AUTO_TEST_CASE( merge_error_double_tet_test )
// {
//     add_vertices({ point_t(0.0f, 0.0f, 1.0f), point_t(-1.0f, -1.0f, 0.0f), point_t(1.0f, -1.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f), point_t(-1.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, -1.0f) });
//     add_face(&vertices[0], &vertices[1], &vertices[2]);
//     add_face(&vertices[0], &vertices[2], &vertices[3]);
//     add_face(&vertices[0], &vertices[3], &vertices[4]);
//     add_face(&vertices[0], &vertices[4], &vertices[1]);
//     add_face(&vertices[5], &vertices[2], &vertices[1]);
//     add_face(&vertices[5], &vertices[3], &vertices[2]);
//     add_face(&vertices[5], &vertices[4], &vertices[3]);
//     add_face(&vertices[5], &vertices[1], &vertices[4]);

//     std::cout << "merge cost " << vertices[0].merge_error(vertices[1], v_new) << std::endl;
//     std::cout << "new vertex " << v_new << std::endl;
//     // BOOST_CHECK(proj_vertices[0].index() == 0);
// }

BOOST_AUTO_TEST_SUITE_END()
} /* namespace test */
} /* namespace raptor_mesh_decimation */
