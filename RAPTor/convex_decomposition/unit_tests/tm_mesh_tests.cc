#ifdef STAND_ALONE
#define BOOST_TEST_MODULE tm_mesh test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <list>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "tm_mesh.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct tm_mesh_fixture : private boost::noncopyable
{
    tm_mesh_fixture() :
        v0(edges.end(), point_t(1.1f, 2.3f, 3.5f), 6),
        v1(edges.end(), point_t(2.1f, 4.3f, 6.5f), 7),
        v2(edges.end(), point_t(1.5f, 3.3f, 4.5f), 8),
        vertices(build_vertices()),
        e0(triangles.end(), triangles.end(), triangles.end(), vertices.begin(), ++vertices.begin()),
        e1(triangles.end(), triangles.end(), triangles.end(), ++vertices.begin(), ++(++vertices.begin())),
        e2(triangles.end(), triangles.end(), triangles.end(), ++(++vertices.begin()), vertices.begin()),
        t0(edges.end(), edges.end(), edges.end(), vertices.end(), vertices.end(), vertices.end()),
        t1(edges.end(), edges.end(), edges.end(), vertices.begin(), ++vertices.begin(), ++(++vertices.begin()))
    {
        edges.push_back(e0);
        edges.push_back(e1);
        edges.push_back(e2);
        triangles.push_back(t0);
        triangles.push_back(t1);
    }

    std::list<tmm_triangle> triangles;
    tmm_vertex              v0;
    tmm_vertex              v1;
    tmm_vertex              v2;
    std::list<tmm_vertex>   vertices;
    tmm_edge                e0;
    tmm_edge                e1;
    tmm_edge                e2;
    std::list<tmm_edge>     edges;
    tmm_triangle            t0;
    tmm_triangle            t1;
    tm_mesh                 mesh;

    std::list<tmm_vertex> build_vertices()
    {
        std::list<tmm_vertex> ret;
        ret.push_back(v0);
        ret.push_back(v1);
        ret.push_back(v2);

        return ret;
    }
};

BOOST_FIXTURE_TEST_SUITE( tm_mesh_tests, tm_mesh_fixture )

const float result_tolerance = 0.0005f;


/* tmm_vertex tests */
BOOST_AUTO_TEST_CASE( vertex_ctor_test )
{
    BOOST_CHECK(v0.duplicate()  == edges.end());
    BOOST_CHECK(v0.position()   == point_t(1.1f, 2.3f, 3.5f));
    BOOST_CHECK(v0.name()       == 6);
    BOOST_CHECK(v0.id()         == 6);
    BOOST_CHECK(v0.on_hull()    == false);
}

BOOST_AUTO_TEST_CASE( vertex_duplicate_test )
{
    v0.duplicate(edges.begin());
    BOOST_CHECK(v0.duplicate() == edges.begin());
}

BOOST_AUTO_TEST_CASE( vertex_id_test )
{
    v0.id(16);
    BOOST_CHECK(v0.id() == 16);
}

BOOST_AUTO_TEST_CASE( vertex_on_hull_test )
{
    v0.on_hull(true);
    BOOST_CHECK(v0.on_hull() == true);

    v0.on_hull(false);
    BOOST_CHECK(v0.on_hull() == false);   
}

/* tmm_edge tests */
BOOST_AUTO_TEST_CASE( edge_ctor_test )
{
    BOOST_CHECK(e0.triangle(0)  == triangles.end());
    BOOST_CHECK(e0.triangle(1)  == triangles.end());
    BOOST_CHECK(e0.vertex(0)    == vertices.begin());
    BOOST_CHECK(e0.vertex(1)    == ++vertices.begin());
    BOOST_CHECK(e0.new_face()   == triangles.end());
}

BOOST_AUTO_TEST_CASE( edge_triangle_test )
{
    auto t_iter = triangles.begin();
    e0.triangle(t_iter++, 1);
    e0.triangle(t_iter++, 0);

    t_iter = triangles.begin();
    BOOST_CHECK(e0.triangle(1)  == t_iter++);
    BOOST_CHECK(e0.triangle(0)  == t_iter++);
}

BOOST_AUTO_TEST_CASE( edge_new_face_test )
{
    e0.new_face(triangles.begin());
    BOOST_CHECK(e0.new_face()   == triangles.begin());

    e0.new_face(++triangles.begin());
    BOOST_CHECK(e0.new_face()   == ++triangles.begin());
}

BOOST_AUTO_TEST_CASE( edge_vertex_test )
{
    auto v_iter = vertices.begin();
    e0.vertex(v_iter++, 1);
    e0.vertex(v_iter++, 0);

    v_iter = vertices.begin();
    BOOST_CHECK(e0.vertex(1)        == v_iter++);
    BOOST_CHECK(e0.vertex(0)        == v_iter++);
    BOOST_CHECK(e0.vertex(1)->id()  == 6);
    BOOST_CHECK(e0.vertex(0)->id()  == 7);
}

/* tmm_triangle tests */
BOOST_AUTO_TEST_CASE( triangle_ctor_test )
{
    BOOST_CHECK(t1.edge(0)      == edges.end());
    BOOST_CHECK(t1.edge(1)      == edges.end());
    BOOST_CHECK(t1.edge(2)      == edges.end());

    auto v_iter = vertices.begin();
    BOOST_CHECK(t1.vertex(0)        == v_iter++);
    BOOST_CHECK(t1.vertex(1)        == v_iter++);
    BOOST_CHECK(t1.vertex(2)        == v_iter++);
    BOOST_CHECK(t1.vertex(0)->id()  == 6);
    BOOST_CHECK(t1.vertex(1)->id()  == 7);
    BOOST_CHECK(t1.vertex(2)->id()  == 8);
    BOOST_CHECK(t1.visible()        == false);
}

BOOST_AUTO_TEST_CASE( triangle_volume_with_point_test )
{
    BOOST_CHECK_CLOSE(t1.volume_with_point(point_t(0.0f, 0.0f, 0.0f)),  0.059999f, result_tolerance);
    BOOST_CHECK_CLOSE(t1.volume_with_point(point_t(0.0f, 8.0f, 0.0f)), -1.539999f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( triangle_edge_test )
{
    auto e_iter = edges.begin();
    t0.edge(e_iter++, 2);
    t0.edge(e_iter++, 1);
    t0.edge(e_iter++, 0);

    e_iter = edges.begin();
    BOOST_CHECK(t0.edge(2)  == e_iter++);
    BOOST_CHECK(t0.edge(1)  == e_iter++);
    BOOST_CHECK(t0.edge(0)  == e_iter++);
}

BOOST_AUTO_TEST_CASE( triangle_vertex_test )
{
    auto v_iter = vertices.begin();
    t0.vertex(v_iter++, 2);
    t0.vertex(v_iter++, 1);
    t0.vertex(v_iter++, 0);

    v_iter = vertices.begin();
    BOOST_CHECK(t0.vertex(2)        == v_iter++);
    BOOST_CHECK(t0.vertex(1)        == v_iter++);
    BOOST_CHECK(t0.vertex(0)        == v_iter++);
    BOOST_CHECK(t0.vertex(2)->id()  == 6);
    BOOST_CHECK(t0.vertex(1)->id()  == 7);
    BOOST_CHECK(t0.vertex(0)->id()  == 8);
}

BOOST_AUTO_TEST_CASE( triangle_visible_test )
{
    t0.visible(true);
    BOOST_CHECK(t0.visible() == true);

    t0.visible(false);
    BOOST_CHECK(t0.visible() == false);
}

/* tm_mesh tests */
BOOST_AUTO_TEST_CASE( mesh_ctor_test )
{
    BOOST_CHECK(mesh.number_of_vertices()   == 0);
    BOOST_CHECK(mesh.number_of_edges()      == 0);
    BOOST_CHECK(mesh.number_of_triangles()  == 0);
    BOOST_CHECK(mesh.get_vertices().empty());
    BOOST_CHECK(mesh.get_edges().empty());
    BOOST_CHECK(mesh.get_triangles().empty());
}

BOOST_AUTO_TEST_CASE( mesh_const_get_vertices_test )
{
    const tm_mesh *const cmesh = &mesh;
    BOOST_CHECK(cmesh->get_vertices().empty());
}

BOOST_AUTO_TEST_CASE( mesh_const_get_edges_test )
{
    const tm_mesh *const cmesh = &mesh;
    BOOST_CHECK(cmesh->get_edges().empty());
}

BOOST_AUTO_TEST_CASE( mesh_const_get_triangles_test )
{
    const tm_mesh *const cmesh = &mesh;
    BOOST_CHECK(cmesh->get_triangles().empty());
}

BOOST_AUTO_TEST_CASE( mesh_add_vertex_test )
{
    /* Add vertex */
    mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    BOOST_REQUIRE(mesh.number_of_vertices() == 1);
    BOOST_CHECK(mesh.number_of_edges()      == 0);
    BOOST_CHECK(mesh.number_of_triangles()  == 0);

    /* Checks */
    auto v_iter = mesh.get_vertices().begin();
    BOOST_CHECK(v_iter->id()        == 0);
    BOOST_CHECK(v_iter->position()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(v_iter->name()      == 0);
    BOOST_CHECK(v_iter->duplicate() == mesh.get_edges().end());
    BOOST_CHECK(v_iter->on_hull()   == false);

    /* Add vertex */
    mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    BOOST_REQUIRE(mesh.number_of_vertices() == 2);
    BOOST_CHECK(mesh.number_of_edges()      == 0);
    BOOST_CHECK(mesh.number_of_triangles()  == 0);

    /* Checks */
    ++v_iter;
    BOOST_CHECK(v_iter->id()        == 1);
    BOOST_CHECK(v_iter->position()  == point_t(2.0f, 5.0f, 4.0f));
    BOOST_CHECK(v_iter->name()      == 1);
    BOOST_CHECK(v_iter->duplicate() == mesh.get_edges().end());
    BOOST_CHECK(v_iter->on_hull()   == false);
}

BOOST_AUTO_TEST_CASE( mesh_add_edge_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edge */
    mesh.add_edge(v0, v1);
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 1);
    BOOST_CHECK(mesh.number_of_triangles()  == 0);

    /* Checks */
    auto e_iter = mesh.get_edges().begin();
    BOOST_CHECK(e_iter->triangle(0)     == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->triangle(1)     == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)       == v0);
    BOOST_CHECK(e_iter->vertex(1)       == v1);
    BOOST_CHECK(e_iter->new_face()      == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)->id() == 0);
    BOOST_CHECK(e_iter->vertex(1)->id() == 1);

    /* Add edge */
    mesh.add_edge(v1, v2);
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 2);
    BOOST_CHECK(mesh.number_of_triangles()  == 0);

    /* Checks */
    ++e_iter;
    BOOST_CHECK(e_iter->triangle(0)     == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->triangle(1)     == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)       == v1);
    BOOST_CHECK(e_iter->vertex(1)       == v2);
    BOOST_CHECK(e_iter->new_face()      == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)->id() == 1);
    BOOST_CHECK(e_iter->vertex(1)->id() == 2);
}

BOOST_AUTO_TEST_CASE( mesh_add_triangle_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add triangle */
    mesh.add_triangle(v0, v1, v2);
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 0);
    BOOST_CHECK(mesh.number_of_triangles()  == 1);

    /* Checks */
    auto t_iter = mesh.get_triangles().begin();
    BOOST_CHECK(t_iter->edge(0)         == mesh.get_edges().end());
    BOOST_CHECK(t_iter->edge(1)         == mesh.get_edges().end());
    BOOST_CHECK(t_iter->edge(2)         == mesh.get_edges().end());
    BOOST_CHECK(t_iter->vertex(0)       == v0);
    BOOST_CHECK(t_iter->vertex(1)       == v1);
    BOOST_CHECK(t_iter->vertex(2)       == v2);
    BOOST_CHECK(t_iter->vertex(0)->id() == 0);
    BOOST_CHECK(t_iter->vertex(1)->id() == 1);
    BOOST_CHECK(t_iter->vertex(2)->id() == 2);
    BOOST_CHECK(t_iter->visible()       == false);

    /* Add triangle */
    mesh.add_triangle(v2, v1, v0);
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 0);
    BOOST_CHECK(mesh.number_of_triangles()  == 2);

    /* Checks */
    ++t_iter;
    BOOST_CHECK(t_iter->edge(0)         == mesh.get_edges().end());
    BOOST_CHECK(t_iter->edge(1)         == mesh.get_edges().end());
    BOOST_CHECK(t_iter->edge(2)         == mesh.get_edges().end());
    BOOST_CHECK(t_iter->vertex(0)       == v2);
    BOOST_CHECK(t_iter->vertex(1)       == v1);
    BOOST_CHECK(t_iter->vertex(2)       == v0);
    BOOST_CHECK(t_iter->vertex(0)->id() == 2);
    BOOST_CHECK(t_iter->vertex(1)->id() == 1);
    BOOST_CHECK(t_iter->vertex(2)->id() == 0);
    BOOST_CHECK(t_iter->visible()       == false);
}

BOOST_AUTO_TEST_CASE( mesh_add_triangle_with_edges_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangle */
    mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 3);
    BOOST_CHECK(mesh.number_of_triangles()  == 1);

    /* Checks */
    auto t_iter = mesh.get_triangles().begin();
    BOOST_CHECK(t_iter->edge(0)         == e0);
    BOOST_CHECK(t_iter->edge(1)         == e1);
    BOOST_CHECK(t_iter->edge(2)         == e2);
    BOOST_CHECK(t_iter->vertex(0)       == v0);
    BOOST_CHECK(t_iter->vertex(1)       == v1);
    BOOST_CHECK(t_iter->vertex(2)       == v2);
    BOOST_CHECK(t_iter->vertex(0)->id() == 0);
    BOOST_CHECK(t_iter->vertex(1)->id() == 1);
    BOOST_CHECK(t_iter->vertex(2)->id() == 2);
    BOOST_CHECK(t_iter->visible()       == false);

    /* Add triangle */
    mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 3);
    BOOST_CHECK(mesh.number_of_triangles()  == 2);

    /* Checks */
    ++t_iter;
    BOOST_CHECK(t_iter->edge(0)         == e2);
    BOOST_CHECK(t_iter->edge(1)         == e1);
    BOOST_CHECK(t_iter->edge(2)         == e0);
    BOOST_CHECK(t_iter->vertex(0)       == v2);
    BOOST_CHECK(t_iter->vertex(1)       == v1);
    BOOST_CHECK(t_iter->vertex(2)       == v0);
    BOOST_CHECK(t_iter->vertex(0)->id() == 2);
    BOOST_CHECK(t_iter->vertex(1)->id() == 1);
    BOOST_CHECK(t_iter->vertex(2)->id() == 0);
    BOOST_CHECK(t_iter->visible()       == false);
}

BOOST_AUTO_TEST_CASE( mesh_clear_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    mesh.add_triangle(e2, e1, e0, v2, v1, v0);

    /* Checks */
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 3);
    BOOST_CHECK(mesh.number_of_triangles()  == 2);

    /* Clear */
    mesh.clear();

    /* Checks */
    BOOST_CHECK(mesh.number_of_vertices()   == 0);
    BOOST_REQUIRE(mesh.number_of_edges()    == 0);
    BOOST_CHECK(mesh.number_of_triangles()  == 0);
}

BOOST_AUTO_TEST_CASE( mesh_points_and_triangles_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    mesh.add_triangle(e2, e1, e0, v2, v1, v0);

    std::vector<point_t>    points;
    std::vector<point_ti>   triangles;
    mesh.points_and_triangles(&points, &triangles);

    BOOST_CHECK(points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(points[1] == point_t(2.0f, 5.0f, 4.0f));
    BOOST_CHECK(points[2] == point_t(2.0f, 0.0f, 0.0f));

    BOOST_CHECK(triangles[0] == point_ti(0, 1, 2));
    BOOST_CHECK(triangles[1] == point_ti(2, 1, 0));
}

BOOST_AUTO_TEST_CASE( mesh_points_and_triangles_after_erase_test )
{
    /* Add vertices */
    const auto ve0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto ve1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto ve2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto ve3 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto ve4 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    mesh.add_triangle(e2, e1, e0, v2, v1, v0);

    /* Remove vertices */
    mesh.get_vertices().erase(ve0);
    mesh.get_vertices().erase(ve1);
    mesh.get_vertices().erase(ve2);
    mesh.get_vertices().erase(ve3);
    mesh.get_vertices().erase(ve4);

    std::vector<point_t>    points;
    std::vector<point_ti>   triangles;
    mesh.points_and_triangles(&points, &triangles);

    BOOST_CHECK(points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(points[1] == point_t(2.0f, 5.0f, 4.0f));
    BOOST_CHECK(points[2] == point_t(2.0f, 0.0f, 0.0f));

    BOOST_CHECK(triangles[0] == point_ti(0, 1, 2));
    BOOST_CHECK(triangles[1] == point_ti(2, 1, 0));
}

BOOST_AUTO_TEST_CASE( mesh_check_consistency_edge_with_no_triangles_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    const auto t0 = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto t1 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);

    /* Set triangles on edges */
    // e0->triangle(t0, 0);
    e1->triangle(t0, 0);
    e2->triangle(t0, 0);
    // e0->triangle(t1, 1);
    e1->triangle(t1, 1);
    e2->triangle(t1, 1);

    /* Checks */
    BOOST_CHECK(!mesh.check_consistency());

    /* Set one edge */
    e0->triangle(t0, 0);
    BOOST_CHECK(!mesh.check_consistency());

    /* Set other edge */
    e0->triangle(t1, 1);
    BOOST_CHECK(mesh.check_consistency());
}

BOOST_AUTO_TEST_CASE( mesh_check_consistency_no_t0_reference_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    const auto t0 = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto t1 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);

    /* Set triangles on edges */
    e0->triangle(t1, 0);
    e1->triangle(t0, 0);
    e2->triangle(t0, 0);
    e0->triangle(t1, 1);
    e1->triangle(t1, 1);
    e2->triangle(t1, 1);

    /* Checks */
    BOOST_CHECK(!mesh.check_consistency());
}

BOOST_AUTO_TEST_CASE( mesh_check_consistency_two_t0_reference_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    const auto t0 = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto t1 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);

    /* Set triangles on edges */
    e0->triangle(t0, 0);
    e1->triangle(t0, 0);
    e2->triangle(t0, 0);
    e0->triangle(t0, 1);
    e1->triangle(t1, 1);
    e2->triangle(t1, 1);

    /* Checks */
    BOOST_CHECK(!mesh.check_consistency());
}

BOOST_AUTO_TEST_CASE( mesh_check_consistency_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    const auto t0 = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto t1 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);

    /* Set triangles on edges */
    e0->triangle(t0, 0);
    e1->triangle(t0, 0);
    e2->triangle(t0, 0);
    e0->triangle(t1, 1);
    e1->triangle(t1, 1);
    e2->triangle(t1, 1);

    /* Checks */
    BOOST_CHECK(mesh.check_consistency());
}

BOOST_AUTO_TEST_CASE( mesh_make_face_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add face */
    const auto t0 = mesh.make_face(v0, v1, v2, mesh.get_triangles().end());
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 3);
    BOOST_CHECK(mesh.number_of_triangles()  == 1);

    /* Checks */
    auto e_iter = mesh.get_edges().begin();
    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)   == v0);
    BOOST_CHECK(e_iter->vertex(1)   == v1);
    BOOST_CHECK(t0->edge(0)         == e_iter++);

    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)   == v1);
    BOOST_CHECK(e_iter->vertex(1)   == v2);
    BOOST_CHECK(t0->edge(1)         == e_iter++);

    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)   == v2);
    BOOST_CHECK(e_iter->vertex(1)   == v0);
    BOOST_CHECK(t0->edge(2)         == e_iter++);

    BOOST_CHECK(t0->vertex(0)       == v0);
    BOOST_CHECK(t0->vertex(1)       == v1);
    BOOST_CHECK(t0->vertex(2)       == v2);
    BOOST_CHECK(t0->vertex(0)->id() == 0);
    BOOST_CHECK(t0->vertex(1)->id() == 1);
    BOOST_CHECK(t0->vertex(2)->id() == 2);
    BOOST_CHECK(t0->visible()       == false);
}

BOOST_AUTO_TEST_CASE( mesh_make_back_face_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add face */
    const auto t0 = mesh.make_face(v0, v1, v2, mesh.get_triangles().end());
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 3);
    BOOST_CHECK(mesh.number_of_triangles()  == 1);

    /* Checks */
    auto e_iter = mesh.get_edges().begin();
    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)   == v0);
    BOOST_CHECK(e_iter->vertex(1)   == v1);
    BOOST_CHECK(t0->edge(0)         == e_iter++);

    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)   == v1);
    BOOST_CHECK(e_iter->vertex(1)   == v2);
    BOOST_CHECK(t0->edge(1)         == e_iter++);

    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e_iter->vertex(0)   == v2);
    BOOST_CHECK(e_iter->vertex(1)   == v0);
    BOOST_CHECK(t0->edge(2)         == e_iter++);

    BOOST_CHECK(t0->vertex(0)       == v0);
    BOOST_CHECK(t0->vertex(1)       == v1);
    BOOST_CHECK(t0->vertex(2)       == v2);
    BOOST_CHECK(t0->vertex(0)->id() == 0);
    BOOST_CHECK(t0->vertex(1)->id() == 1);
    BOOST_CHECK(t0->vertex(2)->id() == 2);
    BOOST_CHECK(t0->visible()       == false);

    /* Add back face */
    const auto t1 = mesh.make_face(v2, v1, v0, t0);
    BOOST_CHECK(mesh.number_of_vertices()   == 3);
    BOOST_REQUIRE(mesh.number_of_edges()    == 3);
    BOOST_CHECK(mesh.number_of_triangles()  == 2);

    /* Checks */
    e_iter = mesh.get_edges().begin();
    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == t1);
    BOOST_CHECK(e_iter->vertex(0)   == v0);
    BOOST_CHECK(e_iter->vertex(1)   == v2);
    BOOST_CHECK(t1->edge(2)         == e_iter++);

    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == t1);
    BOOST_CHECK(e_iter->vertex(0)   == v1);
    BOOST_CHECK(e_iter->vertex(1)   == v0);
    BOOST_CHECK(t1->edge(1)         == e_iter++);

    BOOST_CHECK(e_iter->triangle(0) == t0);
    BOOST_CHECK(e_iter->triangle(1) == t1);
    BOOST_CHECK(e_iter->vertex(0)   == v2);
    BOOST_CHECK(e_iter->vertex(1)   == v1);
    BOOST_CHECK(t1->edge(0)         == e_iter++);

    BOOST_CHECK(t1->vertex(0)       == v2);
    BOOST_CHECK(t1->vertex(1)       == v1);
    BOOST_CHECK(t1->vertex(2)       == v0);
    BOOST_CHECK(t1->vertex(0)->id() == 2);
    BOOST_CHECK(t1->vertex(1)->id() == 1);
    BOOST_CHECK(t1->vertex(2)->id() == 0);
    BOOST_CHECK(t1->visible()       == false);

    BOOST_CHECK(t0->vertex(0)       == v0);
    BOOST_CHECK(t0->vertex(1)       == v1);
    BOOST_CHECK(t0->vertex(2)       == v2);
    BOOST_CHECK(t0->vertex(0)->id() == 0);
    BOOST_CHECK(t0->vertex(1)->id() == 1);
    BOOST_CHECK(t0->vertex(2)->id() == 2);
    BOOST_CHECK(t0->visible()       == false);
}

BOOST_AUTO_TEST_CASE( mesh_clean_triangles_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    const auto e1 = mesh.add_edge(v1, v2);
    const auto e2 = mesh.add_edge(v2, v0);

    /* Add triangles */
    const auto t0 = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto t1 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto t2 = mesh.add_triangle(v0, v1, v2);
    const auto t3 = mesh.add_triangle(v2, v1, v0);
    const auto t4 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto t5 = mesh.add_triangle(v0, v1, v2);

    /* Clean triangles */
    std::vector<tmm_triangle_iter> delete_triangles({ t0, t3, t4 });
    mesh.clean_triangles(&delete_triangles);

    /* Checks */
    BOOST_CHECK(delete_triangles.empty());

    auto t_iter = mesh.get_triangles().begin();
    BOOST_CHECK(t1 == t_iter++);
    BOOST_CHECK(t2 == t_iter++);
    BOOST_CHECK(t5 == t_iter++);
    BOOST_CHECK(mesh.get_triangles().end() == t_iter);
}

BOOST_AUTO_TEST_CASE( mesh_clean_edges_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0   = mesh.add_edge(v0, v1);
    const auto e1   = mesh.add_edge(v1, v2);
    const auto ed0  = mesh.add_edge(v0, v1);
    const auto e2   = mesh.add_edge(v2, v0);
    const auto ed1  = mesh.add_edge(v1, v2);
    const auto ed2  = mesh.add_edge(v2, v0);

    /* Add triangles */
    const auto t0 = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto t1 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto t2 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto t3 = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    e1->triangle(t0, 0);
    e2->triangle(t1, 0);
    e1->triangle(t1, 1);
    e2->triangle(t0, 1);
    e1->new_face(t2);
    e2->new_face(t3);
    t0->visible(true);

    /* Clean edges */
    std::vector<tmm_edge_iter> update_edges({ e0,  e1,  e2  });
    std::vector<tmm_edge_iter> delete_edges({ ed0, ed1, ed2 });
    mesh.clean_edges(&update_edges, &delete_edges);

    /* Checks */
    BOOST_CHECK(delete_edges.empty());
    BOOST_CHECK(update_edges.empty());

    BOOST_CHECK(e0->new_face() == mesh.get_triangles().end());
    BOOST_CHECK(e1->new_face() == mesh.get_triangles().end());
    BOOST_CHECK(e2->new_face() == mesh.get_triangles().end());

    BOOST_CHECK(e0->triangle(0) == mesh.get_triangles().end());
    BOOST_CHECK(e1->triangle(0) == t2);
    BOOST_CHECK(e2->triangle(0) == t1);
    BOOST_CHECK(e0->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e1->triangle(1) == t1);
    BOOST_CHECK(e2->triangle(1) == t3);

    auto e_iter = mesh.get_edges().begin();
    BOOST_CHECK(e0 == e_iter++);
    BOOST_CHECK(e1 == e_iter++);
    BOOST_CHECK(e2 == e_iter++);
    BOOST_CHECK(mesh.get_edges().end() == e_iter);
}

BOOST_AUTO_TEST_CASE( mesh_clean_vertices_test )
{
    /* Add vertices */
    const auto v0   = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 1);
    const auto v1   = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 2);
    mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 3);
    const auto v2   = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 4);
    const auto vd2  = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 5);
    const auto vd3  = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 6);

    /* Add edges */
    const auto e0 = mesh.add_edge(v0, v1);
    mesh.add_edge(v1, v2);
    mesh.add_edge(v2, v0);
    v1->duplicate(e0);

    /* Clean vertices */
    BOOST_CHECK(mesh.clean_vertices(vd2) == 2);

    /* Check which vertices are left */
    auto v_iter = mesh.get_vertices().begin();
    BOOST_CHECK(v0  == v_iter++);
    BOOST_CHECK(v1  == v_iter++);
    BOOST_CHECK(v2  == v_iter++);
    BOOST_CHECK(vd2 == v_iter++);
    BOOST_CHECK(vd3 == v_iter++);
    BOOST_CHECK(mesh.get_vertices().end() == v_iter);

    /* Check duplicate was cleaned */
    BOOST_CHECK(v1->duplicate() == mesh.get_edges().end());
}

BOOST_AUTO_TEST_CASE( mesh_clean_test )
{
    /* Add vertices */
    const auto v0   = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1   = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto v2   = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);
    const auto vd0  = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto vd1  = mesh.add_vertex(point_t(2.0f, 5.0f, 4.0f), 1);
    const auto vd2  = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 2);

    /* Add edges */
    const auto e0   = mesh.add_edge( v0,  v1);
    const auto e1   = mesh.add_edge( v1,  v2);
    const auto ed0  = mesh.add_edge(vd0, vd1);
    const auto e2   = mesh.add_edge( v2,  v0);
    const auto ed1  = mesh.add_edge(vd1, vd2);
    const auto ed2  = mesh.add_edge(vd2, vd0);

    /* Add triangles */
    const auto t0   = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto td0  = mesh.add_triangle(e0, e1, e2, v0, v1, v2);
    const auto t1   = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto td1  = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto t2   = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto t3   = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    const auto td3  = mesh.add_triangle(e2, e1, e0, v2, v1, v0);
    e1->triangle(t0, 0);
    e2->triangle(t1, 0);
    e1->triangle(t1, 1);
    e2->triangle(t0, 1);
    e1->new_face(t2);
    e2->new_face(t3);
    t0->visible(true);

    /* Clean */
    std::vector<tmm_triangle_iter> delete_triangles({ td0, td1, td3 });
    std::vector<tmm_edge_iter> update_edges({ e0,  e1,  e2  });
    std::vector<tmm_edge_iter> delete_edges({ ed0, ed1, ed2 });
    BOOST_CHECK(mesh.clean_up(&delete_triangles, &update_edges, &delete_edges, mesh.get_vertices().end()) == 3);
    BOOST_CHECK(delete_triangles.empty());
    BOOST_CHECK(update_edges.empty());
    BOOST_CHECK(delete_edges.empty());

    /* Check remaining triangles */
    auto t_iter = mesh.get_triangles().begin();
    BOOST_CHECK(t0 == t_iter++);
    BOOST_CHECK(t1 == t_iter++);
    BOOST_CHECK(t2 == t_iter++);
    BOOST_CHECK(t3 == t_iter++);
    BOOST_CHECK(mesh.get_triangles().end() == t_iter);

    /* Check remaining edges */
    BOOST_CHECK(e0->new_face() == mesh.get_triangles().end());
    BOOST_CHECK(e1->new_face() == mesh.get_triangles().end());
    BOOST_CHECK(e2->new_face() == mesh.get_triangles().end());

    BOOST_CHECK(e0->triangle(0) == mesh.get_triangles().end());
    BOOST_CHECK(e1->triangle(0) == t2);
    BOOST_CHECK(e2->triangle(0) == t1);
    BOOST_CHECK(e0->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e1->triangle(1) == t1);
    BOOST_CHECK(e2->triangle(1) == t3);

    auto e_iter = mesh.get_edges().begin();
    BOOST_CHECK(e0 == e_iter++);
    BOOST_CHECK(e1 == e_iter++);
    BOOST_CHECK(e2 == e_iter++);
    BOOST_CHECK(mesh.get_edges().end() == e_iter);

    /* Check remaining vertices */
    auto v_iter = mesh.get_vertices().begin();
    BOOST_CHECK(v0 == v_iter++);
    BOOST_CHECK(v1 == v_iter++);
    BOOST_CHECK(v2 == v_iter++);
    BOOST_CHECK(mesh.get_vertices().end() == v_iter);
}

BOOST_AUTO_TEST_CASE( mesh_make_cone_face_test )
{
    /* Add vertices */
    const auto v0 = mesh.add_vertex(point_t(0.0f, 0.0f, 0.0f), 0);
    const auto v1 = mesh.add_vertex(point_t(2.0f, 0.0f, 0.0f), 1);
    const auto v2 = mesh.add_vertex(point_t(0.0f, 2.0f, 0.0f), 2);
    const auto v3 = mesh.add_vertex(point_t(1.0f, 1.0f, 2.0f), 3);

    /* Add egdes */
    const auto e0 = mesh.add_edge(v0, v2);
    const auto e1 = mesh.add_edge(v3, v2);
    const auto e2 = mesh.add_edge(v3, v0);

    /* Add face */
    const auto t0 = mesh.add_triangle(e0, e1, e2, v0, v2, v3);
    const auto t1 = mesh.add_triangle(v0, v2, v3);
    t0->visible(true);
    e0->triangle(t0, 0);
    e1->triangle(t1, 0);
    e1->triangle(t0, 1);
    e2->triangle(t0, 0);

    BOOST_CHECK(mesh.number_of_vertices()   == 4);
    BOOST_CHECK(mesh.number_of_edges()      == 3);
    BOOST_CHECK(mesh.number_of_triangles()  == 2);

    /* Create new face */
    const auto t2 = mesh.make_cone_face(e0, v1);

    /* Checks */
    BOOST_CHECK(mesh.number_of_vertices()   == 4);
    BOOST_CHECK(mesh.number_of_edges()      == 5);
    BOOST_CHECK(mesh.number_of_triangles()  == 3);

    /* Create new face */
    const auto t3 = mesh.make_cone_face(e1, v1);

    /* Checks */
    BOOST_CHECK(mesh.number_of_vertices()   == 4);
    BOOST_CHECK(mesh.number_of_edges()      == 6);
    BOOST_CHECK(mesh.number_of_triangles()  == 4);

    /* Create new face */
    const auto t4 = mesh.make_cone_face(e2, v1);

    /* Checks */
    BOOST_CHECK(mesh.number_of_vertices()   == 4);
    BOOST_CHECK(mesh.number_of_edges()      == 6);
    BOOST_CHECK(mesh.number_of_triangles()  == 5);

    /* Check new triangles */
    BOOST_CHECK(t2->vertex(0) == v0);
    BOOST_CHECK(t2->vertex(1) == v2);
    BOOST_CHECK(t2->vertex(2) == v1);

    BOOST_CHECK(t3->vertex(0) == v2);
    BOOST_CHECK(t3->vertex(1) == v3);
    BOOST_CHECK(t3->vertex(2) == v1);
    
    BOOST_CHECK(t4->vertex(0) == v3);
    BOOST_CHECK(t4->vertex(1) == v0);
    BOOST_CHECK(t4->vertex(2) == v1);

    /* Check edges */
    auto e_iter = mesh.get_edges().begin();
    BOOST_CHECK(e0 == e_iter++);
    BOOST_CHECK(e1 == e_iter++);
    BOOST_CHECK(e2 == e_iter++);
    BOOST_CHECK(e0->triangle(0) == t0);
    BOOST_CHECK(e1->triangle(0) == t1);
    BOOST_CHECK(e2->triangle(0) == t0);
    BOOST_CHECK(e0->triangle(1) == mesh.get_triangles().end());
    BOOST_CHECK(e1->triangle(1) == t0);
    BOOST_CHECK(e2->triangle(1) == mesh.get_triangles().end());

    BOOST_CHECK(e_iter->triangle(0) == t2);
    BOOST_CHECK(e_iter->triangle(1) == t4);
    ++e_iter;
    BOOST_CHECK(e_iter->triangle(0) == t2);
    BOOST_CHECK(e_iter->triangle(1) == t3);
    ++e_iter;
    BOOST_CHECK(e_iter->triangle(0) == t3);
    BOOST_CHECK(e_iter->triangle(1) == t4);
    ++e_iter;
    BOOST_CHECK(e_iter == mesh.get_edges().end());
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
