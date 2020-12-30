#ifdef STAND_ALONE
#define BOOST_TEST_MODULE dac_convex_hull_3d test
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <chrono>
#include <random>
#include <set>

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Common headers */
#include "logging.h"

/* Convex decomposition headers */
#include "dac_convex_hull_3d.h"


/* Initialise logger */
const raptor_physics::init_logger init_logger;

namespace raptor_convex_decomposition
{
int dac_convex_hull_3d::_merge_num = 0;

namespace test
{
#define LOG_PERFORMANCE_OUTPUT() \
    hull.compact_output(hull_vertices, hull_faces); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: " << boost::unit_test::framework::current_test_case().p_name; \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count(); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 2 - Unique vertices: " << vertices.size(); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 3 - Hull vertices: " << hull_vertices.size(); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 4 - Hull faces: " << hull_faces.size(); \
    BOOST_CHECK(save_vrml(hull));

struct edge_compare
{
    bool operator()(const std::shared_ptr<edge> &l, const std::shared_ptr<edge> &r)
    {
        if (l->start() != r->start())
        {
            return l->start() < r->start();
        }

        return l->end() < r->end();
    }
};

/* Test data */
struct dac_convex_hull_3d_fixture : private boost::noncopyable
{
    dac_convex_hull_3d_fixture() : 
        proj_vertices(std::make_shared<std::vector<projected_vertex>>()),
        scratch(std::make_shared<std::vector<projected_vertex>>()),
        uut(std::make_unique<dac_convex_hull_3d>(projected_hull(scratch, nullptr, nullptr), vertices, 0, 0))
    {
        vertices.reserve(128);
    }

    void create_face(const point_ti<> &t)
    {
        /* Create missing edges */
        assert(t.x < static_cast<int>(vertices.size()));
        assert(t.y < static_cast<int>(vertices.size()));
        assert(t.z < static_cast<int>(vertices.size()));
        vertex *v0 = &vertices[t.x];
        vertex *v1 = &vertices[t.y];
        vertex *v2 = &vertices[t.z];

        const auto &e0 = edges.emplace(std::make_shared<edge>(v0, v1), 0);
        if (e0.second)
        {
            v0->add_edge(e0.first->first);
            v1->add_edge(e0.first->first);
        }
        else
        {
            ++(e0.first->second);
            assert(e0.first->second < 2 || !"Non manifold mesh");
        }

        const auto &e1 = edges.emplace(std::make_shared<edge>(v1, v2), 0);
        if (e1.second)
        {
            v1->add_edge(e1.first->first);
            v2->add_edge(e1.first->first);
        }
        else
        {
            ++(e1.first->second);
            assert(e1.first->second < 2 || !"Non manifold mesh");
        }

        const auto &e2 = edges.emplace(std::make_shared<edge>(v2, v0), 0);
        if (e2.second)
        {
            v0->add_edge(e2.first->first);
            v2->add_edge(e2.first->first);
        }
        else
        {
            ++(e2.first->second);
            assert(e2.first->second < 2 || !"Non manifold mesh");
        }

        /* Create face */
        faces.emplace_back(std::make_shared<face>(v0, v1, v2));

        /* Add edges to face */
        auto &face = faces.back();
        face->edge_at(e0.first->first, 0);
        face->edge_at(e1.first->first, 1);
        face->edge_at(e2.first->first, 2);

        /* Add face to edges */
        e0.first->first->left_face(v0, face);
        e1.first->first->left_face(v1, face);
        e2.first->first->left_face(v2, face);

        /* Check face */
        assert(face->check());
    }

    /* Mainly use create_face, for edge only meshes */
    std::shared_ptr<edge> create_edge(const int v0, const int v1)
    {
        assert(v0 < static_cast<int>(vertices.size()));
        assert(v1 < static_cast<int>(vertices.size()));
        vertex *vert0 = &vertices[v0];
        vertex *vert1 = &vertices[v1];
        const auto &eit = edges.emplace(std::make_shared<edge>(vert0, vert1), 0);
        if (!eit.second)
        {
            ++(eit.first->second);
            assert(eit.first->second < 2 || !"Non manifold mesh");
        }

        /* Add edge to vertices */
        const auto &e = eit.first->first;
        vert0->add_edge(e);
        vert1->add_edge(e);
        return e;
    }

    std::shared_ptr<edge> lookup_edge(const int v0, const int v1)
    {
        if (const auto &fit = edges.find(std::make_shared<edge>(&vertices[v0], &vertices[v1])); fit == edges.end())
        {
            assert(!"Unable to look up edge");
            return nullptr;
        }
        else
        {
            return fit->first;
        }
    }

    projected_hull merge_projected_hull(const int start, const int end)
    {
        const int size = end - start;
        if (size < 3)
        {
            return projected_hull(scratch, &(*proj_vertices)[start], &(*proj_vertices)[end]);
        }

        const int mid = start + (size >> 1);
        auto left_hull(merge_projected_hull(start, mid));
        const auto right_hull(merge_projected_hull(mid, end));
        return left_hull.merge(right_hull);
    }

    std::tuple<std::unique_ptr<dac_convex_hull_3d>, std::unique_ptr<dac_convex_hull_3d>> make_hulls(const int mid)
    {
        scratch->resize(vertices.size(), projected_vertex(point2d<long>(0, 0), 0));
        for (const auto &v : vertices)
        {
            const auto &p = v.position();
            proj_vertices->emplace_back(point2d<long>(p.x, p.y), proj_vertices->size());
        }
        
        /* Clean duplicate projected */
        std::sort(&(*proj_vertices)[0], &(*proj_vertices)[mid]);
        std::sort(&(*proj_vertices)[mid], &(*proj_vertices)[proj_vertices->size()]);
        const auto &proj_left_end = std::unique(&(*proj_vertices)[0], &(*proj_vertices)[mid]);
        const auto &proj_right_end = std::unique(&(*proj_vertices)[mid], &(*proj_vertices)[proj_vertices->size()]);

        return
        {
            std::make_unique<dac_convex_hull_3d>(merge_projected_hull(0, std::distance(&(*proj_vertices)[0], proj_left_end)), vertices, 0, mid),
            std::make_unique<dac_convex_hull_3d>(merge_projected_hull(mid, std::distance(&(*proj_vertices)[0], proj_right_end)), vertices, mid, vertices.size())
        };
    }

    void collect_edges_and_faces()
    {
        /* Collect all the egdes from all the vertices */
        for (const auto &v : vertices)
        {
            for (int i = 0; i < v.edges(); ++i)
            {
                added_edges.emplace(v.edge_at(i));
            }
        }

        /* Collect all the faces from the unique edges collected */
        for (const auto &e : added_edges)
        {
            if (const auto &f = e->left_face(e->start()); f != nullptr)
            {
                added_faces.emplace(f);
            }

            if (const auto &f = e->right_face(e->start()); f != nullptr)
            {
                added_faces.emplace(f);
            }
        }
    }

    void consistency_check_edges(const int line, const size_t edges, const int allowed_null_faces = 0)
    {
        BOOST_CHECK_MESSAGE(added_edges.size() == edges, "line " << line << " wrong number of edges");
        for (auto [eit, i] = std::tuple{ added_edges.begin(), 0 }; eit != added_edges.end(); ++eit, ++i)
        {
            /* Check vertices are valid and unique */
            const auto &e = *eit;
            BOOST_CHECK_MESSAGE(e->end() != nullptr, "line " << line << " null end at edge " << i);
            BOOST_CHECK_MESSAGE(e->start() != nullptr, "line " << line << " null start at edge " << i);
            BOOST_CHECK_MESSAGE(e->start() != e->end(), "line " << line << " duplicate start and end at edge " << i);

            /* Check face are valid and unique */
            const auto &l = e->left_face(e->start());
            const auto &r = e->right_face(e->start());
            if (allowed_null_faces == 1)
            {
                BOOST_CHECK_MESSAGE((l != nullptr) || (r != nullptr), "line " << line << " null both faces at edge " << e->start()->position() << " to " << e->end()->position());
                BOOST_CHECK_MESSAGE(l != r, "line " << line << " duplicate left and right face at edge " << e->start()->position() << " to " << e->end()->position());
            }
            else if (allowed_null_faces == 0)
            {
                BOOST_CHECK_MESSAGE(l != nullptr, "line " << line << " null left face at edge " << e->start()->position() << " to " << e->end()->position());
                BOOST_CHECK_MESSAGE(r != nullptr, "line " << line << " null right at edge " << e->start()->position() << " to " << e->end()->position());
                BOOST_CHECK_MESSAGE(l != r, "line " << line << " duplicate left and right face at edge " << e->start()->position() << " to " << e->end()->position());
            }

            /* Check faces contain vertices */
            if (l != nullptr)
            {
                BOOST_CHECK_MESSAGE(l->contains(e->end()), "line " << line << " left does not contain end at edge " << e->start()->position() << " to " << e->end()->position());
                BOOST_CHECK_MESSAGE(l->contains(e->start()), "line " << line << " left does not contain start at edge " << e->start()->position() << " to " << e->end()->position());
            }

            if (r != nullptr)
            {
                BOOST_CHECK_MESSAGE(r->contains(e->end()), "line " << line << " right does not contain end at edge " << e->start()->position() << " to " << e->end()->position());
                BOOST_CHECK_MESSAGE(r->contains(e->start()), "line " << line << " right does not contain start at edge " << e->start()->position() << " to " << e->end()->position());
            }
        }
    }

    void consistency_check_faces(const int line, const size_t faces) const
    {
        BOOST_CHECK_MESSAGE(added_faces.size() == faces, "line " << line << " wrong number of faces, actual " << added_faces.size());
        for (auto [fit, i] = std::tuple{ added_faces.begin(), 0 }; fit != added_faces.end(); ++fit, ++i)
        {
            /* Check vertices are valid and unique */
            const auto &f = *fit;
            const auto &v0 = f->vertice(0);
            const auto &v1 = f->vertice(1);
            const auto &v2 = f->vertice(2);
            BOOST_CHECK_MESSAGE(v0 != nullptr, "line " << line << " null vertex 0 at face " << i);
            BOOST_CHECK_MESSAGE(v1 != nullptr, "line " << line << " null vertex 1 at face " << i);
            BOOST_CHECK_MESSAGE(v2 != nullptr, "line " << line << " null vertex 2 at face " << i);

            BOOST_CHECK_MESSAGE(v0 != v1, "line " << line << " duplicate vertex 0,1 at face " << i);
            BOOST_CHECK_MESSAGE(v1 != v2, "line " << line << " duplicate vertex 1,2 at face " << i);
            BOOST_CHECK_MESSAGE(v0 != v2, "line " << line << " duplicate vertex 2,0 at face " << i);

            /* Check edges are valid and unique */
            const auto &e0 = f->edge_at(0);
            const auto &e1 = f->edge_at(1);
            const auto &e2 = f->edge_at(2);
            BOOST_CHECK_MESSAGE(e0 != nullptr, "line " << line << " null edge 0 at face " << i);
            BOOST_CHECK_MESSAGE(e1 != nullptr, "line " << line << " null edge 1 at face " << i);
            BOOST_CHECK_MESSAGE(e2 != nullptr, "line " << line << " null edge 1 at face " << i);

            BOOST_CHECK_MESSAGE(e0 != e1, "line " << line << " duplicate edge 0,1 at face " << i);
            BOOST_CHECK_MESSAGE(e1 != e2, "line " << line << " duplicate edge 1,2 at face " << i);
            BOOST_CHECK_MESSAGE(e0 != e2, "line " << line << " duplicate edge 2,0 at face " << i);

            /* Check edges and vertices align */
            BOOST_CHECK_MESSAGE(e0->contains(v0), "line " << line << " edge 0 does not conatin vertex 0 at face " << i);
            BOOST_CHECK_MESSAGE(e0->contains(v1), "line " << line << " edge 0 does not conatin vertex 1 at face " << i);
            BOOST_CHECK_MESSAGE(e1->contains(v1), "line " << line << " edge 1 does not conatin vertex 1 at face " << i);
            BOOST_CHECK_MESSAGE(e1->contains(v2), "line " << line << " edge 1 does not conatin vertex 2 at face " << i);
            BOOST_CHECK_MESSAGE(e2->contains(v2), "line " << line << " edge 2 does not conatin vertex 2 at face " << i);
            BOOST_CHECK_MESSAGE(e2->contains(v0), "line " << line << " edge 2 does not conatin vertex 0 at face " << i);

            /* Check ordering */
            BOOST_CHECK_MESSAGE(e0->left_face(v0) == f, "line " << line << " edge 0 has incorrect left face at face " << i);
            BOOST_CHECK_MESSAGE(e1->left_face(v1) == f, "line " << line << " edge 1 has incorrect left face at face " << i);
            BOOST_CHECK_MESSAGE(e2->left_face(v2) == f, "line " << line << " edge 2 has incorrect left face at face " << i);
        }
    }

    void check_removals(const int line, const std::set<const vertex *> &removed_vertices)
    {
        /* Check vertices are marked correctly */
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            const auto &v = vertices[i];
            if (v.removed())
            {
                BOOST_CHECK_MESSAGE(removed_vertices.find(&v) != removed_vertices.end(), "line " << line << " incorrectly removed vertex " << i << " at " << v.position());
            }
            else
            {
                BOOST_CHECK_MESSAGE(removed_vertices.find(&v) == removed_vertices.end(), "line " << line << " incorrectly remaining vertex " << i << " at " << v.position());
            }
        }

        /* Check all edges are gone from removed vertices */
        for (const auto &v : removed_vertices)
        {
            BOOST_CHECK_MESSAGE(v->edges() == 0, "line " << line << " removed vertex with edges");
        }

        /* Check no edges lead to remove vertices */
        for (const auto &e : added_edges)
        {
            BOOST_CHECK_MESSAGE(removed_vertices.find(e->end()) == removed_vertices.end(), "line " << line << " edge end to removed vertex");
            BOOST_CHECK_MESSAGE(removed_vertices.find(e->start()) == removed_vertices.end(), "line " << line << " edge start to removed vertex");
        }

        /* Check no faces contain remove vertices */
        for (const auto &f : added_faces)
        {
            BOOST_CHECK_MESSAGE(removed_vertices.find(f->vertice(0)) == removed_vertices.end(), "line " << line << " face with removed vertex");
            BOOST_CHECK_MESSAGE(removed_vertices.find(f->vertice(1)) == removed_vertices.end(), "line " << line << " face with removed vertex");
            BOOST_CHECK_MESSAGE(removed_vertices.find(f->vertice(2)) == removed_vertices.end(), "line " << line << " face with removed vertex");
        }
    }

    std::shared_ptr<std::vector<projected_vertex>>      proj_vertices;
    std::shared_ptr<std::vector<projected_vertex>>      scratch;
    std::vector<vertex>                                 vertices;
    std::vector<const vertex *>                         hull_vertices;
    std::vector<point_ti<>>                             hull_faces;
    std::unique_ptr<dac_convex_hull_3d>                 uut;
    std::default_random_engine                          random;
    std::map<std::shared_ptr<edge>, int, edge_compare>  edges;
    std::vector<std::shared_ptr<face>>                  faces;
    std::vector<std::shared_ptr<edge>>                  wrapping_edges;
    std::set<std::shared_ptr<edge>>                     added_edges;
    std::set<std::shared_ptr<face>>                     added_faces;
    vertex *                                            l_iter;
    vertex *                                            r_iter;
};

BOOST_FIXTURE_TEST_SUITE( dac_convex_hull_3d_tests, dac_convex_hull_3d_fixture );


/* Is convex tests */
BOOST_AUTO_TEST_CASE( point_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(0, 0, 0));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(uut->is_convex());
}

BOOST_AUTO_TEST_CASE( edge_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));
    create_edge(0, 1);

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(uut->is_convex());
}

BOOST_AUTO_TEST_CASE( triangle_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));
    vertices.emplace_back(point_ti<long>( 0, 1, 0));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(0, 2, 1));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(uut->is_convex());
}

BOOST_AUTO_TEST_CASE( tetrahedron_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));
    vertices.emplace_back(point_ti<long>( 0, 1, 0));
    vertices.emplace_back(point_ti<long>( 0, 0, 1));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(2, 1, 3));
    create_face(point_ti<>(0, 2, 3));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(uut->is_convex());
}

BOOST_AUTO_TEST_CASE( tetrahedron_flip_face_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));
    vertices.emplace_back(point_ti<long>( 0, 1, 0));
    vertices.emplace_back(point_ti<long>( 0, 0, 1));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(2, 1, 3));
    create_face(point_ti<>(0, 3, 2));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(!uut->is_convex());
}

BOOST_AUTO_TEST_CASE( tetrahedron_flip_apex_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0,  0));
    vertices.emplace_back(point_ti<long>( 1, 0,  0));
    vertices.emplace_back(point_ti<long>( 0, 1,  0));
    vertices.emplace_back(point_ti<long>( 0, 0, -1));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(2, 1, 3));
    create_face(point_ti<>(0, 2, 3));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(!uut->is_convex());
}

BOOST_AUTO_TEST_CASE( cube_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1, -1,  1));
    vertices.emplace_back(point_ti<long>( 1, -1,  1));
    vertices.emplace_back(point_ti<long>( 1,  1,  1));
    vertices.emplace_back(point_ti<long>(-1,  1,  1));
    create_face(point_ti<>(0, 1, 3));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(5, 4, 7));
    create_face(point_ti<>(5, 7, 6));

    create_face(point_ti<>(1, 5, 2));
    create_face(point_ti<>(5, 6, 2));
    create_face(point_ti<>(0, 3, 4));
    create_face(point_ti<>(4, 3, 7));

    create_face(point_ti<>(7, 3, 6));
    create_face(point_ti<>(2, 6, 3));
    create_face(point_ti<>(0, 4, 1));
    create_face(point_ti<>(5, 1, 4));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(uut->is_convex());
}

BOOST_AUTO_TEST_CASE( cube_flip_face_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1, -1,  1));
    vertices.emplace_back(point_ti<long>( 1, -1,  1));
    vertices.emplace_back(point_ti<long>( 1,  1,  1));
    vertices.emplace_back(point_ti<long>(-1,  1,  1));
    create_face(point_ti<>(0, 1, 3));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(5, 4, 7));
    create_face(point_ti<>(5, 7, 6));

    create_face(point_ti<>(1, 5, 2));
    create_face(point_ti<>(5, 6, 2));
    create_face(point_ti<>(0, 3, 4));
    create_face(point_ti<>(4, 3, 7));

    create_face(point_ti<>(7, 3, 6));
    create_face(point_ti<>(2, 6, 3));
    create_face(point_ti<>(0, 4, 1));
    create_face(point_ti<>(5, 4, 1));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(!uut->is_convex());
}

BOOST_AUTO_TEST_CASE( cube_in_vertex_is_convex_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1, -1,  1));
    vertices.emplace_back(point_ti<long>( 1, -1,  1));
    vertices.emplace_back(point_ti<long>( 0,  0,  0));
    vertices.emplace_back(point_ti<long>(-1,  1,  1));
    create_face(point_ti<>(0, 1, 3));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(5, 4, 7));
    create_face(point_ti<>(5, 7, 6));

    create_face(point_ti<>(1, 5, 2));
    create_face(point_ti<>(5, 6, 2));
    create_face(point_ti<>(0, 3, 4));
    create_face(point_ti<>(4, 3, 7));

    create_face(point_ti<>(7, 3, 6));
    create_face(point_ti<>(2, 6, 3));
    create_face(point_ti<>(0, 4, 1));
    create_face(point_ti<>(5, 4, 1));

    uut.reset(new dac_convex_hull_3d(projected_hull(scratch, nullptr, nullptr), vertices, 0, vertices.size()));
    BOOST_CHECK(!uut->is_convex());
}


/* Wrapping tests */
BOOST_AUTO_TEST_CASE( wrap_points_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));

    l_iter = &vertices[0];
    r_iter = &vertices[1];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_CHECK(edges.empty());
}

BOOST_AUTO_TEST_CASE( wrap_points_reverse_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));

    l_iter = &vertices[1];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_CHECK(edges.empty());
}

BOOST_AUTO_TEST_CASE( wrap_point_edge_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    const auto &e0 = create_edge(1, 2);

    l_iter = &vertices[0];
    r_iter = &vertices[1];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == e0);
    BOOST_CHECK(edges[1] == e0);
}

BOOST_AUTO_TEST_CASE( wrap_edge_point_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    const auto &e0 = create_edge(1, 2);

    l_iter = &vertices[1];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == e0);
    BOOST_CHECK(edges[1] == e0);
}

BOOST_AUTO_TEST_CASE( wrap_point_reverse_edge_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    const auto &e0 = create_edge(1, 2);

    l_iter = &vertices[0];
    r_iter = &vertices[1];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == e0);
    BOOST_CHECK(edges[1] == e0);
}

BOOST_AUTO_TEST_CASE( wrap_reverse_edge_point_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    const auto &e0 = create_edge(1, 2);

    l_iter = &vertices[1];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == e0);
    BOOST_CHECK(edges[1] == e0);
}

BOOST_AUTO_TEST_CASE( wrap_edges_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    const auto &e0 = create_edge(0, 1);
    const auto &e1 = create_edge(2, 3);

    l_iter = &vertices[0];
    r_iter = &vertices[2];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 4);
    BOOST_CHECK(edges[0] == e0);
    BOOST_CHECK(edges[1] == e1);
    BOOST_CHECK(edges[2] == e0);
    BOOST_CHECK(edges[3] == e1);
}

BOOST_AUTO_TEST_CASE( wrap_edges_reverse_both_test )
{
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    const auto &e0 = create_edge(0, 1);
    const auto &e1 = create_edge(2, 3);

    l_iter = &vertices[0];
    r_iter = &vertices[2];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 4);
    BOOST_CHECK(edges[0] == e0);
    BOOST_CHECK(edges[1] == e1);
    BOOST_CHECK(edges[2] == e0);
    BOOST_CHECK(edges[3] == e1);
}

// Not valid start vertices because they arent a top tangent, one is a bottom tangent
// BOOST_AUTO_TEST_CASE( wrap_edges_reverse_l_test )
// {
//     vertices.emplace_back(point_ti<long>(-1,  1, 0));
//     vertices.emplace_back(point_ti<long>(-1, -1, 0));
//     vertices.emplace_back(point_ti<long>( 1, -1, 0));
//     vertices.emplace_back(point_ti<long>( 1,  1, 0));
//     const auto &e0 = create_edge(0, 1);
//     const auto &e1 = create_edge(2, 3);

    // l_iter = &vertices[0];
    // r_iter = &vertices[2];
//     const auto &edges = uut->wrap(&l_iter, &r_iter);
//     BOOST_REQUIRE(edges.size() == 4);
//     BOOST_CHECK(edges[0] == e0);
//     BOOST_CHECK(edges[1] == e0);
//     BOOST_CHECK(edges[2] == e1);
//     BOOST_CHECK(edges[3] == e1);
// }

// BOOST_AUTO_TEST_CASE( wrap_edges_reverse_r_test )
// {
//     vertices.emplace_back(point_ti<long>(-1, -1, 0));
//     vertices.emplace_back(point_ti<long>(-1,  1, 0));
//     vertices.emplace_back(point_ti<long>( 1,  1, 0));
//     vertices.emplace_back(point_ti<long>( 1, -1, 0));
//     const auto &e0 = create_edge(0, 1);
//     const auto &e1 = create_edge(2, 3);

    // l_iter = &vertices[0];
    // r_iter = &vertices[2];
//     const auto &edges = uut->wrap(&l_iter, &r_iter);
//     BOOST_REQUIRE(edges.size() == 4);
//     BOOST_CHECK(edges[0] == e0);
//     BOOST_CHECK(edges[1] == e0);
//     BOOST_CHECK(edges[2] == e1);
//     BOOST_CHECK(edges[3] == e1);
// }

/* Triangle versus point */
/* Show it doesnt matter which vertex on the triangle we start from */
BOOST_AUTO_TEST_CASE( wrap_point_triangle1_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[0];
    r_iter = &vertices[1];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(1, 3));
    BOOST_CHECK(edges[1] == lookup_edge(3, 2));
    BOOST_CHECK(edges[2] == lookup_edge(2, 1));
}

BOOST_AUTO_TEST_CASE( wrap_point_triangle2_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[0];
    r_iter = &vertices[2];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(2, 1));
    BOOST_CHECK(edges[1] == lookup_edge(1, 3));
    BOOST_CHECK(edges[2] == lookup_edge(3, 2));
}

BOOST_AUTO_TEST_CASE( wrap_point_triangle3_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[0];
    r_iter = &vertices[3];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(3, 2));
    BOOST_CHECK(edges[1] == lookup_edge(2, 1));
    BOOST_CHECK(edges[2] == lookup_edge(1, 3));
}

/* Show the left or right doesnt matter */
BOOST_AUTO_TEST_CASE( wrap_triangle_point_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[1];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(1, 2));
    BOOST_CHECK(edges[1] == lookup_edge(2, 3));
    BOOST_CHECK(edges[2] == lookup_edge(3, 1));
}

/* Show the triangle edge ordering doesnt matter */
BOOST_AUTO_TEST_CASE( wrap_point_reverse_triangle_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 2, 3));

    l_iter = &vertices[0];
    r_iter = &vertices[1];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(1, 3));
    BOOST_CHECK(edges[1] == lookup_edge(3, 2));
    BOOST_CHECK(edges[2] == lookup_edge(2, 1));
}

BOOST_AUTO_TEST_CASE( wrap_reverse_triangle_point_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 2, 3));

    l_iter = &vertices[1];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(1, 2));
    BOOST_CHECK(edges[1] == lookup_edge(2, 3));
    BOOST_CHECK(edges[2] == lookup_edge(3, 1));
}

/* Show the z order flips the removed face */
BOOST_AUTO_TEST_CASE( wrap_point_triangle_far_point_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 1));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[0];
    r_iter = &vertices[1];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(1, 2));
    BOOST_CHECK(edges[1] == lookup_edge(2, 3));
    BOOST_CHECK(edges[2] == lookup_edge(3, 1));
}

/* Triangle versus co planar point */
/* Show it doesnt matter where we start, NOTE 1 is not a valid tangent */
BOOST_AUTO_TEST_CASE( wrap_point_triangle2_co_planar_test )
{
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[2];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(2, 1));
    BOOST_CHECK(edges[1] == lookup_edge(1, 3));
    BOOST_CHECK(edges[2] == lookup_edge(3, 2));
}

BOOST_AUTO_TEST_CASE( wrap_point_triangle3_co_planar_test )
{
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[3];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(3, 2));
    BOOST_CHECK(edges[1] == lookup_edge(2, 1));
    BOOST_CHECK(edges[2] == lookup_edge(1, 3));
}

/* Show left or right doesnt matter */
BOOST_AUTO_TEST_CASE( wrap_triangle_point_co_planar_test )
{
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[0];
    r_iter = &vertices[2];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 3);
    BOOST_CHECK(edges[0] == lookup_edge(2, 1));
    BOOST_CHECK(edges[1] == lookup_edge(1, 3));
    BOOST_CHECK(edges[2] == lookup_edge(3, 2));
}

/* Show edge ordering doesnt matter */
BOOST_AUTO_TEST_CASE( wrap_point_reverse_triangle_co_planar_test )
{
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 2, 3));

    l_iter = &vertices[3];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 4);
    BOOST_CHECK(edges[0] == lookup_edge(3, 1));
    BOOST_CHECK(edges[1] == lookup_edge(1, 2));
    BOOST_CHECK(edges[2] == lookup_edge(2, 1));
    BOOST_CHECK(edges[3] == lookup_edge(1, 3));
}

BOOST_AUTO_TEST_CASE( wrap_reverse_triangle_point_co_planar_test )
{
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 2, 3));

    l_iter = &vertices[0];
    r_iter = &vertices[2];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 4);
    BOOST_CHECK(edges[0] == lookup_edge(2, 1));
    BOOST_CHECK(edges[1] == lookup_edge(1, 3));
    BOOST_CHECK(edges[2] == lookup_edge(3, 1));
    BOOST_CHECK(edges[3] == lookup_edge(1, 2));
}

/* Triangle with co linear point */
/* Show it doesnt matter where we start. NOTE 2 is not a valid tangent (co linear) */
BOOST_AUTO_TEST_CASE( wrap_point_triangle1_co_linear_test )
{
    vertices.emplace_back(point_ti<long>( 2, -1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[0];
    r_iter = &vertices[1];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == lookup_edge(1, 3));
    BOOST_CHECK(edges[1] == lookup_edge(3, 1));
}

BOOST_AUTO_TEST_CASE( wrap_point_triangle3_co_linear_test )
{
    vertices.emplace_back(point_ti<long>( 2, -1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[0];
    r_iter = &vertices[3];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == lookup_edge(3, 1));
    BOOST_CHECK(edges[1] == lookup_edge(1, 3));
}

/* Show left and right doesnt matter */
BOOST_AUTO_TEST_CASE( wrap_triangle_point_co_linear_test )
{
    vertices.emplace_back(point_ti<long>( 2, -1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));

    l_iter = &vertices[1];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == lookup_edge(1, 3));
    BOOST_CHECK(edges[1] == lookup_edge(3, 1));
}

/* Show edge ordering doesnt matter */
BOOST_AUTO_TEST_CASE( wrap_point_reverse_triangle_co_linear_test )
{
    vertices.emplace_back(point_ti<long>( 2, -1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 2, 3));

    l_iter = &vertices[0];
    r_iter = &vertices[3];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == lookup_edge(3, 1));
    BOOST_CHECK(edges[1] == lookup_edge(1, 3));
}

/* Show left and right doesnt matter */
BOOST_AUTO_TEST_CASE( wrap_reverse_triangle_point_co_linear_test )
{
    vertices.emplace_back(point_ti<long>( 2, -1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 2, 3));

    l_iter = &vertices[1];
    r_iter = &vertices[0];
    const auto &edges = uut->wrap(&l_iter, &r_iter);
    BOOST_REQUIRE(edges.size() == 2);
    BOOST_CHECK(edges[0] == lookup_edge(1, 3));
    BOOST_CHECK(edges[1] == lookup_edge(3, 1));
}


/* Add new faces tests */
BOOST_AUTO_TEST_CASE( edge_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));

    uut->add_new_faces(wrapping_edges, &vertices[0], &vertices[1]);
    BOOST_REQUIRE(vertices[0].edges() == 1);
    BOOST_REQUIRE(vertices[1].edges() == 1);
    BOOST_CHECK(vertices[0].edge_at(0) == vertices[1].edge_at(0));
    
    const auto &e = vertices[0].edge_at(0);
    BOOST_CHECK(e->start() == &vertices[0]);
    BOOST_CHECK(e->end() == &vertices[1]);
}

BOOST_AUTO_TEST_CASE( triangle_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(1, 2);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e0);

    uut->add_new_faces(wrapping_edges, &vertices[0], &vertices[1]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 3);
    consistency_check_faces(__LINE__, 2);
}

BOOST_AUTO_TEST_CASE( triangle_reverse_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(1, 2);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e0);

    uut->add_new_faces(wrapping_edges, &vertices[1], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 3);
    consistency_check_faces(__LINE__, 2);
}

BOOST_AUTO_TEST_CASE( square_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge in each hull */
    const auto &e0 = create_edge(0, 1);
    const auto &e1 = create_edge(2, 3);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e1);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e1);

    uut->add_new_faces(wrapping_edges, &vertices[0], &vertices[2]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 6);
    consistency_check_faces(__LINE__, 4);
}

BOOST_AUTO_TEST_CASE( square_reverse_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge in each hull */
    const auto &e0 = create_edge(0, 1);
    const auto &e1 = create_edge(2, 3);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e1);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e1);

    uut->add_new_faces(wrapping_edges, &vertices[2], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 6);
    consistency_check_faces(__LINE__, 4);
}

BOOST_AUTO_TEST_CASE( tetrahedron_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 3, 2));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(1, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 1));

    uut->add_new_faces(wrapping_edges, &vertices[0], &vertices[1]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 6);
    consistency_check_faces(__LINE__, 4);
}

BOOST_AUTO_TEST_CASE( tetrahedron_reverse_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 3, 2));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(1, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 1));

    uut->add_new_faces(wrapping_edges, &vertices[1], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 6);
    consistency_check_faces(__LINE__, 4);
}

BOOST_AUTO_TEST_CASE( cube_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, -1));
    vertices.emplace_back(point_ti<long>(-1, -1,  1));
    vertices.emplace_back(point_ti<long>(-1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1,  1,  1));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(1, 3, 2));

    vertices.emplace_back(point_ti<long>( 1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1, -1,  1));
    vertices.emplace_back(point_ti<long>( 1,  1, -1));
    vertices.emplace_back(point_ti<long>( 1,  1,  1));
    create_face(point_ti<>(4, 6, 5));
    create_face(point_ti<>(5, 6, 7));
    
    /* Wrap around individual squares */
    wrapping_edges.emplace_back(lookup_edge(7, 5));
    wrapping_edges.emplace_back(lookup_edge(3, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 0));
    wrapping_edges.emplace_back(lookup_edge(5, 4));
    wrapping_edges.emplace_back(lookup_edge(4, 6));
    wrapping_edges.emplace_back(lookup_edge(0, 2));
    wrapping_edges.emplace_back(lookup_edge(6, 7));
    wrapping_edges.emplace_back(lookup_edge(2, 3));

    uut->add_new_faces(wrapping_edges, &vertices[3], &vertices[7]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 18);
    consistency_check_faces(__LINE__, 12);
}

BOOST_AUTO_TEST_CASE( cube_reverse_add_new_faces_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, -1));
    vertices.emplace_back(point_ti<long>(-1, -1,  1));
    vertices.emplace_back(point_ti<long>(-1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1,  1,  1));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(1, 3, 2));

    vertices.emplace_back(point_ti<long>( 1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1, -1,  1));
    vertices.emplace_back(point_ti<long>( 1,  1, -1));
    vertices.emplace_back(point_ti<long>( 1,  1,  1));
    create_face(point_ti<>(4, 6, 5));
    create_face(point_ti<>(5, 6, 7));
    
    /* Wrap around individual squares */
    wrapping_edges.emplace_back(lookup_edge(3, 2));
    wrapping_edges.emplace_back(lookup_edge(7, 6));
    wrapping_edges.emplace_back(lookup_edge(6, 4));
    wrapping_edges.emplace_back(lookup_edge(2, 0));
    wrapping_edges.emplace_back(lookup_edge(4, 5));
    wrapping_edges.emplace_back(lookup_edge(0, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 3));
    wrapping_edges.emplace_back(lookup_edge(5, 7));

    uut->add_new_faces(wrapping_edges, &vertices[7], &vertices[3]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 18);
    consistency_check_faces(__LINE__, 12);
}


/* Remove hidden features tests */
BOOST_AUTO_TEST_CASE( edge_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[1]);
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 0, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( triangle_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(1, 2);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e0);

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[1]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 1, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( triangle_reverse_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(1, 2);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e0);

    uut->remove_hidden_features(wrapping_edges, &vertices[1], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 1, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( triangle_interior_edge_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(0, 1);
    const auto &e1 = create_edge(2, 3);
    wrapping_edges.emplace_back(e1);
    wrapping_edges.emplace_back(e1);

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[2]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 1, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {&vertices[1]});
}

BOOST_AUTO_TEST_CASE( triangle_interior_edge_reverse_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(0, 1);
    const auto &e1 = create_edge(2, 3);
    wrapping_edges.emplace_back(e1);
    wrapping_edges.emplace_back(e1);

    uut->remove_hidden_features(wrapping_edges, &vertices[2], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 1, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {&vertices[1]});
}

BOOST_AUTO_TEST_CASE( triangle_interior_face_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    vertices.emplace_back(point_ti<long>( 0, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  3, 0));
    vertices.emplace_back(point_ti<long>( 1, -3, 0));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(0, 2, 1));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(3, 4);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e0);

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[3]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 1, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {&vertices[1], &vertices[2]});
}

BOOST_AUTO_TEST_CASE( triangle_interior_face_reverse_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 0,  1, 0));
    vertices.emplace_back(point_ti<long>( 0, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  3, 0));
    vertices.emplace_back(point_ti<long>( 1, -3, 0));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(0, 2, 1));
    
    /* Wrap back and forth along the only edge */
    const auto &e0 = create_edge(3, 4);
    wrapping_edges.emplace_back(e0);
    wrapping_edges.emplace_back(e0);

    uut->remove_hidden_features(wrapping_edges, &vertices[3], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 1, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {&vertices[1], &vertices[2]});
}

BOOST_AUTO_TEST_CASE( square_remove_interior_edge_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 0,  0, 1));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(2, 3, 0));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(1, 3, 2));
    
    /* Wrap back and forth along the only edge in each hull */
    wrapping_edges.emplace_back(lookup_edge(0, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 0));

    /* Must start on an edge, there is gaurenteed to be an interior vertex, only edges */
    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[4]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 5, 1);
    consistency_check_faces(__LINE__, 2);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( square_remove_interior_reverse_edge_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 0,  0, 1));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(2, 3, 0));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(1, 3, 2));
    
    /* Wrap back and forth along the only edge in each hull */
    wrapping_edges.emplace_back(lookup_edge(0, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 0));

    /* Must start on an edge, there is gaurenteed to be an interior vertex, only edges */
    uut->remove_hidden_features(wrapping_edges, &vertices[4], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 5, 1);
    consistency_check_faces(__LINE__, 2);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( tetrahedron_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(1, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 1));

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[1]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 3, 1);
    consistency_check_faces(__LINE__, 2); /* The faces will be dropped when we add new faces, is this ok? */
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( tetrahedron_reverse_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>( 0,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 2));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(1, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 1));

    uut->remove_hidden_features(wrapping_edges, &vertices[1], &vertices[0]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 3, 1);
    consistency_check_faces(__LINE__, 2); /* The faces will be dropped when we add new faces, is this ok? */
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( hexagon_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 2,  0, 1));
    vertices.emplace_back(point_ti<long>( 1,  1, 1));
    vertices.emplace_back(point_ti<long>(-1,  1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(0, 1, 5));
    create_face(point_ti<>(1, 4, 5));
    create_face(point_ti<>(1, 2, 4));
    create_face(point_ti<>(2, 3, 4));

    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(0, 5, 3));
    create_face(point_ti<>(3, 5, 4));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(0, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 4));
    wrapping_edges.emplace_back(lookup_edge(4, 5));
    wrapping_edges.emplace_back(lookup_edge(5, 0));

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[6]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9, 1);
    consistency_check_faces(__LINE__, 4);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( hexagon_back_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 2,  0, 1));
    vertices.emplace_back(point_ti<long>( 1,  1, 1));
    vertices.emplace_back(point_ti<long>(-1,  1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(0, 1, 5));
    create_face(point_ti<>(1, 4, 5));
    create_face(point_ti<>(1, 2, 4));
    create_face(point_ti<>(2, 3, 4));

    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(0, 5, 3));
    create_face(point_ti<>(3, 5, 4));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(0, 5));
    wrapping_edges.emplace_back(lookup_edge(5, 4));
    wrapping_edges.emplace_back(lookup_edge(4, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 0));

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[6]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9, 1);
    consistency_check_faces(__LINE__, 4);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( hexagon_interior_triangle_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-5,  0, 0));
    vertices.emplace_back(point_ti<long>(-3, -3, 1));
    vertices.emplace_back(point_ti<long>( 3, -3, 1));
    vertices.emplace_back(point_ti<long>( 5,  0, 1));
    vertices.emplace_back(point_ti<long>( 3,  3, 1));
    vertices.emplace_back(point_ti<long>(-3,  3, 1));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(6, 7, 8));
    create_face(point_ti<>(0, 1, 6));
    create_face(point_ti<>(1, 2, 6));
    create_face(point_ti<>(2, 7, 6));
    create_face(point_ti<>(2, 3, 7));
    create_face(point_ti<>(3, 4, 7));
    create_face(point_ti<>(4, 8, 7));
    create_face(point_ti<>(4, 5, 8));
    create_face(point_ti<>(5, 0, 8));
    create_face(point_ti<>(0, 6, 8));

    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(0, 5, 3));
    create_face(point_ti<>(3, 5, 4));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(0, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 4));
    wrapping_edges.emplace_back(lookup_edge(4, 5));
    wrapping_edges.emplace_back(lookup_edge(5, 0));

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[9]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9, 1);
    consistency_check_faces(__LINE__, 4);
    check_removals(__LINE__, {&vertices[6], &vertices[7], &vertices[8]});
}

BOOST_AUTO_TEST_CASE( hexagon_interior_triangle_back_remove_hidden_features_test )
{
    vertices.emplace_back(point_ti<long>(-5,  0, 0));
    vertices.emplace_back(point_ti<long>(-3, -3, 1));
    vertices.emplace_back(point_ti<long>( 3, -3, 1));
    vertices.emplace_back(point_ti<long>( 5,  0, 1));
    vertices.emplace_back(point_ti<long>( 3,  3, 1));
    vertices.emplace_back(point_ti<long>(-3,  3, 1));
    vertices.emplace_back(point_ti<long>(-1, -1, 1));
    vertices.emplace_back(point_ti<long>( 1, -1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    vertices.emplace_back(point_ti<long>( 0,  1, 1));
    create_face(point_ti<>(6, 7, 8));
    create_face(point_ti<>(0, 1, 6));
    create_face(point_ti<>(1, 2, 6));
    create_face(point_ti<>(2, 7, 6));
    create_face(point_ti<>(2, 3, 7));
    create_face(point_ti<>(3, 4, 7));
    create_face(point_ti<>(4, 8, 7));
    create_face(point_ti<>(4, 5, 8));
    create_face(point_ti<>(5, 0, 8));
    create_face(point_ti<>(0, 6, 8));

    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(0, 5, 3));
    create_face(point_ti<>(3, 5, 4));
    
    /* Wrap around the triangle */
    wrapping_edges.emplace_back(lookup_edge(0, 5));
    wrapping_edges.emplace_back(lookup_edge(5, 4));
    wrapping_edges.emplace_back(lookup_edge(4, 3));
    wrapping_edges.emplace_back(lookup_edge(3, 2));
    wrapping_edges.emplace_back(lookup_edge(2, 1));
    wrapping_edges.emplace_back(lookup_edge(1, 0));

    uut->remove_hidden_features(wrapping_edges, &vertices[0], &vertices[9]);

    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 18, 1);
    consistency_check_faces(__LINE__, 10);
    check_removals(__LINE__, {});
}


/* Merge tests */
BOOST_AUTO_TEST_CASE( point_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1, 0, 0));
    vertices.emplace_back(point_ti<long>( 1, 0, 0));

    auto [l, r] = make_hulls(1);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 1, 2);
    consistency_check_faces(__LINE__, 0);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( point_edge_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 1,  0, 0));
    create_edge(0, 1);

    auto [l, r] = make_hulls(2);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 3);
    consistency_check_faces(__LINE__, 2);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( edge_interior_point_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  0, 0));
    create_edge(0, 1);
    create_edge(2, 3);

    auto [l, r] = make_hulls(2);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 3);
    consistency_check_faces(__LINE__, 2);
    check_removals(__LINE__, {&vertices[3]});
}

BOOST_AUTO_TEST_CASE( triangle_point_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  0, 1));
    vertices.emplace_back(point_ti<long>( 1,  0, 0));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(0, 2, 1));

    auto [l, r] = make_hulls(3);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 6);
    consistency_check_faces(__LINE__, 4);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( point_triangle_co_planar_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1,  0, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 2,  1, 0)); // point
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(0, 2, 1));

    auto [l, r] = make_hulls(3);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 6);
    consistency_check_faces(__LINE__, 4);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( squares_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, -1));
    vertices.emplace_back(point_ti<long>(-1,  1, -1));
    vertices.emplace_back(point_ti<long>(-1,  1,  1));
    vertices.emplace_back(point_ti<long>(-1, -1,  1));

    vertices.emplace_back(point_ti<long>( 1, -1, -1));
    vertices.emplace_back(point_ti<long>( 1,  1, -1));
    vertices.emplace_back(point_ti<long>( 1,  1,  1));
    vertices.emplace_back(point_ti<long>( 1, -1,  1));
    create_face(point_ti<>(0, 1, 3));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 0, 2));
    create_face(point_ti<>(0, 3, 2));

    create_face(point_ti<>(4, 5, 7));
    create_face(point_ti<>(5, 6, 7));
    create_face(point_ti<>(5, 4, 6));
    create_face(point_ti<>(4, 7, 6));

    auto [l, r] = make_hulls(4);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 18);
    consistency_check_faces(__LINE__, 12);
    check_removals(__LINE__, {});
}

BOOST_AUTO_TEST_CASE( squares_tetrahedron_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1, -2, -2));
    vertices.emplace_back(point_ti<long>(-1,  2, -2));
    vertices.emplace_back(point_ti<long>(-1,  2,  2));
    vertices.emplace_back(point_ti<long>(-1, -2,  2));

    vertices.emplace_back(point_ti<long>( 2, -1, -1));
    vertices.emplace_back(point_ti<long>( 2, -1,  1));
    vertices.emplace_back(point_ti<long>( 2,  1,  0));
    vertices.emplace_back(point_ti<long>( 1,  0,  0));
    create_face(point_ti<>(0, 1, 3));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 0, 2));
    create_face(point_ti<>(0, 3, 2));

    create_face(point_ti<>(4, 5, 6));
    create_face(point_ti<>(4, 6, 7));
    create_face(point_ti<>(5, 4, 7));
    create_face(point_ti<>(6, 5, 7));

    auto [l, r] = make_hulls(4);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 15);
    consistency_check_faces(__LINE__, 10);
    check_removals(__LINE__, {&vertices[7]});
}

BOOST_AUTO_TEST_CASE( hexagon_point_merge_test )
{
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 5,  0, 0));
    create_face(point_ti<>(0, 1, 5));
    create_face(point_ti<>(1, 4, 5));
    create_face(point_ti<>(1, 2, 4));
    create_face(point_ti<>(2, 3, 4));

    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(0, 5, 3));
    create_face(point_ti<>(3, 5, 4));

    auto [l, r] = make_hulls(6);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 15);
    consistency_check_faces(__LINE__, 10);
    check_removals(__LINE__, {}); /* Annoying we cant get rid of 3, but it beat returning to 1 on area*/
}

BOOST_AUTO_TEST_CASE( hexagon_point_flip_sides_merge_test )
{
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 5,  0, 0));
    create_face(point_ti<>(0, 5, 1));
    create_face(point_ti<>(1, 5, 4));
    create_face(point_ti<>(1, 4, 2));
    create_face(point_ti<>(2, 4, 3));

    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 0));
    create_face(point_ti<>(0, 3, 5));
    create_face(point_ti<>(3, 4, 5));

    auto [l, r] = make_hulls(6);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 15);
    consistency_check_faces(__LINE__, 10);
    check_removals(__LINE__, {}); /* Annoying we cant get rid of 3, but it beat 5 on area*/
}

BOOST_AUTO_TEST_CASE( hexagon_co_linear_point_merge_test )
{
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 5, -1, 0));
    create_face(point_ti<>(0, 1, 5));
    create_face(point_ti<>(1, 4, 5));
    create_face(point_ti<>(1, 2, 4));
    create_face(point_ti<>(2, 3, 4));

    create_face(point_ti<>(1, 3, 2));
    create_face(point_ti<>(1, 0, 3));
    create_face(point_ti<>(0, 5, 3));
    create_face(point_ti<>(3, 5, 4));

    auto [l, r] = make_hulls(6);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9);
    consistency_check_faces(__LINE__, 6);
    check_removals(__LINE__, {&vertices[2], &vertices[3]});
}

BOOST_AUTO_TEST_CASE( hexagon_co_linear_point_flip_sides_merge_test )
{
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>( 5, -1, 0));
    create_face(point_ti<>(0, 5, 1));
    create_face(point_ti<>(1, 5, 4));
    create_face(point_ti<>(1, 4, 2));
    create_face(point_ti<>(2, 4, 3));

    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 3, 0));
    create_face(point_ti<>(0, 3, 5));
    create_face(point_ti<>(3, 4, 5));

    auto [l, r] = make_hulls(6);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9);
    consistency_check_faces(__LINE__, 6);
    check_removals(__LINE__, {&vertices[2], &vertices[3]});
}

BOOST_AUTO_TEST_CASE( point_hexagon_merge_test )
{
    vertices.emplace_back(point_ti<long>(-5,  0, 0));
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    create_face(point_ti<>(1, 2, 6));
    create_face(point_ti<>(2, 5, 6));
    create_face(point_ti<>(2, 3, 5));
    create_face(point_ti<>(3, 4, 5));

    create_face(point_ti<>(2, 4, 3));
    create_face(point_ti<>(2, 1, 4));
    create_face(point_ti<>(1, 6, 4));
    create_face(point_ti<>(4, 6, 5));

    auto [l, r] = make_hulls(1);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 12);
    consistency_check_faces(__LINE__, 8);
    check_removals(__LINE__, {&vertices[1]});
}

BOOST_AUTO_TEST_CASE( point_hexagon_flip_sides_merge_test )
{
    vertices.emplace_back(point_ti<long>(-5,  0, 0));
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    create_face(point_ti<>(1, 6, 2));
    create_face(point_ti<>(2, 6, 5));
    create_face(point_ti<>(2, 5, 3));
    create_face(point_ti<>(3, 5, 4));

    create_face(point_ti<>(2, 3, 4));
    create_face(point_ti<>(2, 4, 1));
    create_face(point_ti<>(1, 4, 6));
    create_face(point_ti<>(4, 5, 6));

    auto [l, r] = make_hulls(1);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 12);
    consistency_check_faces(__LINE__, 8);
    check_removals(__LINE__, {&vertices[1]});
}

BOOST_AUTO_TEST_CASE( co_linear_point_hexagon_merge_test )
{
    vertices.emplace_back(point_ti<long>(-5,  1, 0));
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    create_face(point_ti<>(1, 2, 6));
    create_face(point_ti<>(2, 5, 6));
    create_face(point_ti<>(2, 3, 5));
    create_face(point_ti<>(3, 4, 5));

    create_face(point_ti<>(2, 4, 3));
    create_face(point_ti<>(2, 1, 4));
    create_face(point_ti<>(1, 6, 4));
    create_face(point_ti<>(4, 6, 5));

    auto [l, r] = make_hulls(1);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9);
    consistency_check_faces(__LINE__, 6);
    check_removals(__LINE__, {&vertices[1], &vertices[6]});
}

BOOST_AUTO_TEST_CASE( co_linear_point_hexagon_flip_sides_merge_test )
{
    vertices.emplace_back(point_ti<long>(-5,  1, 0));
    vertices.emplace_back(point_ti<long>(-2,  0, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 2,  0, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    create_face(point_ti<>(1, 6, 2));
    create_face(point_ti<>(2, 6, 5));
    create_face(point_ti<>(2, 5, 3));
    create_face(point_ti<>(3, 5, 4));

    create_face(point_ti<>(2, 3, 4));
    create_face(point_ti<>(2, 4, 1));
    create_face(point_ti<>(1, 4, 6));
    create_face(point_ti<>(4, 5, 6));

    auto [l, r] = make_hulls(1);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9);
    consistency_check_faces(__LINE__, 6);
    check_removals(__LINE__, {&vertices[1], &vertices[6]});
}

BOOST_AUTO_TEST_CASE( sqaure_edges_merge_test )
{
    vertices.emplace_back(point_ti<long>(-2, -1, 0));
    vertices.emplace_back(point_ti<long>(-2,  1, 0));
    vertices.emplace_back(point_ti<long>(-1,  1, 0));
    vertices.emplace_back(point_ti<long>(-1, -1, 0));

    vertices.emplace_back(point_ti<long>( 1, -1, 0));
    vertices.emplace_back(point_ti<long>( 1,  1, 0));
    vertices.emplace_back(point_ti<long>( 2,  1, 0));
    vertices.emplace_back(point_ti<long>( 2, -1, 0));
    create_face(point_ti<>(0, 1, 3));
    create_face(point_ti<>(1, 2, 3));
    create_face(point_ti<>(1, 0, 2));
    create_face(point_ti<>(0, 3, 2));

    create_face(point_ti<>(4, 5, 7));
    create_face(point_ti<>(5, 6, 7));
    create_face(point_ti<>(5, 4, 6));
    create_face(point_ti<>(4, 7, 6));

    auto [l, r] = make_hulls(4);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 6);
    consistency_check_faces(__LINE__, 4);
    check_removals(__LINE__, {&vertices[2], &vertices[3], &vertices[4], &vertices[5]});
}

BOOST_AUTO_TEST_CASE( y_triangles_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1,  5, -6));
    vertices.emplace_back(point_ti<long>(-1,  1, -6));
    vertices.emplace_back(point_ti<long>(-1,  7,  4));
    vertices.emplace_back(point_ti<long>(-1, -1, -2));
    vertices.emplace_back(point_ti<long>(-1, -4,  0));
    vertices.emplace_back(point_ti<long>(-1, -5,  3));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(0, 2, 1));
    create_face(point_ti<>(3, 4, 5));
    create_face(point_ti<>(3, 5, 4));

    auto [l, r] = make_hulls(3);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9);
    consistency_check_faces(__LINE__, 6);
    check_removals(__LINE__, {&vertices[3]});
}

BOOST_AUTO_TEST_CASE( y_triangles_reverse_merge_test )
{
    vertices.emplace_back(point_ti<long>(-1, -1, -2));
    vertices.emplace_back(point_ti<long>(-1, -4,  0));
    vertices.emplace_back(point_ti<long>(-1, -5,  3));
    vertices.emplace_back(point_ti<long>(-1,  5, -6));
    vertices.emplace_back(point_ti<long>(-1,  1, -6));
    vertices.emplace_back(point_ti<long>(-1,  7,  4));
    create_face(point_ti<>(0, 1, 2));
    create_face(point_ti<>(0, 2, 1));
    create_face(point_ti<>(3, 4, 5));
    create_face(point_ti<>(3, 5, 4));

    auto [l, r] = make_hulls(3);
    l->merge(*r);

    BOOST_CHECK(l->is_convex());
    collect_edges_and_faces();
    consistency_check_edges(__LINE__, 9);
    consistency_check_faces(__LINE__, 6);
    check_removals(__LINE__, {&vertices[0]});
}

/* Performance tests */
BOOST_AUTO_TEST_CASE( small_size_low_density_cube_point_cloud_performance_test )
{
    const long size = 10;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_medium_density_cube_point_cloud_performance_test )
{
    const long size = 10;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_high_density_cube_point_cloud_performance_test )
{
    const long size = 10;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_extreme_density_cube_point_cloud_performance_test )
{
    const long size = 10;
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( medium_size_low_density_cube_point_cloud_performance_test )
{
    const long size = 2000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_medium_density_cube_point_cloud_performance_test )
{
    const long size = 2000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_high_density_cube_point_cloud_performance_test )
{
    const long size = 2000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_extreme_density_cube_point_cloud_performance_test )
{
    const long size = 2000;
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( large_size_low_density_cube_point_cloud_performance_test )
{
    const long size = 200000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_medium_density_cube_point_cloud_performance_test )
{
    const long size = 200000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_high_density_cube_point_cloud_performance_test )
{
    const long size = 200000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_extreme_density_cube_point_cloud_performance_test )
{
    const long size = 20000;
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( extreme_size_low_density_cube_point_cloud_performance_test )
{
    const long size = 1100000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_medium_density_cube_point_cloud_performance_test )
{
    const long size = 1100000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_high_density_cube_point_cloud_performance_test )
{
    const long size = 250000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

/* Forms very large triangles, this is actually slightly larger than the expected worst case size */
BOOST_AUTO_TEST_CASE( extreme_size_extreme_density_cube_point_cloud_performance_test )
{
    const long size = 40000;
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-size, size);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(dist(random), dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( small_size_low_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_medium_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_high_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_extreme_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20.0;
    const int number_points = 1000000;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);

    for (int i = 0; i < number_points; ++i)
    {
        const float r = dist_r(random);
        const float pi = dist_pi(random);
        const float theta = dist_pi(random);
        vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( medium_size_low_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_medium_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_high_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_extreme_density_sphere_point_cloud_performance_test )
{
    const float size_r = 20000.0;
    const int number_points = 1000000;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);

    for (int i = 0; i < number_points; ++i)
    {
        const float r = dist_r(random);
        const float pi = dist_pi(random);
        const float theta = dist_pi(random);
        vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( large_size_low_density_sphere_point_cloud_performance_test )
{
    const float size_r = 200000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_medium_density_sphere_point_cloud_performance_test )
{
    const float size_r = 200000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_high_density_sphere_point_cloud_performance_test )
{
    const float size_r = 200000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_extreme_density_sphere_point_cloud_performance_test )
{
    const float size_r = 200000.0;
    const int number_points = 1000000;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);

    for (int i = 0; i < number_points; ++i)
    {
        const float r = dist_r(random);
        const float pi = dist_pi(random);
        const float theta = dist_pi(random);
        vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( extreme_size_low_density_sphere_point_cloud_performance_test )
{
    const float size_r = 1000000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_medium_density_sphere_point_cloud_performance_test )
{
    const float size_r = 1000000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_high_density_sphere_point_cloud_performance_test )
{
    const float size_r = 2000000.0;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();        
        for (int i = 0; i < number_points; ++i)
        {
            const float r = dist_r(random);
            const float pi = dist_pi(random);
            const float theta = dist_pi(random);
            vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_extreme_density_sphere_point_cloud_performance_test )
{
    const float size_r = 5000000.0;
    const int number_points = 1000000;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);

    for (int i = 0; i < number_points; ++i)
    {
        const float r = dist_r(random);
        const float pi = dist_pi(random);
        const float theta = dist_pi(random);
        vertices.emplace_back(point_ti<long>(r * sin(pi) * cos(theta), r * sin(pi) * sin(theta), r * cos(pi)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( small_size_low_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-10, 10);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_medium_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-10, 10);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_high_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-10, 10);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( small_size_extreme_density_yz_plane_point_cloud_performance_test )
{
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-10, 10);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( medium_size_low_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-2000, 2000);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_medium_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-2000, 2000);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_high_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-2000, 2000);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( medium_size_extreme_density_yz_plane_point_cloud_performance_test )
{
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-2000, 2000);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( large_size_low_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-200000, 200000);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_medium_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-200000, 200000);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_high_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-200000, 200000);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( large_size_extreme_density_yz_plane_point_cloud_performance_test )
{
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-200000, 200000);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_CASE( extreme_size_low_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-2000000000, 2000000000);
    for (int number_points = 10; number_points < 210; number_points += 10)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_medium_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-5000000000, 5000000000);
    for (int number_points = 200; number_points < 2100; number_points += 100)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_high_density_yz_plane_point_cloud_performance_test )
{
    std::uniform_int_distribution<long> dist(-15000000000, 15000000000);
    for (int number_points = 3000; number_points < 21000; number_points += 1000)
    {
        vertices.clear();
        hull_faces.clear();
        hull_vertices.clear();
        proj_vertices->clear();
        for (int i = 0; i < number_points; ++i)
        {
            vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
        }

        auto t0(std::chrono::system_clock::now());
        const auto hull(build(*proj_vertices, vertices));
        auto t1(std::chrono::system_clock::now());

        LOG_PERFORMANCE_OUTPUT();
        BOOST_CHECK(hull.is_convex());
    }
}

BOOST_AUTO_TEST_CASE( extreme_size_extreme_density_yz_plane_point_cloud_performance_test )
{
    const int number_points = 1000000;
    std::uniform_int_distribution<long> dist(-140000000000, 140000000000);
    for (int i = 0; i < number_points; ++i)
    {
        vertices.emplace_back(point_ti<long>(0, dist(random), dist(random)));
    }

    auto t0(std::chrono::system_clock::now());
    const auto hull(build(*proj_vertices, vertices));
    auto t1(std::chrono::system_clock::now());

    LOG_PERFORMANCE_OUTPUT();
    BOOST_CHECK(hull.is_convex());
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
