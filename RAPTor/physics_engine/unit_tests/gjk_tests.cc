#ifdef STAND_ALONE
#define BOOST_TEST_MODULE gjk test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Common headers*/
#include "quaternion_t.h"

/* Physics headers */
#include "simplex.h"

/* Test headers */
#include "gjk_test_access.h"


using namespace raptor_physics;

/* Test data */
class gjk_fixture
{
    public :
        gjk_fixture()
        : origin_a_verts(
            {
                point_t(-10.0, 0.0,  -5.0),
                point_t(-10.0, 0.0,  10.0),
                point_t( 10.0, 0.0,  10.0),
                point_t( 10.0, 0.0,  -5.0)
            }),
        origin_b_verts(
            {
                point_t(-0.5, -0.5, -0.5),
                point_t( 0.5, -0.5, -0.5),
                point_t( 0.5,  0.5, -0.5),
                point_t(-0.5,  0.5, -0.5),
                point_t(-0.5, -0.5,  0.5),
                point_t( 0.5, -0.5,  0.5),
                point_t( 0.5,  0.5,  0.5),
                point_t(-0.5,  0.5,  0.5)
            }),
        verts_a(
            {
                point_t(0.0, 0.0, 1.0),
                point_t(1.0, 0.0, 1.0),
                point_t(0.0, 1.0, 1.0),
                point_t(0.0, 0.0, 2.0)
            }),
        verts_b(
            {
                point_t(0.0, 0.0, 0.0)
            }),
        po_a(new physics_object(new vertex_group(verts_a, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0)),
        po_b(new physics_object(new vertex_group(verts_b, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0)),
        sim_a(new simplex(*po_a)),
        sim_b(new simplex(*po_b)),
        uut(*po_a->get_vertex_group(), *po_b->get_vertex_group(), sim_a.get(), sim_b.get()),
        verts{0, 0, 0, 0} {  };

        std::vector<point_t>    origin_a_verts;
        std::vector<point_t>    origin_b_verts;
        std::vector<point_t>    verts_a;
        std::vector<point_t>    verts_b;

    private :
        std::unique_ptr<physics_object> po_a;
        std::unique_ptr<physics_object> po_b;
        std::unique_ptr<simplex>        sim_a;
        std::unique_ptr<simplex>        sim_b;

    public :
        gjk_test_access uut;
        point_t         dir;
        int             verts[4];
};


BOOST_FIXTURE_TEST_SUITE( gjk_tests, gjk_fixture )

const float result_tolerance = 0.0001;

/* Test find support vertex */
BOOST_AUTO_TEST_CASE( find_support_vertex_test )
{
    std::vector<point_t> a_verts({
        point_t(0.5, 0.5, 1.0),
        point_t(1.5, 0.5, 1.0),
        point_t(0.5, 1.5, 1.0),
        point_t(0.5, 0.5, 2.0)
    });
    std::unique_ptr<matrix_3d> verts_a(new matrix_3d(a_verts));

    dir = point_t(0.0, 0.0, 1.0);
    int sv = find_support_vertex(*verts_a.get(), dir);
    BOOST_CHECK(sv == 3);

    dir = point_t(1.0, 0.0, 0.0);
    sv = find_support_vertex(*verts_a.get(), dir);
    BOOST_CHECK(sv == 1);

    dir = point_t(0.0, 1.0, 1.0);
    sv = find_support_vertex(*verts_a.get(), dir);
    BOOST_CHECK(sv == 2);
}


BOOST_AUTO_TEST_CASE( one_vertex_v0_test )
{
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(0.0, 0.0, 1.0),
        point_t(1.0, 0.0, 1.0),
        point_t(0.0, 1.0, 1.0),
        point_t(0.0, 0.0, 2.0)
    });

    /* Single vertex case, must return the vertex */
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);
    
    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.0, 0.0, 1.0);
    std::unique_ptr<int []> exp_verts(new int [4] { 0, 0, 0, 0 });

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( two_vertex_v0_test )
{
    uut.set_simplex_size(1);

    /* Vertex 0 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(0.1, 0.0, 1.0),
        point_t(1.0, 0.0, 1.0),
        point_t(0.0, 1.0, 1.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 1.0);
    const int exp_verts[4] = { 0, 0, 0, 0 };
    
    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( two_vertex_v1_test )
{
    uut.set_simplex_size(1);
    
    /* Vertex 1 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, 0.0, 1.0),
        point_t(-0.1, 0.0, 1.0),
        point_t(0.0, 1.0, 1.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(-0.1, 0.0, 1.0);
    const int exp_verts[4] = { 1, 0, 0, 0 };
    
    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
 }


BOOST_AUTO_TEST_CASE( two_vertex_e0_test )
{   
    uut.set_simplex_size(1);
    
    /* Edge side */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, 0.0, 3.0),
        point_t( 1.0, 0.0, 3.0),
        point_t( 0.0, 1.0, 3.0),
        point_t( 0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(0.0, 0.0, 3.0);
    const int exp_verts[4] = { 0, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( two_vertex_e1_test )
{
    uut.set_simplex_size(1);
    
    /* Other edge side */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, 0.0, -1.0),
        point_t(1.0, 0.0, -1.0),
        point_t(0.0, 1.0, 1.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(0.0, 0.0, -1.0);
    const int exp_verts[4] = { 0, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_v0_test )
{
    uut.set_simplex_size(2);
    
    /* Vertex 0 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(0.1, 0.0, 0.0),
        point_t(1.0, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 0.0);
    const int exp_verts[4] = { 0, 0, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_v1_test )
{
    uut.set_simplex_size(2);
    
    /* Vertex 1 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 0.0, 0.0),
        point_t(0.1, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 0.0);
    const int exp_verts[4] = { 1, 0, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_v2_test )
{
    uut.set_simplex_size(2);
    
    /* Vertex 2 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, 0.0, 0.0),
        point_t(0.1, 0.0, 0.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 0.0);
    const int exp_verts[4] = { 2, 0, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_e0_test )
{
    uut.set_simplex_size(2);
    
    /* Edge 0 - 1 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, 1.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(0.0, 2.0, 0.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(0.0, 1.0, 0.0);
    const int exp_verts[4] = { 0, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_e1_test )
{
    uut.set_simplex_size(2);
    
    /* Edge 1 - 2 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(2.0, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, -1.0, 0.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(1.0, 0.0, 0.0);
    const int exp_verts[4] = { 1, 2, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_e2_test )
{
    uut.set_simplex_size(2);
    
    /* Edge 2 - 0 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 0.0, -1.0),
        point_t(2.0, 0.0, 0.0),
        point_t(1.0, 0.0, 1.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(1.0, 0.0, 0.0);
    const int exp_verts[4] = { 0, 2, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_f0_test )
{
    uut.set_simplex_size(2);
    
    /* Face side */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, -1.0, 1.0),
        point_t(1.0, -1.0, 1.0),
        point_t(0.0, 1.0, 1.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(0.0, 0.0, 1.0);
    const int exp_verts[4] = { 0, 1, 2, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( three_vertex_f1_test )
{
    uut.set_simplex_size(2);
    
    /* Other face side */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, -1.0, -1.0),
        point_t(1.0, -1.0, -1.0),
        point_t(0.0, 1.0, -1.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(0.0, 0.0, -1.0);
    const int exp_verts[4] = { 2, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_v0_test )
{
    uut.set_simplex_size(3);
    
    /* Vertex 0 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(0.1, 0.0, 0.0),
        point_t(1.0, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, 0.0, 1.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 0.0);
    const int exp_verts[4] = { 0, 0, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_v1_test )
{
    uut.set_simplex_size(3);
    
    /* Vertex 1 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 0.0, 0.0),
        point_t(0.1, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, 0.0, 1.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 0.0);
    const int exp_verts[4] = { 1, 0, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_v2_test )
{
    uut.set_simplex_size(3);
    
    /* Vertex 2 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, 0.0, 0.0),
        point_t(0.1, 0.0, 0.0),
        point_t(1.0, 0.0, 1.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 0.0);
    const int exp_verts[4] = { 2, 0, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_v3_test )
{
    uut.set_simplex_size(3);
    
    /* Vertex 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, 0.0, 0.0),
        point_t(1.0, 0.0, 1.0),
        point_t(0.1, 0.0, 0.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 1;
    const point_t exp_dir(0.1, 0.0, 0.0);
    const int exp_verts[4] = { 3, 0, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_e0_test )
{
    uut.set_simplex_size(3);
    
    /* Edge 0 - 1 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, 1.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(0.0, 2.0, 0.0),
        point_t(0.0, 2.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(0.0, 1.0, 0.0);
    const int exp_verts[4] = { 0, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_e1_test )
{
    uut.set_simplex_size(3);
    
    /* Edge 1 - 2 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(2.0, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, -1.0, 0.0),
        point_t(2.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(1.0, 0.0, 0.0);
    const int exp_verts[4] = { 1, 2, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_e2_test )
{
    uut.set_simplex_size(3);
    
    /* Edge 2 - 0 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 0.0, -1.0),
        point_t(2.0, 0.0, 0.0),
        point_t(1.0, 0.0, 1.0),
        point_t(2.0, 2.0, 0.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(1.0, 0.0, 0.0);
    const int exp_verts[4] = { 0, 2, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_e3_test )
{
    uut.set_simplex_size(3);
    
    /* Edge 0 - 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, 0.0, -1.0),
        point_t(2.0, 2.0, 0.0),
        point_t(2.0, 0.0, 0.0),
        point_t(1.0, 0.0, 1.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(1.0, 0.0, 0.0);
    const int exp_verts[4] = { 0, 3, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_e4_test )
{
    uut.set_simplex_size(3);
    
    /* Edge 1 - 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(2.0, 0.0, 2.0),
        point_t(1.0, -1.0, 0.0),
        point_t(2.0, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(1.0, 0.0, 0.0);
    const int exp_verts[4] = { 1, 3, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_e5_test )
{
    uut.set_simplex_size(3);
    
    /* Edge 2 - 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(2.0, 0.0, 2.0),
        point_t(2.0, 0.0, 0.0),
        point_t(1.0, 1.0, 0.0),
        point_t(1.0, -1.0, 0.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(1.0, 0.0, 0.0);
    const int exp_verts[4] = { 2, 3, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_f0_test )
{
    uut.set_simplex_size(3);
    
    /* Face 0, 1, 2 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(0.0, 1.0, 1.0),
        point_t(1.0, -1.0, 1.0),
        point_t(-1.0, -1.0, 1.0),
        point_t(0.0, 0.0, 2.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(0.0, 0.0, 1.0);
    const int exp_verts[4] = { 2, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_f1_test )
{
    uut.set_simplex_size(3);
    
    /* Face 0, 1, 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-1.0, -1.0, 1.0),
        point_t(1.0, -1.0, 1.0),
        point_t(0.0, 0.0, 2.0),
        point_t(0.0, 1.0, 1.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(0.0, 0.0, 1.0);
    const int exp_verts[4] = { 1, 3, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_f2_test )
{
    uut.set_simplex_size(3);
    
    /* Face 0, 2, 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(1.0, -1.0, 1.0),
        point_t(0.0, 0.0, 2.0),
        point_t(-1.0, -1.0, 1.0),
        point_t(0.0, 1.0, 1.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(0.0, 0.0, 1.0);
    const int exp_verts[4] = { 0, 3, 2, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_f3_test )
{
    uut.set_simplex_size(3);
    
    /* Face 1, 2, 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(0.0, 0.0, 4.0),
        point_t(-1.0, -1.0, 3.0),
        point_t(1.0, -1.0, 3.0),
        point_t(0.0, 1.0, 3.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(0.0, 0.0, 3.0);
    const int exp_verts[4] = { 2, 3, 1, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( four_vertex_inside_test )
{
    uut.set_simplex_size(3);
    
    /* Volumne 0, 1, 2, 3 */
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(0.0, 0.0, 1.0),
        point_t(-1.0, -1.0, -1.0),
        point_t(1.0, -1.0, -1.0),
        point_t(0.0, 1.0, -1.0)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 4;
    const point_t exp_dir(0.0, 0.0, 0.0);
    const int exp_verts[4] = { 0, 1, 2, 3 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( special_0_test )
{
    uut.set_simplex_size(3);
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(6.95801, -5.15599,   8.5),
        point_t(-14.057, -5.14549,   8.5),
        point_t(6.94301, -5.14549, -12.5),
        point_t(6.94301, -5.14549,   8.5)
    });
    
    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(0.0, -5.14549, 0.0);
    const int exp_verts[4] = { 2, 3, 1, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( special_1_test )
{
    uut.set_simplex_size(3);
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(-6.0, -10.0,   8.0),
        point_t(-6.0, -10.0, -12.0),
        point_t( 3.0, -15.0,  12.0),
        point_t(-6.0, -10.0,   7.0)
    });

    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(-5.66038, -10.1887, 0.0);
    const int exp_verts[4] = { 2, 3, 1, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( special_2_test )
{
    uut.set_simplex_size(3);
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(  1.8, -11.5, -14.0),
        point_t( 10.3, -15.0,  11.0),
        point_t(-18.5, -11.0, -13.0),
        point_t(  1.4, -11.0,   6.0)
    });

    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 3;
    const point_t exp_dir(-0.260023, -11.1016, 0.27234);
    const int exp_verts[4] = { 0, 3, 2, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( special_3_test )
{
    uut.set_simplex_size(2);
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(  1.0, -11.5, -14.0),
        point_t( -1.5, -11.5, -14.0),
        point_t( -0.5, -11.5, -14.0),
        point_t(  1.4, -11.0,   6.0)
    });

    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(0.0, -11.5, -14.0);
    const int exp_verts[4] = {0, 2, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( special_4_test )
{
    uut.set_simplex_size(2);
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(  1.0, -11.5, -14.0),
        point_t( -1.5, -11.5, -14.0),
        point_t(  0.3, -11.5, -14.0),
        point_t(  1.4, -11.0,   6.0)
    });

    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(0.0, -11.5, -14.0);
    const int exp_verts[4] = {1, 2, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( special_5_test )
{
    uut.set_simplex_size(1);
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(3.0,  0.0, -0.000001),
        point_t(3.0,  0.0,  0.000001),
        point_t(4.0,  0.0,  0.0),
        point_t(0.0,  0.0,  2.0)
    });

    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(3.0, 0.0, -0.00001);
    const int exp_verts[4] = {0, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( special_6_test )
{
    uut.set_simplex_size(2);
    std::unique_ptr<point_t []> c_space(new point_t[4] {
        point_t(3.0,  0.0, -0.000001),
        point_t(3.0,  0.0,  0.000001),
        point_t(4.0,  0.0,  0.0),
        point_t(0.0,  0.0,  2.0)
    });

    const int size = uut.find_closest_feature_to_origin(c_space.get(), verts, &dir);

    /* Checks */
    const int exp_size = 2;
    const point_t exp_dir(3.0, 0.0, -0.00001);
    const int exp_verts[4] = {0, 1, 0, 0 };

    BOOST_CHECK(size == exp_size);
    for (int i = 0; i < exp_size; i++)
    {
        BOOST_CHECK(verts[i] == exp_verts[i]);
    }

    BOOST_CHECK(fabs(dir.x - exp_dir.x) < result_tolerance);
    BOOST_CHECK(fabs(dir.y - exp_dir.y) < result_tolerance);
    BOOST_CHECK(fabs(dir.z - exp_dir.z) < result_tolerance);
}


/* End to end tests */
BOOST_AUTO_TEST_CASE( basic_hit_test )
{
    std::vector<point_t> a_verts({
        point_t(-10.0, -10.0, -10.0),
        point_t(-10.0, -10.0,  10.0),
        point_t( 10.0, -10.0,  10.0),
        point_t( 10.0, -10.0, -10.0)
    });
    physics_object po_a(new vertex_group(a_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);
    
    std::vector<point_t> b_verts({
            point_t(-4.5, -10.25, -0.5),
            point_t(-3.5, -10.25, -0.5),
            point_t(-3.5,  -9.25, -0.5),
            point_t(-4.5,  -9.25, -0.5),
            point_t(-4.5, -10.25,  0.5),
            point_t(-3.5, -10.25,  0.5),
            point_t(-3.5,  -9.25,  0.5),
            point_t(-4.5,  -9.25,  0.5)
        });
    physics_object po_b(new vertex_group(b_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);

    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(test_gjk.find_minimum_distance(&dir, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
}


BOOST_AUTO_TEST_CASE( basic_miss_test )
{
    std::vector<point_t> a_verts({
            point_t(-10.0, -10.5, -10.0),
            point_t(-10.0, -10.5,  10.0),
            point_t( 10.0, -10.5,  10.0),
            point_t( 10.0, -10.5, -10.0)
        });
    physics_object po_a(new vertex_group(a_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);
    
    std::vector<point_t> b_verts({
            point_t(-4.5, -10.25, -0.5),
            point_t(-3.5, -10.25, -0.5),
            point_t(-3.5,  -9.25, -0.5),
            point_t(-4.5,  -9.25, -0.5),
            point_t(-4.5, -10.25,  0.5),
            point_t(-3.5, -10.25,  0.5),
            point_t(-3.5,  -9.25,  0.5),
            point_t(-4.5,  -9.25,  0.5)
        });
    physics_object po_b(new vertex_group(b_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);

    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk.find_minimum_distance(&dir, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
    BOOST_CHECK(dir == point_t(0.0, -0.25, 0.0));
}


BOOST_AUTO_TEST_CASE( displaced_hit_test )
{
    std::vector<point_t> a_verts({
        point_t(-10.0, 0.0, -10.0),
        point_t(-10.0, 0.0,  10.0),
        point_t( 10.0, 0.0,  10.0),
        point_t( 10.0, 0.0, -10.0)
    });
    physics_object po_a(new vertex_group(a_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);
    
    std::vector<point_t> b_verts({
        point_t(-4.5, -10.25, -0.5),
        point_t(-3.5, -10.25, -0.5),
        point_t(-3.5,  -9.25, -0.5),
        point_t(-4.5,  -9.25, -0.5),
        point_t(-4.5, -10.25,  0.5),
        point_t(-3.5, -10.25,  0.5),
        point_t(-3.5,  -9.25,  0.5),
        point_t(-4.5,  -9.25,  0.5)
    });
    physics_object po_b(new vertex_group(b_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);

    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(test_gjk.find_minimum_distance(&dir, point_t(0.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
}


BOOST_AUTO_TEST_CASE( displaced_miss_test )
{
    std::vector<point_t> a_verts({
        point_t(-10.0, -10.0, -10.0),
        point_t(-10.0, -10.0,  10.0),
        point_t( 10.0, -10.0,  10.0),
        point_t( 10.0, -10.0, -10.0)
    });
    physics_object po_a(new vertex_group(a_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);
    
    std::vector<point_t> b_verts({
        point_t(-4.5, -10.25, -0.5),
        point_t(-3.5, -10.25, -0.5),
        point_t(-3.5,  -9.25, -0.5),
        point_t(-4.5,  -9.25, -0.5),
        point_t(-4.5, -10.25,  0.5),
        point_t(-3.5, -10.25,  0.5),
        point_t(-3.5,  -9.25,  0.5),
        point_t(-4.5,  -9.25,  0.5)
    });
    physics_object po_b(new vertex_group(b_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);

    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk0(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk0.find_minimum_distance(&dir, point_t(0.0, -0.5, 0.0), point_t(0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
    BOOST_CHECK(dir == point_t(0.0, -0.25, 0.0));

    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk1(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk1.find_minimum_distance(&dir, point_t(0.0, 1.5, 0.0), point_t(0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
    BOOST_CHECK(dir == point_t(0.0, 0.75, 0.0));

    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk2(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk2.find_minimum_distance(&dir, point_t(0.0, 0.0, 11.0), point_t(0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
    BOOST_CHECK(dir == point_t(0.0, 0.0, 0.5));
}


BOOST_AUTO_TEST_CASE( displacing_hit_test )
{
    std::vector<point_t>  a_verts({
        point_t(-10.0, 0.0, -10.0),
        point_t(-10.0, 0.0,  10.0),
        point_t( 10.0, 0.0,  10.0),
        point_t( 10.0, 0.0, -10.0)
    });
    physics_object po_a(new vertex_group(a_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);
    
    std::vector<point_t>  b_verts({
        point_t(-4.5, -10.25, -0.5),
        point_t(-3.5, -10.25, -0.5),
        point_t(-3.5,  -9.25, -0.5),
        point_t(-4.5,  -9.25, -0.5),
        point_t(-4.5, -10.25,  0.5),
        point_t(-3.5, -10.25,  0.5),
        point_t(-3.5,  -9.25,  0.5),
        point_t(-4.5,  -9.25,  0.5)
    });
    physics_object po_b(new vertex_group(b_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);

    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk0(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(test_gjk0.find_minimum_distance(&dir, point_t(0.0, -7.0, 0.0), point_t(0.0, -4.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));

    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk1(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(test_gjk1.find_minimum_distance(&dir, point_t(0.0, -9.25, 0.0), point_t(0.0, 4.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
}


BOOST_AUTO_TEST_CASE( displacing_miss_test )
{
    std::vector<point_t> a_verts({
        point_t(-10.0, -10.0, -10.0),
        point_t(-10.0, -10.0,  10.0),
        point_t( 10.0, -10.0,  10.0),
        point_t( 10.0, -10.0, -10.0)
    });
    physics_object po_a(new vertex_group(a_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);
    
    std::vector<point_t> b_verts({
        point_t(-4.5, -10.25, -0.5),
        point_t(-3.5, -10.25, -0.5),
        point_t(-3.5,  -9.25, -0.5),
        point_t(-4.5,  -9.25, -0.5),
        point_t(-4.5, -10.25,  0.5),
        point_t(-3.5, -10.25,  0.5),
        point_t(-3.5,  -9.25,  0.5),
        point_t(-4.5,  -9.25,  0.5)
    });
    physics_object po_b(new vertex_group(b_verts, new std::vector<int>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);

    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk0(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk0.find_minimum_distance(&dir, point_t(0.0, -0.5, 0.0), point_t(0.0, -5.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
    BOOST_CHECK(dir == point_t(0.0, -0.25, 0.0));

    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk1(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk1.find_minimum_distance(&dir, point_t(0.0, 3.5, 0.0), point_t(0.0, -2.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
    BOOST_CHECK(dir == point_t(0.0, 0.75, 0.0));

    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk2(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk2.find_minimum_distance(&dir, point_t(0.0, 0.0, 11.0), point_t(5.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0), quaternion_t(1.0, 0.0, 0.0, 0.0)));
    BOOST_CHECK(dir == point_t(0.0, 0.0, 0.5));
}


BOOST_AUTO_TEST_CASE( rotated_hit_test )
{
    physics_object po_a(new vertex_group(origin_a_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    physics_object po_b(new vertex_group(origin_b_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5),  sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk0(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(test_gjk0.find_minimum_distance(&dir, point_t(0.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));

    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk1(*po_b.get_vertex_group(), *po_a.get_vertex_group(), sim_b.get(), sim_a.get());
    BOOST_CHECK(test_gjk1.find_minimum_distance(&dir, point_t(0.0, 10.0, 0.0), point_t(0.0, 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0)));
}


BOOST_AUTO_TEST_CASE( rotated_miss_test )
{
    physics_object po_a(new vertex_group(origin_a_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    physics_object po_b(new vertex_group(origin_b_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk0(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk0.find_minimum_distance(&dir, point_t(0.0, 0.0, -1.0), point_t(0.0, 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));
    BOOST_CHECK(fabs(magnitude(dir - point_t(0.0, 0.0, -0.5))) < result_tolerance);
    
    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk1(*po_b.get_vertex_group(), *po_a.get_vertex_group(), sim_b.get(), sim_a.get());
    BOOST_CHECK(!test_gjk1.find_minimum_distance(&dir, point_t(0.0, 0.0, -1.0), point_t(0.0, 0.0, -10.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));
    BOOST_CHECK(fabs(magnitude(dir - point_t(0.0, 0.0, -0.5))) < result_tolerance);
    
    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk2(*po_b.get_vertex_group(), *po_a.get_vertex_group(), sim_b.get(), sim_a.get());
    BOOST_CHECK(!test_gjk2.find_minimum_distance(&dir, point_t(0.0, 0.0, -1.0), point_t(0.0, 0.0, 0.49), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));
    BOOST_CHECK(fabs(magnitude(dir - point_t(0.0, 0.0, -0.01))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( rotated_translating_hit_test )
{
    physics_object po_a(new vertex_group(origin_a_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    physics_object po_b(new vertex_group(origin_b_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5),  sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk0(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(test_gjk0.find_minimum_distance(&dir, point_t(0.0, -10.0, -1.0), point_t(0.0, 0.0, 2.0), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));

    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk1(*po_b.get_vertex_group(), *po_a.get_vertex_group(), sim_b.get(), sim_a.get());
    BOOST_CHECK(test_gjk1.find_minimum_distance(&dir, point_t(-11.0, 10.0, 0.0), point_t(0.5, 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0)));
}


BOOST_AUTO_TEST_CASE( rotated_translating_miss_test )
{
    physics_object po_a(new vertex_group(origin_a_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    physics_object po_b(new vertex_group(origin_b_verts, new std::vector<int>(), nullptr), quaternion_t(sqrt(0.5), sqrt(0.5), 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);
    std::unique_ptr<simplex> sim_a(new simplex(po_a));
    std::unique_ptr<simplex> sim_b(new simplex(po_b));
    
    gjk test_gjk0(*po_a.get_vertex_group(), *po_b.get_vertex_group(), sim_a.get(), sim_b.get());
    BOOST_CHECK(!test_gjk0.find_minimum_distance(&dir, point_t(0.0, 0.0, -1.0), point_t(0.0, 10.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));
    BOOST_CHECK(fabs(magnitude(dir - point_t(0.0, 0.0, -0.5))) < result_tolerance);
    
    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk1(*po_b.get_vertex_group(), *po_a.get_vertex_group(), sim_b.get(), sim_a.get());
    BOOST_CHECK(!test_gjk1.find_minimum_distance(&dir, point_t(0.0, 0.0, -10.0), point_t(0.0, 0.0, 9.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));
    BOOST_CHECK(fabs(magnitude(dir - point_t(0.0, 0.0, -0.5))) < result_tolerance);
    
    sim_a.reset(new simplex(po_a));
    sim_b.reset(new simplex(po_b));
    gjk test_gjk2(*po_b.get_vertex_group(), *po_a.get_vertex_group(), sim_b.get(), sim_a.get());
    BOOST_CHECK(!test_gjk2.find_minimum_distance(&dir, point_t(0.0, 0.0, 10.0), point_t(0.0, 0.0, -9.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0), quaternion_t(sqrt(0.5), -sqrt(0.5), 0.0, 0.0)));
    BOOST_CHECK(fabs(magnitude(dir - point_t(0.0, 0.0, 0.5))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
