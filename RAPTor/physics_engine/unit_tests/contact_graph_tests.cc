#ifdef STAND_ALONE
#define BOOST_TEST_MODULE contact_graph test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <new>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "vertex_group.h"
#include "physics_object.h"
#include "contact_graph.h"

/* Test headers */
#include "mock_physics_object.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct contact_graph_base_fixture : private boost::noncopyable
{
    contact_graph_base_fixture()
    : vg_memory{},
    po0( new (&vg_memory[alignof(mock_physics_object)                                     ]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667, 2.66667, 2.66667, 0.0,  0.0, 0.0 }, point_t( 2.5, 0.5,  2.45 ), 4.0))),
    po1( new (&vg_memory[alignof(mock_physics_object) + ( 1 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 2.5, 0.5,  0.45 ), 2.0))),
    po2( new (&vg_memory[alignof(mock_physics_object) + ( 2 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 2.5, 0.5, -2.45 ), 2.0))),
    po3( new (&vg_memory[alignof(mock_physics_object) + ( 3 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 2.5, 0.5, -4.45 ), 2.0))),
    po4( new (&vg_memory[alignof(mock_physics_object) + ( 4 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 2.5, 0.5, -6.45 ), 2.0))),
    po5( new (&vg_memory[alignof(mock_physics_object) + ( 5 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 1.5, 0.5,  2.45 ), 2.0))),
    po6( new (&vg_memory[alignof(mock_physics_object) + ( 6 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667, 2.66667, 2.66667, 0.0,  0.0, 0.0 }, point_t( 2.5, 0.5,  0.45 ), 4.0))),
    po7( new (&vg_memory[alignof(mock_physics_object) + ( 7 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 1.5, 0.5, -2.45 ), 2.0))),
    po8( new (&vg_memory[alignof(mock_physics_object) + ( 8 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667, 2.66667, 2.66667, 0.0,  0.0, 0.0 }, point_t( 0.5, 0.5,  0.45 ), 4.0))),
    po9( new (&vg_memory[alignof(mock_physics_object) + ( 9 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667, 2.66667, 2.66667, 0.0,  0.0, 0.0 }, point_t( 1.0, 0.0, -2.3  ), 4.0))),
    po10(new (&vg_memory[alignof(mock_physics_object) + (10 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667, 2.66667, 2.66667, 0.0,  0.0, 0.0 }, point_t( 1.5, 0.5,  1.45000001), 4.0))),
    po11(new (&vg_memory[alignof(mock_physics_object) + (11 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 2.5, 1.5,  1.44444449), 2.0))),
    po12(new (&vg_memory[alignof(mock_physics_object) + (12 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667, 2.66667, 2.66667, 0.0,  0.0, 0.0 }, point_t( 1.5, 0.5,  1.45000001), 4.0))),
    po13(new (&vg_memory[alignof(mock_physics_object) + (13 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 0.0,  0.0, 0.0 }, point_t( 2.5, 1.5,  1.44444449), std::numeric_limits<float>::infinity()))),
    data0( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5,  1.5       ) }))),
    data1( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5,  1.4       ) }))),
    data2( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5, -1.4       ) }))),
    data3( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5, -1.5       ) }))),
    data4( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5, -3.4       ) }))),
    data5( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5, -3.5       ) }))),
    data6( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5, -5.4       ) }))),
    data7( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5, -5.5       ) }))),
    data8( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(0.5,  0.5, -1.5       ) }))),
    data9( physics_object_for_simplex_testing(std::vector<point_t>({ point_t(0.5,  0.5, -1.4       ) }))),
    data10(physics_object_for_simplex_testing(std::vector<point_t>({ point_t(0.5,  0.5,  1.4       ) }))),
    data11(physics_object_for_simplex_testing(std::vector<point_t>({ point_t(0.5,  0.5,  1.5       ) }))),
    data12(physics_object_for_simplex_testing(std::vector<point_t>({ point_t(1.0,  0.0,  0.04999999) }), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 1.5, 0.5,  1.45000001))),
    data13(physics_object_for_simplex_testing(std::vector<point_t>({ point_t(0.0, -1.0, -0.04444449) }), quaternion_t(1.0, 0.0, 0.0, 0.0), point_t( 2.5, 1.5,  1.44444449))),
    data14(physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5,  1.5       ) }))),
    data15(physics_object_for_simplex_testing(std::vector<point_t>({ point_t(2.5,  0.5,  1.4       ) }))),
    s0( *data0 ),
    s1( *data1 ),
    s2( *data2 ),
    s3( *data3 ),
    s4( *data4 ),
    s5( *data5 ),
    s6( *data6 ),
    s7( *data7 ),
    s8( *data8 ),
    s9( *data9 ),
    s10(*data10),
    s11(*data11),
    s12(*data12),
    s13(*data13),
    s14(*data14),
    s15(*data15),
    uut(nullptr)
    {  };

    virtual ~contact_graph_base_fixture()
    {
        po0->~mock_physics_object();
        po1->~mock_physics_object();
        po2->~mock_physics_object();
        po3->~mock_physics_object();
        po4->~mock_physics_object();
        po5->~mock_physics_object();
        po6->~mock_physics_object();
        po7->~mock_physics_object();
        po8->~mock_physics_object();
        po9->~mock_physics_object();
        po10->~mock_physics_object();
        po11->~mock_physics_object();
        po12->~mock_physics_object();
        po13->~mock_physics_object();

        for (auto &p : info)
        {
            delete p.second;
        }

        delete uut;
    }

    /* Raw memory to hold the vg */
    /* It is important the addresses are fixed relative to one another or the graph vertices are in slightly different orders */
    char vg_memory[sizeof(mock_physics_object) * 15];

    mock_physics_object *const po0;
    mock_physics_object *const po1;
    mock_physics_object *const po2;
    mock_physics_object *const po3;
    mock_physics_object *const po4;
    mock_physics_object *const po5;
    mock_physics_object *const po6;
    mock_physics_object *const po7;
    mock_physics_object *const po8;
    mock_physics_object *const po9;
    mock_physics_object *const po10;
    mock_physics_object *const po11;
    mock_physics_object *const po12;
    mock_physics_object *const po13;
    std::unique_ptr<physics_object> data0;
    std::unique_ptr<physics_object> data1;
    std::unique_ptr<physics_object> data2;
    std::unique_ptr<physics_object> data3;
    std::unique_ptr<physics_object> data4;
    std::unique_ptr<physics_object> data5;
    std::unique_ptr<physics_object> data6;
    std::unique_ptr<physics_object> data7;
    std::unique_ptr<physics_object> data8;
    std::unique_ptr<physics_object> data9;
    std::unique_ptr<physics_object> data10;
    std::unique_ptr<physics_object> data11;
    std::unique_ptr<physics_object> data12;
    std::unique_ptr<physics_object> data13;
    std::unique_ptr<physics_object> data14;
    std::unique_ptr<physics_object> data15;
    simplex s0;
    simplex s1;
    simplex s2;
    simplex s3;
    simplex s4;
    simplex s5;
    simplex s6;
    simplex s7;
    simplex s8;
    simplex s9;
    simplex s10;
    simplex s11;
    simplex s12;
    simplex s13;
    simplex s14;
    simplex s15;

    std::map<mock_physics_object*, tracking_info<mock_physics_object>*> info;
    contact_graph<mock_physics_object> *uut;
};


struct contact_graph_2_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_contact_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object>(po1, new simplex(s0), s1, 0.0, SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object>(po0, new simplex(s1), s0, 0.0, SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object>(info, po0);
    };
};


struct contact_graph_2_offset_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_offset_contact_fixture()
    {
        info = {{po10, new tracking_info<mock_physics_object>(po11, new simplex(s12), s13, 0.0, SLIDING_COLLISION)},
                {po11, new tracking_info<mock_physics_object>(po10, new simplex(s13), s12, 0.0, SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object>(info, po10);
    };
};


struct contact_graph_2_infinite_mass_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_infinite_mass_contact_fixture()
    {
        info = {{po12, new tracking_info<mock_physics_object>(po13, new simplex(s14), s15, 0.0, SLIDING_COLLISION)},
                {po13, new tracking_info<mock_physics_object>(po12, new simplex(s15), s14, 0.0, SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object>(info, po12);
    };
};


struct contact_graph_4_circle_fixture : public contact_graph_base_fixture
{
    contact_graph_4_circle_fixture()
    {
        info= {{po0, new tracking_info<mock_physics_object>(po1, new simplex( s0),  s1, 0.0, SLIDING_COLLISION)},
               {po1, new tracking_info<mock_physics_object>(po2, new simplex( s2),  s3, 0.0, SLIDING_COLLISION)},
               {po2, new tracking_info<mock_physics_object>(po6, new simplex( s8),  s9, 0.0, SLIDING_COLLISION)},
               {po6, new tracking_info<mock_physics_object>(po0, new simplex(s10), s11, 0.0, SLIDING_COLLISION)}};

        info[po1]->update(po0, new simplex( s1),  s0, 0.0, SLIDING_COLLISION);
        info[po2]->update(po1, new simplex( s3),  s2, 0.0, SLIDING_COLLISION);
        info[po6]->update(po2, new simplex( s9),  s8, 0.0, SLIDING_COLLISION);
        info[po0]->update(po6, new simplex(s11), s10, 0.0, SLIDING_COLLISION);

        uut = new contact_graph<mock_physics_object>(info, po0);
    };
};


struct contact_graph_5_p_fixture : public contact_graph_base_fixture
{
    contact_graph_5_p_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object>(po1, new simplex( s0),  s1, 0.0, SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object>(po2, new simplex( s2),  s3, 0.0, SLIDING_COLLISION)},
                {po2, new tracking_info<mock_physics_object>(po6, new simplex( s8),  s9, 0.0, SLIDING_COLLISION)},
                {po6, new tracking_info<mock_physics_object>(po0, new simplex(s10), s11, 0.0, SLIDING_COLLISION)},
                {po4, new tracking_info<mock_physics_object>(po2, new simplex( s5),  s4, 0.0, SLIDING_COLLISION)}};

        info[po1]->update(po0, new simplex( s1),  s0, 0.0, SLIDING_COLLISION);
        info[po2]->update(po1, new simplex( s3),  s2, 0.0, SLIDING_COLLISION);
        info[po6]->update(po2, new simplex( s9),  s8, 0.0, SLIDING_COLLISION);
        info[po0]->update(po6, new simplex(s11), s10, 0.0, SLIDING_COLLISION);
        info[po2]->update(po4, new simplex( s4),  s5, 0.0, SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object>(info, po0);
    };
};


struct contact_graph_5_stacked_fixture : public contact_graph_base_fixture
{
    contact_graph_5_stacked_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object>(po1, new simplex(s0), s1, 0.0, SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object>(po2, new simplex(s2), s3, 0.0, SLIDING_COLLISION)},
                {po2, new tracking_info<mock_physics_object>(po3, new simplex(s4), s5, 0.0, SLIDING_COLLISION)},
                {po3, new tracking_info<mock_physics_object>(po4, new simplex(s6), s7, 0.0, SLIDING_COLLISION)},
                {po4, new tracking_info<mock_physics_object>(po3, new simplex(s7), s6, 0.0, SLIDING_COLLISION)}};

        info[po1]->update(po0, new simplex(s1), s0, 0.0, SLIDING_COLLISION);
        info[po2]->update(po1, new simplex(s3), s2, 0.0, SLIDING_COLLISION);
        info[po3]->update(po2, new simplex(s5), s4, 0.0, SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object>(info, po0);
    };
};


struct contact_graph_9_disjoint_fixture : public contact_graph_base_fixture
{
    contact_graph_9_disjoint_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object>(po1, new simplex(s0), s1, 0.0, SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object>(po2, new simplex(s2), s3, 0.0, SLIDING_COLLISION)},
                {po2, new tracking_info<mock_physics_object>(po3, new simplex(s4), s5, 0.0, SLIDING_COLLISION)},
                {po3, new tracking_info<mock_physics_object>(po4, new simplex(s6), s7, 0.0, SLIDING_COLLISION)},
                {po4, new tracking_info<mock_physics_object>(po3, new simplex(s7), s6, 0.0, SLIDING_COLLISION)},

                {po5, new tracking_info<mock_physics_object>(po6, new simplex( s0),  s1, 0.0, SLIDING_COLLISION)},
                {po6, new tracking_info<mock_physics_object>(po7, new simplex( s2),  s3, 0.0, SLIDING_COLLISION)},
                {po7, new tracking_info<mock_physics_object>(po8, new simplex( s8),  s9, 0.0, SLIDING_COLLISION)},
                {po8, new tracking_info<mock_physics_object>(po5, new simplex(s10), s11, 0.0, SLIDING_COLLISION)}};

        info[po1]->update(po0, new simplex(s1), s0, 0.0, SLIDING_COLLISION);
        info[po2]->update(po1, new simplex(s3), s2, 0.0, SLIDING_COLLISION);
        info[po3]->update(po2, new simplex(s5), s4, 0.0, SLIDING_COLLISION);

        info[po6]->update(po5, new simplex( s1),  s0, 0.0, SLIDING_COLLISION);
        info[po7]->update(po6, new simplex( s3),  s2, 0.0, SLIDING_COLLISION);
        info[po8]->update(po7, new simplex( s9),  s8, 0.0, SLIDING_COLLISION);
        info[po5]->update(po8, new simplex(s11), s10, 0.0, SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object>(info, po0);
    };
};


BOOST_AUTO_TEST_SUITE( contact_graph_tests )

const float result_tolerance = 0.0005;


BOOST_AUTO_TEST_CASE( default_ctor_test )
{
    /* Construct */
    contact_graph<mock_physics_object> uut;

    /* Check graph */
    BOOST_CHECK(uut.number_of_vertices() == 0);
    BOOST_CHECK(uut.number_of_edges() == 0);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_ctor_test, contact_graph_2_contact_fixture )
{
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 2);
    BOOST_CHECK(uut->number_of_edges() == 1);
    BOOST_CHECK(uut->vertices()[0] == po0);
    BOOST_CHECK(uut->vertices()[1] == po1);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 2);
    BOOST_CHECK(uut->number_of_edges() == 1);
    BOOST_CHECK(uut->vertices()[0] == po1);
    BOOST_CHECK(uut->vertices()[1] == po0);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
}


BOOST_FIXTURE_TEST_CASE( contact_graph_5_stacked_ctor_test, contact_graph_5_stacked_fixture )
{
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK(uut->number_of_edges() == 4);
    BOOST_CHECK(uut->vertices()[0] == po0);
    BOOST_CHECK(uut->vertices()[1] == po1);
    BOOST_CHECK(uut->vertices()[2] == po2);
    BOOST_CHECK(uut->vertices()[3] == po3);
    BOOST_CHECK(uut->vertices()[4] == po4);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
    BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
    BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK(uut->number_of_edges() == 4);
    BOOST_CHECK(uut->vertices()[0] == po1);
    BOOST_CHECK(uut->vertices()[1] == po0);
    BOOST_CHECK(uut->vertices()[2] == po2);
    BOOST_CHECK(uut->vertices()[3] == po3);
    BOOST_CHECK(uut->vertices()[4] == po4);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(0)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
    BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
    BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());
}


BOOST_FIXTURE_TEST_CASE( contact_graph_4_circle_ctor_test, contact_graph_4_circle_fixture )
{
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 4);
    BOOST_CHECK(uut->number_of_edges() == 4);
    BOOST_CHECK(uut->vertices()[0] == po0);
    BOOST_CHECK(uut->vertices()[1] == po1);
    BOOST_CHECK(uut->vertices()[2] == po2);
    BOOST_CHECK(uut->vertices()[3] == po6);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(3)[1].to() == 0);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[1].edge_id());
    BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 4);
    BOOST_CHECK(uut->number_of_edges() == 4);
    BOOST_CHECK(uut->vertices()[0] == po1);
    BOOST_CHECK(uut->vertices()[1] == po0);
    BOOST_CHECK(uut->vertices()[2] == po6);
    BOOST_CHECK(uut->vertices()[3] == po2);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(3)[1].to() == 0);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[1].edge_id());
    BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
}


BOOST_FIXTURE_TEST_CASE( contact_graph_5_p_ctor_test, contact_graph_5_p_fixture )
{
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK(uut->number_of_edges() == 5);
    BOOST_CHECK(uut->vertices()[0] == po0);
    BOOST_CHECK(uut->vertices()[1] == po1);
    BOOST_CHECK(uut->vertices()[2] == po2);
    BOOST_CHECK(uut->vertices()[3] == po4);
    BOOST_CHECK(uut->vertices()[4] == po6);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(0)[1].to() == 4);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(2)[2].to() == 4);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(4)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(4)[1].to() == 0);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(4)[1].edge_id());
    BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
    BOOST_CHECK(uut->adjacent(2)[2].edge_id() == uut->adjacent(4)[0].edge_id());

    /* Rebuild, but from po1 */
    uut->rebuild(info, po4);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK(uut->number_of_edges() == 5);
    BOOST_CHECK(uut->vertices()[0] == po4);
    BOOST_CHECK(uut->vertices()[1] == po2);
    BOOST_CHECK(uut->vertices()[2] == po1);
    BOOST_CHECK(uut->vertices()[3] == po0);
    BOOST_CHECK(uut->vertices()[4] == po6);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(1)[2].to() == 4);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
    BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
    BOOST_CHECK(uut->adjacent(4)[1].to() == 1);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(1)[2].edge_id() == uut->adjacent(4)[1].edge_id());
    BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
    BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());
}


BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_ctor_test, contact_graph_9_disjoint_fixture )
{
    /* Construct */
    uut->rebuild(info, po2);

    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK(uut->number_of_edges() == 4);
    BOOST_CHECK(uut->vertices()[0] == po2);
    BOOST_CHECK(uut->vertices()[1] == po1);
    BOOST_CHECK(uut->vertices()[2] == po0);
    BOOST_CHECK(uut->vertices()[3] == po3);
    BOOST_CHECK(uut->vertices()[4] == po4);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
    BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[0].edge_id());
    BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());

    /* Rebuild, but from po1 */
    uut->rebuild(info, po6);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 4);
    BOOST_CHECK(uut->number_of_edges() == 4);
    BOOST_CHECK(uut->vertices()[0] == po6);
    BOOST_CHECK(uut->vertices()[1] == po5);
    BOOST_CHECK(uut->vertices()[2] == po8);
    BOOST_CHECK(uut->vertices()[3] == po7);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
    BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
    BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
    BOOST_CHECK(uut->adjacent(3)[1].to() == 0);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
    BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[1].edge_id());
    BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
    BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
}


BOOST_FIXTURE_TEST_CASE ( contact_graph_void_collisions_test, contact_graph_9_disjoint_fixture )
{
    /* Void */
    uut->void_collisions(&info);

    /* Check the stack is void */
    BOOST_CHECK(info[po0]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po0]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po0]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po1]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po1]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po1]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po2]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po2]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po2]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po3]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po3]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po3]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po4]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po4]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po4]->get_first_collision() == nullptr);

    /* Check the circle isnt void */
    BOOST_CHECK(info[po5]->get_first_collision_time() == 0.0);
    BOOST_CHECK(info[po5]->get_first_collision_type() == SLIDING_COLLISION);
    BOOST_CHECK(info[po5]->get_first_collision() == po6);

    BOOST_CHECK(info[po6]->get_first_collision_time() == 0.0);
    BOOST_CHECK(info[po6]->get_first_collision_type() == SLIDING_COLLISION);
    BOOST_CHECK(info[po6]->get_first_collision() == po7);

    BOOST_CHECK(info[po7]->get_first_collision_time() == 0.0);
    BOOST_CHECK(info[po7]->get_first_collision_type() == SLIDING_COLLISION);
    BOOST_CHECK(info[po7]->get_first_collision() == po8);

    BOOST_CHECK(info[po8]->get_first_collision_time() == 0.0);
    BOOST_CHECK(info[po8]->get_first_collision_type() == SLIDING_COLLISION);
    BOOST_CHECK(info[po8]->get_first_collision() == po5);

    /* Rebuild for the circle */
    uut->rebuild(info, po7);
    uut->void_collisions(&info);

    /* Check the stack is void */
    BOOST_CHECK(info[po0]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po0]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po0]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po1]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po1]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po1]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po2]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po2]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po2]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po3]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po3]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po3]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po4]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po4]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po4]->get_first_collision() == nullptr);

    /* Check the circle is void */
    BOOST_CHECK(info[po5]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po5]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po5]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po6]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po6]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po6]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po7]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po7]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po7]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po8]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po8]->get_first_collision_type() == NO_COLLISION);
    BOOST_CHECK(info[po8]->get_first_collision() == nullptr);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0, 0.0, -1.0));
    po1->set_force(point_t(0.0, 0.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po0->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 0.0, 0.0));

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po0->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 0.0, 0.0));

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0, 0.0, -4.0));
    po1->set_force(point_t(0.0, 0.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, -2.0));
    BOOST_CHECK(po0->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 0.0, 0.0));
    
    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, -2.0));
    BOOST_CHECK(po0->get_torque() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 0.0, 0.0));
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0, 0.0, -1.0));
    po0->set_torque(point_t(-1.0, 0.0, -1.0));
    po1->set_force(point_t(0.0, 0.0,  1.0));
    po1->set_torque(point_t(0.0, 2.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po0->get_torque() == point_t(-1.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 2.0, 1.0));

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po0->get_torque() == point_t(-1.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 2.0, 1.0));

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0, 0.0, -4.0));
    po0->set_torque(point_t(-3.0, 0.0, -1.0));
    po1->set_force(point_t(0.0, 0.0,  1.0));
    po1->set_torque(point_t(0.0, 1.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, -2.0));
    BOOST_CHECK(po0->get_torque() == point_t(-3.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 1.0, 1.0));

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(po0->get_force()  == point_t(0.0, 0.0, -2.0));
    BOOST_CHECK(po0->get_torque() == point_t(-3.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_force()  == point_t(0.0, 0.0, -1.0));
    BOOST_CHECK(po1->get_torque() == point_t(0.0, 1.0, 1.0));
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_non_perp_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 2.0, -1.0));
    po10->set_torque(point_t(-3.0, 0.0, 0.5));
    po11->set_force(point_t(3.0, 0.0,  1.0));
    po11->set_torque(point_t(0.0, 0.7, 2.0));
    uut->resolve_forces(1.0);

    /* Shows only forces and torques that cause the relative acceleration to increase matter */
    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t( 0.0,  2.0, -0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(-3.0, -0.4,  0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t( 3.0,  0.0,  0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t( 0.4,  0.7,  2.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t( 0.0,  2.0, -0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(-3.0, -0.4,  0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t( 3.0,  0.0,  0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t( 0.4,  0.7,  2.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -1.0));
    po11->set_force(point_t(0.0, 0.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,  0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,  0.0))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -4.0));
    po10->set_torque(point_t(0.0, 0.0, 0.0));
    po11->set_force(point_t(0.0, 0.0,  1.0));
    po11->set_torque(point_t(0.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -3.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.8,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  0.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0,  0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -3.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.8,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  0.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0,  0.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -1.0));
    po10->set_torque(point_t(0.0, 3.0, 0.0));
    po11->set_force(point_t(0.0, 0.0,  1.0));
    po11->set_torque(point_t(1.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0, 0.0, -0.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, 2.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0, 0.0,  0.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0,  0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0, 0.0, -0.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, 2.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0, 0.0,  0.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0,  0.0))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -4.0));
    po10->set_torque(point_t(0.0, 1.0, 0.0));
    po11->set_force(point_t(0.0, 0.0,  1.0));
    po11->set_torque(point_t(-1.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -2.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0, -0.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,  0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -2.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0, -0.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,  0.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0, 0.0, -1.0));
    po1->set_force(point_t(0.0, 0.0, -1.0));
    po2->set_force(point_t(0.0, 0.0,  0.0));
    po3->set_force(point_t(0.0, 0.0,  1.0));
    po4->set_force(point_t(0.0, 0.0,  1.0));

    po5->set_force( point_t(0.0, 0.0, 1.1));
    po6->set_force( point_t(0.0, 0.0, 1.2));
    po7->set_force( point_t(0.0, 0.0, 1.3));
    po8->set_force( point_t(0.0, 0.0, 1.4));
    po5->set_torque(point_t(0.0, 1.1, 0.0));
    po6->set_torque(point_t(0.0, 1.2, 0.0));
    po7->set_torque(point_t(0.0, 1.3, 0.0));
    po8->set_torque(point_t(0.0, 1.4, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0, 0.0, -4.5));
    po6->set_force( point_t(0.0, 0.0,  1.0));
    po7->set_force( point_t(0.0, 0.0,  1.0));
    po8->set_force( point_t(0.0, 0.0,  1.0));
    po5->set_torque(point_t(0.0, 0.0,  0.0));
    po6->set_torque(point_t(0.0, 0.0,  0.0));
    po7->set_torque(point_t(0.0, 0.0,  0.0));
    po8->set_torque(point_t(0.0, 0.0,  0.0));

    po0->set_force( point_t(0.0, 0.0, 1.0));
    po1->set_force( point_t(0.0, 0.0, 1.1));
    po2->set_force( point_t(0.0, 0.0, 1.2));
    po3->set_force( point_t(0.0, 0.0, 1.3));
    po4->set_force( point_t(0.0, 0.0, 1.4));
    po0->set_torque(point_t(0.0, 1.0, 0.0));
    po1->set_torque(point_t(0.0, 1.1, 0.0));
    po2->set_torque(point_t(0.0, 1.2, 0.0));
    po3->set_torque(point_t(0.0, 1.3, 0.0));
    po4->set_torque(point_t(0.0, 1.4, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_rotational_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Construct */
    uut->rebuild(info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0,  0.0, -4.5));
    po6->set_force( point_t(0.0,  0.0,  1.0));
    po7->set_force( point_t(0.0,  0.0,  1.0));
    po8->set_force( point_t(0.0,  0.0,  1.0));
    po5->set_torque(point_t(0.0, -1.0,  0.0));
    po6->set_torque(point_t(0.0,  0.0,  0.0));
    po7->set_torque(point_t(0.0,  1.0,  0.0));
    po8->set_torque(point_t(0.0,  0.0,  0.0));

    po0->set_force( point_t(0.0, 0.0, 1.0));
    po1->set_force( point_t(0.0, 0.0, 1.1));
    po2->set_force( point_t(0.0, 0.0, 1.2));
    po3->set_force( point_t(0.0, 0.0, 1.3));
    po4->set_force( point_t(0.0, 0.0, 1.4));
    po0->set_torque(point_t(0.0, 1.0, 0.0));
    po1->set_torque(point_t(0.0, 1.1, 0.0));
    po2->set_torque(point_t(0.0, 1.2, 0.0));
    po3->set_torque(point_t(0.0, 1.3, 0.0));
    po4->set_torque(point_t(0.0, 1.4, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0, -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0, -0.5)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 0.0,  0.0)))    < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0,  0.0, -4.5));
    po6->set_force( point_t(0.0,  0.0,  1.0));
    po7->set_force( point_t(0.0,  0.0,  1.0));
    po8->set_force( point_t(0.0,  0.0,  1.0));
    po5->set_torque(point_t(0.0,  2.0,  0.0));
    po6->set_torque(point_t(0.0,  0.0,  0.0));
    po7->set_torque(point_t(0.0, -1.0,  0.0));
    po8->set_torque(point_t(0.0,  0.0,  0.0));

    po0->set_force( point_t(0.0, 0.0, 1.0));
    po1->set_force( point_t(0.0, 0.0, 1.1));
    po2->set_force( point_t(0.0, 0.0, 1.2));
    po3->set_force( point_t(0.0, 0.0, 1.3));
    po4->set_force( point_t(0.0, 0.0, 1.4));
    po0->set_torque(point_t(0.0, 1.0, 0.0));
    po1->set_torque(point_t(0.0, 1.1, 0.0));
    po2->set_torque(point_t(0.0, 1.2, 0.0));
    po3->set_torque(point_t(0.0, 1.3, 0.0));
    po4->set_torque(point_t(0.0, 1.4, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0,   -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0,   -0.875)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0,   -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0,   -0.125)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 0.125,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 0.0,    0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 0.125,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 0.0,    0.0)))    < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_force()  - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_force()  - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_force()  - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_torque() - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_torque() - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_torque() - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_force()  - point_t(0.0, 0.0,   -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_force()  - point_t(0.0, 0.0,   -0.875)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_force()  - point_t(0.0, 0.0,   -0.25)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_force()  - point_t(0.0, 0.0,   -0.125)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_torque() - point_t(0.0, 0.125,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_torque() - point_t(0.0, 0.0,    0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_torque() - point_t(0.0, 0.125,  0.0)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_torque() - point_t(0.0, 0.0,    0.0)))    < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_resting_impulse_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0, 0.0, -1.0));
    po1->set_velocity(point_t(0.0, 0.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t(0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(0.0, 0.0,  0.0)))      < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t(0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t(0.0, 0.0,  0.0)))      < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t(0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(0.0, 0.0,  0.0)))      < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t(0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t(0.0, 0.0,  0.0)))      < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);

    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0, 0.0, -4.0));
    po1->set_velocity(point_t(0.0, 0.0,  2.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t(0.0, 0.0, -2.0)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(0.0, 0.0, 0.0)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t(0.0, 0.0, -2.0)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t(0.0, 0.0, 0.0)))   < result_tolerance);
    
    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t(0.0, 0.0, -2.0)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(0.0, 0.0, 0.0)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t(0.0, 0.0, -2.0)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t(0.0, 0.0, 0.0)))   < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_resting_implulse_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0, 0.0, -1.0));
    po0->set_angular_velocity(point_t(-1.0, 0.0, -1.0));
    po1->set_velocity(point_t(0.0, 0.0,  1.0));
    po1->set_angular_velocity(point_t(0.0, 2.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t( 0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(-1.0, 0.0, -1.0)))      < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t( 0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0, 2.0,  1.0)))      < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t( 0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(-1.0, 0.0, -1.0)))      < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t( 0.0, 0.0, -0.333333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0, 2.0,  1.0)))      < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0, 0.0, -4.0));
    po0->set_angular_velocity(point_t(-3.0, 0.0, -1.0));
    po1->set_velocity(point_t(0.0, 0.0,  2.0));
    po1->set_angular_velocity(point_t(0.0, 1.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t( 0.0, 0.0, -2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(-3.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t( 0.0, 0.0, -2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0, 1.0,  1.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()         - point_t( 0.0, 0.0, -2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity() - point_t(-3.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()         - point_t( 0.0, 0.0, -2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0, 1.0,  1.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_non_perp_resting_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0, 2.0, -1.0));
    po10->set_angular_velocity(point_t(-3.0, 0.0, 0.5));
    po11->set_velocity(point_t(3.0, 0.0,  1.0));
    po11->set_angular_velocity(point_t(0.0, 0.7, 2.0));
    uut->resolve_forces(1.0);

    /* Shows only forces and torques that cause the relative acceleration to increase matter */
    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t( 0.0,  2.0, -0.733333)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t(-3.0, -0.4,  0.5)))       < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t( 3.0,  0.0,  0.466666)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.7,  2.0)))       < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t( 0.0,  2.0, -0.733333)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t(-3.0, -0.4,  0.5)))       < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t( 3.0,  0.0,  0.466666)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.7,  2.0)))       < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_resting_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0, 0.0, -1.0));
    po11->set_velocity(point_t(0.0, 0.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t( 0.0,  0.0, -0.733333)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0, -0.4,  0.0)))       < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t( 0.0,  0.0,  0.466666)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.0,  0.0)))       < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t( 0.0,  0.0, -0.733333)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0, -0.4,  0.0)))       < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t( 0.0,  0.0,  0.466666)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.0,  0.0)))       < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(info, po11);

    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0, 0.0, -4.0));
    po10->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po11->set_velocity(point_t(0.0, 0.0,  1.0));
    po11->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t(0.0,  0.0, -3.33333)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t(0.0, -1.0,  0.0)))        < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t(0.0,  0.0, -0.33333)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t(2.0,  0.0,  0.0)))        < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t(0.0,  0.0, -3.33333)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t(0.0, -1.0,  0.0)))        < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t(0.0,  0.0, -0.33333)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t(2.0,  0.0,  0.0)))        < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_resting_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0, 0.0, -1.0));
    po10->set_angular_velocity(point_t(0.0, 3.0, 0.0));
    po11->set_velocity(point_t(0.0, 0.0,  1.0));
    po11->set_angular_velocity(point_t(1.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t( 0.0,  0.0, -0.4666666))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0,  2.2,  0.0)))       < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t( 0.0,  0.0, -0.06666)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t( 2.6,  0.0,  0.0)))       < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t( 0.0,  0.0, -0.4666666))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0,  2.2,  0.0)))       < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t( 0.0,  0.0, -0.06666)))   < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t( 2.6,  0.0,  0.0)))       < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(info, po11);

    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0, 0.0, -4.0));
    po10->set_angular_velocity(point_t(0.0, 1.0, 0.0));
    po11->set_velocity(point_t(0.0, 0.0,  1.0));
    po11->set_angular_velocity(point_t(-1.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t(0.0,  0.0, -3.06666))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t(0.0, -0.4,  0.0)))     < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t(0.0,  0.0, -0.86666))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t(1.8,  0.0,  0.0)))     < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_velocity()         - point_t(0.0,  0.0, -3.06666))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_angular_velocity() - point_t(0.0, -0.4,  0.0)))     < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_velocity()         - point_t(0.0,  0.0, -0.86666))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_angular_velocity() - point_t(1.8,  0.0,  0.0)))     < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_resting_impulse_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0, 0.0, -0.5));
    po1->set_velocity(point_t(0.0, 0.0, -1.0));
    po2->set_velocity(point_t(0.0, 0.0,  0.0));
    po3->set_velocity(point_t(0.0, 0.0,  1.0));
    po4->set_velocity(point_t(0.0, 0.0,  1.0));

    po5->set_velocity(point_t(0.0, 0.0, 1.1));
    po6->set_velocity(point_t(0.0, 0.0, 1.2));
    po7->set_velocity(point_t(0.0, 0.0, 1.3));
    po8->set_velocity(point_t(0.0, 0.0, 1.4));
    po5->set_angular_velocity(point_t(0.0, 1.1, 0.0));
    po6->set_angular_velocity(point_t(0.0, 1.2, 0.0));
    po7->set_angular_velocity(point_t(0.0, 1.3, 0.0));
    po8->set_angular_velocity(point_t(0.0, 1.4, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_velocity()          - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_velocity()          - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_velocity()          - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_velocity()          - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_velocity()          - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_velocity()          - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_velocity()          - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_velocity()          - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_velocity()          - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0, 0.0,  2.0));
    po1->set_velocity(point_t(0.0, 0.0,  1.0));
    po2->set_velocity(point_t(0.0, 0.0,  0.0));
    po3->set_velocity(point_t(0.0, 0.0, -1.0));
    po4->set_velocity(point_t(0.0, 0.0, -1.0));

    po5->set_velocity(point_t(0.0, 0.0, 1.1));
    po6->set_velocity(point_t(0.0, 0.0, 1.2));
    po7->set_velocity(point_t(0.0, 0.0, 1.3));
    po8->set_velocity(point_t(0.0, 0.0, 1.4));
    po5->set_angular_velocity(point_t(0.0, 1.1, 0.0));
    po6->set_angular_velocity(point_t(0.0, 1.2, 0.0));
    po7->set_angular_velocity(point_t(0.0, 1.3, 0.0));
    po8->set_angular_velocity(point_t(0.0, 1.4, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()          - point_t(0.0, 0.0,  2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()          - point_t(0.0, 0.0,  1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_velocity()          - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_velocity()          - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_velocity()          - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_velocity()          - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_velocity()          - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_velocity()          - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_velocity()          - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()          - point_t(0.0, 0.0,  2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()          - point_t(0.0, 0.0,  1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_velocity()          - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_velocity()          - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_velocity()          - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_velocity()          - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_velocity()          - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_velocity()          - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_velocity()          - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0, 1.4, 0.0))) < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(info, po6);

    /* Resolve forces and check vgs */
    po5->set_velocity(point_t(0.0, 0.0, -4.5));
    po6->set_velocity(point_t(0.0, 0.0,  1.0));
    po7->set_velocity(point_t(0.0, 0.0,  2.0));
    po8->set_velocity(point_t(0.0, 0.0,  1.0));
    po5->set_angular_velocity(point_t(0.0, 0.0,  0.0));
    po6->set_angular_velocity(point_t(0.0, 0.0,  0.0));
    po7->set_angular_velocity(point_t(0.0, 0.0,  0.0));
    po8->set_angular_velocity(point_t(0.0, 0.0,  0.0));

    po0->set_velocity(point_t(0.0, 0.0, 1.0));
    po1->set_velocity(point_t(0.0, 0.0, 1.1));
    po2->set_velocity(point_t(0.0, 0.0, 1.2));
    po3->set_velocity(point_t(0.0, 0.0, 1.3));
    po4->set_velocity(point_t(0.0, 0.0, 1.4));
    po0->set_angular_velocity(point_t(0.0, 1.0, 0.0));
    po1->set_angular_velocity(point_t(0.0, 1.1, 0.0));
    po2->set_angular_velocity(point_t(0.0, 1.2, 0.0));
    po3->set_angular_velocity(point_t(0.0, 1.3, 0.0));
    po4->set_angular_velocity(point_t(0.0, 1.4, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()          - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()          - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_velocity()          - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_velocity()          - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_velocity()          - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_velocity()          - point_t(0.0, 0.0, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_velocity()          - point_t(0.0, 0.0, 1.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_velocity()          - point_t(0.0, 0.0, 1.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_velocity()          - point_t(0.0, 0.0, 1.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_velocity()          - point_t(0.0, 0.0, 1.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0, 1.1, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0, 1.2, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0, 1.3, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0, 1.4, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(po5->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_velocity()          - point_t(0.0, 0.0, 0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0, 0.0,  0.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_resting_force_test, contact_graph_2_infinite_mass_contact_fixture )
{
    /* Resolve forces and check vgs */
    po12->set_force(point_t(0.0,   0.0, -1000.0));
    po13->set_force(point_t(0.0,   0.0,     0.0));
    po12->set_torque(point_t(0.0, 250.0,    0.0));
    po13->set_torque(point_t(0.0,   0.0,    0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po12->get_force()  - point_t(0.0,    0.0, -450.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po12->get_torque() - point_t(0.0, -300.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_force()  - point_t(0.0,    0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_torque() - point_t(0.0,    0.0,    0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po12->get_force()  - point_t(0.0,    0.0, -450.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po12->get_torque() - point_t(0.0, -300.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_force()  - point_t(0.0,    0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_torque() - point_t(0.0,    0.0,    0.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_resting_impulse_test, contact_graph_2_infinite_mass_contact_fixture )
{   
    po12->set_velocity(point_t(0.0, 0.0, -4.0));
    po13->set_velocity(point_t(0.0, 0.0,  0.0));
    po12->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po13->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po12->get_velocity()         - point_t(0.0,  0.0, -2.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po12->get_angular_velocity() - point_t(0.0, -2.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_velocity()         - point_t(0.0,  0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_angular_velocity() - point_t(0.0,  0.0,  0.0))) < result_tolerance);
 
    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po12->get_velocity()         - point_t(0.0,  0.0, -2.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po12->get_angular_velocity() - point_t(0.0, -2.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_velocity()         - point_t(0.0,  0.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_angular_velocity() - point_t(0.0,  0.0,  0.0))) < result_tolerance);

    /* Rebuild, but from po12 */
    uut->rebuild(info, po12);
    
    po12->set_velocity(point_t(0.0, 0.0, -4.0));
    po13->set_velocity(point_t(0.0, 0.0,  0.0));
    po12->set_angular_velocity(point_t(0.0, 6.0, 0.0));
    po13->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po12->get_velocity()         - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po12->get_angular_velocity() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_velocity()         - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_angular_velocity() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
 
    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po12->get_velocity()         - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po12->get_angular_velocity() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_velocity()         - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po13->get_angular_velocity() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_resting_force_and_impulse_test, contact_graph_2_contact_fixture )
{
    /* No change for joint movement and movemnt in normal */
    po0->set_force(point_t(0.0, 0.0, -1.0));
    po1->set_force(point_t(0.0, 0.0,  1.0));
    po0->set_velocity(point_t(-1.0, 1.0, -1.0));
    po1->set_velocity(point_t(-1.0, 1.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    
    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0, 0.0, -160.0));
    po1->set_force(point_t(0.0, 0.0,   80.0));
    po0->set_velocity(point_t(0.0,  1.0, 0.0));
    po1->set_velocity(point_t(0.0, -1.0, 0.0));
    data0->set_velocity(point_t(0.0,  1.0, 0.0));
    data1->set_velocity(point_t(0.0, -1.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, -160.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0,   80.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, -160.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0,   80.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);

    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0, 0.0, -164.0));
    po1->set_force(point_t(0.0, 0.0,   81.0));
    po0->set_velocity(point_t(0.0,  1.0, 0.0));
    po1->set_velocity(point_t(0.0, -1.0, 0.0));
    data0->set_velocity(point_t(0.0,  1.0, 0.0));
    data1->set_velocity(point_t(0.0, -1.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, -162.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0,   79.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po0->get_force()  - point_t(0.0, 0.0, -162.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po0->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_force()  - point_t(0.0, 0.0,   79.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po1->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_resting_force_and_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
    po10->set_force(point_t(0.0, 0.0, -1.0));
    po11->set_force(point_t(0.0, 0.0,  1.0));
    po10->set_velocity(point_t(-1.0, 1.0,  1.0));
    po11->set_velocity(point_t(-1.0, 1.0, -1.0));
    data12->set_velocity(point_t(-1.0, 1.0,  1.0));
    data13->set_velocity(point_t(-1.0, 1.0, -1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,  0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  0.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,  0.0))) < result_tolerance);

    /* Rebuild, but from po10 */
    uut->rebuild(info, po10);

    po10->set_force(point_t(0.0, 0.0, -1.0));
    po11->set_force(point_t(0.0, 0.0,  1.0));
    po10->set_torque(point_t(0.0, 0.0, 0.0));
    po11->set_torque(point_t(0.0, 0.0, 0.0));
    po10->set_velocity(point_t(-1.0, 1.0, -1.0));
    po11->set_velocity(point_t(-1.0, 1.0,  1.0));
    data12->set_velocity(point_t(-1.0, 1.0, -1.0));
    data13->set_velocity(point_t(-1.0, 1.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,       0.0,      -0.601897))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0,      -0.398103,  0.0     ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,       0.0,       0.601897))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.398103,  0.0,       0.0     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,       0.0,      -0.601897))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0,      -0.398103,  0.0     ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,       0.0,       0.601897))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.398103,  0.0,       0.0     ))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -160.0));
    po11->set_force(point_t(0.0, 0.0,   80.0));
    po10->set_torque(point_t(0.0, 0.0, 0.0));
    po11->set_torque(point_t(0.0, 0.0, 0.0));
    po10->set_velocity(point_t(0.0,  1.0, 0.0));
    po11->set_velocity(point_t(0.0, -1.0, 0.0));
    po10->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po11->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    data12->set_velocity(point_t(0.0,  1.0, 0.0));
    data13->set_velocity(point_t(0.0, -1.0, 0.0));
    data12->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    data13->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0, 0.0, -160.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0, 0.0,   80.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0, 0.0, -160.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0, 0.0,   80.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.0, 0.0,    0.0))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -164.0));
    po10->set_torque(point_t(0.0, 0.0, 0.0));
    po11->set_force(point_t(0.0, 0.0,   81.0));
    po11->set_torque(point_t(0.0, 0.0, 0.0));
    po10->set_velocity(point_t(0.0,  1.0, 0.0));
    po11->set_velocity(point_t(0.0, -1.0, 0.0));
    data12->set_velocity(point_t(0.0,  1.0, 0.0));
    data13->set_velocity(point_t(0.0, -1.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -163.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.8,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,   80.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0,    0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -163.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.8,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,   80.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0,    0.0))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_resting_force_and_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -161.0));
    po10->set_torque(point_t(0.0, 3.0, 0.0));
    po11->set_force(point_t(0.0, 0.0,  81.0));
    po11->set_torque(point_t(1.0, 0.0, 0.0));
    po10->set_velocity(point_t(-1.0, 0.0, 0.0));
    po11->set_velocity(point_t( 1.0, 0.0, 0.0));
    data12->set_velocity(point_t(-1.0, 0.0, 0.0));
    data13->set_velocity(point_t( 1.0, 0.0, 0.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0, 0.0, -160.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, 2.4,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0, 0.0,   80.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0,    0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0, 0.0, -160.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, 2.4,    0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0, 0.0,   80.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0,    0.0))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0, 0.0, -84.0));
    po10->set_torque(point_t(0.0, 1.0, 0.0));
    po11->set_force(point_t(0.0, 0.0,  41.0));
    po11->set_torque(point_t(-1.0, 0.0, 0.0));
    po10->set_velocity(point_t(0.0, 0.0, 0.0));
    po11->set_velocity(point_t(0.0, 0.0, 0.0));
    po10->set_angular_velocity(point_t(0.0, 0.0, -1.0));
    po11->set_angular_velocity(point_t(0.0, 0.0,  1.0));
    data12->set_velocity(point_t(0.0, 0.0, 0.0));
    data13->set_velocity(point_t(0.0, 0.0, 0.0));
    data12->set_angular_velocity(point_t(0.0, 0.0, -1.0));
    data13->set_angular_velocity(point_t(0.0, 0.0,  1.0));
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -82.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,   0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  39.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,   0.0))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0);

    BOOST_CHECK(fabs(magnitude(po10->get_force()  - point_t(0.0,  0.0, -82.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po10->get_torque() - point_t(0.0, -0.4,   0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_force()  - point_t(0.0,  0.0,  39.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po11->get_torque() - point_t(0.4,  0.0,   0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
