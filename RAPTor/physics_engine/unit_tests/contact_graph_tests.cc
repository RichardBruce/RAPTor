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
#include "contact_graph.h"
#include "rigid_body_collider.h"

/* Test headers */
#include "mock_physics_object.h"
#include "mock_simplex.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct contact_graph_base_fixture : private boost::noncopyable
{
    contact_graph_base_fixture() :
        vg_memory{},
        po0( new (&vg_memory[alignof(mock_physics_object)                                     ]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667f, 2.66667f, 2.66667f, 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 0.5f,  2.45f ), 4.0f), 0)),
        po1( new (&vg_memory[alignof(mock_physics_object) + ( 1 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 0.5f,  0.45f ), 2.0f), 1)),
        po2( new (&vg_memory[alignof(mock_physics_object) + ( 2 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 0.5f, -2.45f ), 2.0f), 1)),
        po3( new (&vg_memory[alignof(mock_physics_object) + ( 3 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 0.5f, -4.45f ), 2.0f), 0)),
        po4( new (&vg_memory[alignof(mock_physics_object) + ( 4 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 0.5f, -6.45f ), 2.0f), 2)),
        po5( new (&vg_memory[alignof(mock_physics_object) + ( 5 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 1.5f, 0.5f,  2.45f ), 2.0f), 2)),
        po6( new (&vg_memory[alignof(mock_physics_object) + ( 6 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667f, 2.66667f, 2.66667f, 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 0.5f,  0.45f ), 4.0f), 0)),
        po7( new (&vg_memory[alignof(mock_physics_object) + ( 7 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 1.5f, 0.5f, -2.45f ), 2.0f), 0)),
        po8( new (&vg_memory[alignof(mock_physics_object) + ( 8 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667f, 2.66667f, 2.66667f, 0.0f,  0.0f, 0.0f }, point_t( 0.5f, 0.5f,  0.45f ), 4.0f), 1)),
        po9( new (&vg_memory[alignof(mock_physics_object) + ( 9 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667f, 2.66667f, 2.66667f, 0.0f,  0.0f, 0.0f }, point_t( 1.0f, 0.0f, -2.3f  ), 4.0f))),
        po10(new (&vg_memory[alignof(mock_physics_object) + (10 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667f, 2.66667f, 2.66667f, 0.0f,  0.0f, 0.0f }, point_t( 1.5f, 0.5f,  1.45000001f), 4.0f), 0)),
        po11(new (&vg_memory[alignof(mock_physics_object) + (11 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 1.5f,  1.44444449f), 2.0f), 1)),
        po12(new (&vg_memory[alignof(mock_physics_object) + (12 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667f, 2.66667f, 2.66667f, 0.0f,  0.0f, 0.0f }, point_t( 1.5f, 0.5f,  1.45000001f), 4.0f))),
        po13(new (&vg_memory[alignof(mock_physics_object) + (13 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 0.0f,  0.0f, 0.0f }, point_t( 2.5f, 1.5f,  1.44444449f), std::numeric_limits<float>::infinity()))),
        po14(new (&vg_memory[alignof(mock_physics_object) + (14 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 1.33333f, 1.33333f, 1.33333f, 0.0f,  0.0f, 0.0f }, point_t( 0.0f, 0.0f, 1.5f), 2.0f), 0)),
        po15(new (&vg_memory[alignof(mock_physics_object) + (15 * sizeof(mock_physics_object))]) mock_physics_object(new inertia_tensor(new float[6]{ 2.66667f, 2.66667f, 2.66667f, 0.0f,  0.0f, 0.0f }, point_t( 0.0f, 0.0f, 0.5f), 4.0f), 1)),
        s0( { point_t(2.5f, 0.5f,  1.5f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s1( { point_t(2.5f, 0.5f,  1.4f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s2( { point_t(2.5f, 0.5f, -1.4f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s3( { point_t(2.5f, 0.5f, -1.5f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s4( { point_t(2.5f, 0.5f, -3.4f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s5( { point_t(2.5f, 0.5f, -3.5f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s6( { point_t(2.5f, 0.5f, -5.4f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s7( { point_t(2.5f, 0.5f, -5.5f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s8( { point_t(0.5f, 0.5f, -1.5f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s9( { point_t(0.5f, 0.5f, -1.4f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s10({ point_t(0.5f, 0.5f,  1.4f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s11({ point_t(0.5f, 0.5f,  1.5f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s12({ point_t(2.5f, 0.5f,  1.45f) },  point_t(0.0f, 0.0f,  1.0f)),
        s13({ point_t(2.5f, 0.5f,  1.45f) },  point_t(0.0f, 0.0f, -1.0f)),
        s14({ point_t(2.5f, 0.5f,  1.5f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s15({ point_t(2.5f, 0.5f,  1.4f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s16({ point_t(2.0f, 0.0f,  1.45f),    point_t(3.0f, 0.0f,  1.45f),   point_t(3.0f, 1.0f, 1.45f),    point_t(2.0f, 1.0f, 1.45f) }, point_t(0.0f, 0.0f,  1.0f)),
        s17({ point_t(2.0f, 0.0f,  1.45f),    point_t(3.0f, 0.0f,  1.45f),   point_t(3.0f, 1.0f, 1.45f),    point_t(2.0f, 1.0f, 1.45f) }, point_t(0.0f, 0.0f, -1.0f)),
        s18({ point_t(2.0f, 0.0f,  1.45f),    point_t(5.0f, 0.0f,  1.45f),   point_t(5.0f, 4.0f, 1.45f),    point_t(2.0f, 4.0f, 1.45f) }, point_t(0.0f, 0.0f,  1.0f)),
        s19({ point_t(2.0f, 0.0f,  1.45f),    point_t(5.0f, 0.0f,  1.45f),   point_t(5.0f, 4.0f, 1.45f),    point_t(2.0f, 4.0f, 1.45f) }, point_t(0.0f, 0.0f, -1.0f)),
        s20({ point_t(0.5f, 0.0f,  1.0f ) },  point_t(0.0f, 0.0f,  1.0f)),
        s21({ point_t(0.5f, 0.0f,  1.0f ) },  point_t(0.0f, 0.0f, -1.0f)),
        s22({ point_t(2.0f, 0.0f, -1.45f),    point_t(3.0f, 0.0f, -1.45f),   point_t(3.0f, 1.0f, -1.45f),   point_t(2.0f, 1.0f, -1.45f) }, point_t(0.0f, 0.0f,  1.0f)),
        s23({ point_t(2.0f, 0.0f, -1.45f),    point_t(3.0f, 0.0f, -1.45f),   point_t(3.0f, 1.0f, -1.45f),   point_t(2.0f, 1.0f, -1.45f) }, point_t(0.0f, 0.0f, -1.0f)),
        s24({ point_t(2.0f, 0.0f, -3.45f),    point_t(3.0f, 1.0f, -3.45f) }, point_t(0.0f, 0.0f,  1.0f)),
        s25({ point_t(2.0f, 0.0f, -3.45f),    point_t(3.0f, 1.0f, -3.45f) }, point_t(0.0f, 0.0f, -1.0f)),
        s26({ point_t(2.5f, 0.5f, -5.45f) },  point_t(0.0f, 0.0f,  1.0f)),
        s27({ point_t(2.5f, 0.5f, -5.45f) },  point_t(0.0f, 0.0f, -1.0f)),
        s28({ point_t(0.0f, 0.0f, -1.45f ),   point_t(1.0f, 1.0f, -1.45f) }, point_t(0.0f, 0.0f, -1.0f)),
        s29({ point_t(0.0f, 0.0f, -1.45f ),   point_t(1.0f, 1.0f, -1.45f) }, point_t(0.0f, 0.0f,  1.0f)),
        s30({ point_t(0.5f, 0.5f,  1.45f ) }, point_t(0.0f, 0.0f, -1.0f)),
        s31({ point_t(0.5f, 0.5f,  1.45f ) }, point_t(0.0f, 0.0f,  1.0f)),
        no_friction{
            { 0, new std::map<unsigned int, const collider*>{
                { 0, new rigid_body_collider(0.0f, 0.0f )},
                { 1, new rigid_body_collider(0.0f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.0f )}},
            },
            { 1, new std::map<unsigned int, const collider*>{
                { 1, new rigid_body_collider(0.0f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.0f )}},
            },
            { 2, new std::map<unsigned int, const collider*>{
                { 2, new rigid_body_collider(0.0f, 0.0f )}},
            }
        },
        dynamic_friction{
            { 0, new std::map<unsigned int, const collider*>{
                { 0, new rigid_body_collider(0.0f, 0.1f )},
                { 1, new rigid_body_collider(0.0f, 0.5f )},
                { 2, new rigid_body_collider(0.0f, 0.9f )}},
            },
            { 1, new std::map<unsigned int, const collider*>{
                { 1, new rigid_body_collider(0.0f, 0.2f )},
                { 2, new rigid_body_collider(0.0f, 0.8f )}},
            },
            { 2, new std::map<unsigned int, const collider*>{
                { 2, new rigid_body_collider(0.0f, 0.7f )}},
            }
        },
        static_friction{
            { 0, new std::map<unsigned int, const collider*>{
                { 0, new rigid_body_collider(0.0f, 0.0f )},
                { 1, new rigid_body_collider(0.0f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.0f )}},
            },
            { 1, new std::map<unsigned int, const collider*>{
                { 1, new rigid_body_collider(0.0f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.0f )}},
            },
            { 2, new std::map<unsigned int, const collider*>{
                { 2, new rigid_body_collider(0.0f, 0.0f )}},
            }
        },
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
        po14->~mock_physics_object();
        po15->~mock_physics_object();

        /* Clean up pairwise colliders */
        for (auto& outer : no_friction)
        {
            for (auto& inner : *(outer.second))
            {
                delete inner.second;
            }
            delete outer.second;
        }

        for (auto& outer : dynamic_friction)
        {
            for (auto& inner : *(outer.second))
            {
                delete inner.second;
            }
            delete outer.second;
        }

        for (auto& outer : static_friction)
        {
            for (auto& inner : *(outer.second))
            {
                delete inner.second;
            }
            delete outer.second;
        }

        for (auto &p : info)
        {
            delete p.second;
        }

        delete uut;
    }

    /* Raw memory to hold the vg */
    /* It is important the addresses are fixed relative to one another or the graph vertices are in slightly different orders */
    char vg_memory[sizeof(mock_physics_object) * 17];

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
    mock_physics_object *const po14;
    mock_physics_object *const po15;
    mock_simplex s0;
    mock_simplex s1;
    mock_simplex s2;
    mock_simplex s3;
    mock_simplex s4;
    mock_simplex s5;
    mock_simplex s6;
    mock_simplex s7;
    mock_simplex s8;
    mock_simplex s9;
    mock_simplex s10;
    mock_simplex s11;
    mock_simplex s12;
    mock_simplex s13;
    mock_simplex s14;
    mock_simplex s15;
    mock_simplex s16;
    mock_simplex s17;
    mock_simplex s18;
    mock_simplex s19;
    mock_simplex s20;
    mock_simplex s21;
    mock_simplex s22;
    mock_simplex s23;
    mock_simplex s24;
    mock_simplex s25;
    mock_simplex s26;
    mock_simplex s27;
    mock_simplex s28;
    mock_simplex s29;
    mock_simplex s30;
    mock_simplex s31;

    std::map<unsigned int, std::map<unsigned int, const collider*>*> no_friction;
    std::map<unsigned int, std::map<unsigned int, const collider*>*> dynamic_friction;
    std::map<unsigned int, std::map<unsigned int, const collider*>*> static_friction;

    std::map<mock_physics_object*, tracking_info<mock_physics_object, mock_simplex>*> info;
    contact_graph<mock_physics_object, mock_simplex> *uut;
};


struct contact_graph_2_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_contact_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s0), s1, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s1), s0, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

struct contact_graph_2_offset_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_offset_contact_fixture()
    {
        info = {{po10, new tracking_info<mock_physics_object, mock_simplex>(po11, new mock_simplex(s12), s13, 0.0f, collision_t::SLIDING_COLLISION)},
                {po11, new tracking_info<mock_physics_object, mock_simplex>(po10, new mock_simplex(s13), s12, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po10);
    };
};

struct contact_graph_2_side_offset_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_side_offset_contact_fixture()
    {
        info = {{po14, new tracking_info<mock_physics_object, mock_simplex>(po15, new mock_simplex(s20), s21, 0.0f, collision_t::SLIDING_COLLISION)},
                {po15, new tracking_info<mock_physics_object, mock_simplex>(po14, new mock_simplex(s21), s20, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po14);
    };
};

struct contact_graph_2_contact_4_point_fixture : public contact_graph_base_fixture
{
    contact_graph_2_contact_4_point_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s16), s17, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s17), s16, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

struct contact_graph_2_contact_4_point_asym_fixture : public contact_graph_base_fixture
{
    contact_graph_2_contact_4_point_asym_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s18), s19, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s19), s18, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

struct contact_graph_2_infinite_mass_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_infinite_mass_contact_fixture()
    {
        info = {{po12, new tracking_info<mock_physics_object, mock_simplex>(po13, new mock_simplex(s14), s15, 0.0f, collision_t::SLIDING_COLLISION)},
                {po13, new tracking_info<mock_physics_object, mock_simplex>(po12, new mock_simplex(s15), s14, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po12);
    };
};

struct contact_graph_4_circle_fixture : public contact_graph_base_fixture
{
    contact_graph_4_circle_fixture()
    {
        info= {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex( s0),  s1, 0.0f, collision_t::SLIDING_COLLISION)},
               {po1, new tracking_info<mock_physics_object, mock_simplex>(po2, new mock_simplex( s2),  s3, 0.0f, collision_t::SLIDING_COLLISION)},
               {po2, new tracking_info<mock_physics_object, mock_simplex>(po6, new mock_simplex( s8),  s9, 0.0f, collision_t::SLIDING_COLLISION)},
               {po6, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s10), s11, 0.0f, collision_t::SLIDING_COLLISION)}};

        info[po1]->update(po0, new mock_simplex( s1),  s0, 0.0f, collision_t::SLIDING_COLLISION);
        info[po2]->update(po1, new mock_simplex( s3),  s2, 0.0f, collision_t::SLIDING_COLLISION);
        info[po6]->update(po2, new mock_simplex( s9),  s8, 0.0f, collision_t::SLIDING_COLLISION);
        info[po0]->update(po6, new mock_simplex(s11), s10, 0.0f, collision_t::SLIDING_COLLISION);

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

struct contact_graph_5_p_fixture : public contact_graph_base_fixture
{
    contact_graph_5_p_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex( s0),  s1, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po2, new mock_simplex( s2),  s3, 0.0f, collision_t::SLIDING_COLLISION)},
                {po2, new tracking_info<mock_physics_object, mock_simplex>(po6, new mock_simplex( s8),  s9, 0.0f, collision_t::SLIDING_COLLISION)},
                {po6, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s10), s11, 0.0f, collision_t::SLIDING_COLLISION)},
                {po4, new tracking_info<mock_physics_object, mock_simplex>(po2, new mock_simplex( s5),  s4, 0.0f, collision_t::SLIDING_COLLISION)}};

        info[po1]->update(po0, new mock_simplex( s1),  s0, 0.0f, collision_t::SLIDING_COLLISION);
        info[po2]->update(po1, new mock_simplex( s3),  s2, 0.0f, collision_t::SLIDING_COLLISION);
        info[po6]->update(po2, new mock_simplex( s9),  s8, 0.0f, collision_t::SLIDING_COLLISION);
        info[po0]->update(po6, new mock_simplex(s11), s10, 0.0f, collision_t::SLIDING_COLLISION);
        info[po2]->update(po4, new mock_simplex( s4),  s5, 0.0f, collision_t::SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

struct contact_graph_5_stacked_fixture : public contact_graph_base_fixture
{
    contact_graph_5_stacked_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s0), s1, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po2, new mock_simplex(s2), s3, 0.0f, collision_t::SLIDING_COLLISION)},
                {po2, new tracking_info<mock_physics_object, mock_simplex>(po3, new mock_simplex(s4), s5, 0.0f, collision_t::SLIDING_COLLISION)},
                {po3, new tracking_info<mock_physics_object, mock_simplex>(po4, new mock_simplex(s6), s7, 0.0f, collision_t::SLIDING_COLLISION)},
                {po4, new tracking_info<mock_physics_object, mock_simplex>(po3, new mock_simplex(s7), s6, 0.0f, collision_t::SLIDING_COLLISION)}};

        info[po1]->update(po0, new mock_simplex(s1), s0, 0.0f, collision_t::SLIDING_COLLISION);
        info[po2]->update(po1, new mock_simplex(s3), s2, 0.0f, collision_t::SLIDING_COLLISION);
        info[po3]->update(po2, new mock_simplex(s5), s4, 0.0f, collision_t::SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

struct contact_graph_9_disjoint_fixture : public contact_graph_base_fixture
{
    contact_graph_9_disjoint_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s0), s1, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po2, new mock_simplex(s2), s3, 0.0f, collision_t::SLIDING_COLLISION)},
                {po2, new tracking_info<mock_physics_object, mock_simplex>(po3, new mock_simplex(s4), s5, 0.0f, collision_t::SLIDING_COLLISION)},
                {po3, new tracking_info<mock_physics_object, mock_simplex>(po4, new mock_simplex(s6), s7, 0.0f, collision_t::SLIDING_COLLISION)},
                {po4, new tracking_info<mock_physics_object, mock_simplex>(po3, new mock_simplex(s7), s6, 0.0f, collision_t::SLIDING_COLLISION)},

                {po5, new tracking_info<mock_physics_object, mock_simplex>(po6, new mock_simplex( s0),  s1, 0.0f, collision_t::SLIDING_COLLISION)},
                {po6, new tracking_info<mock_physics_object, mock_simplex>(po7, new mock_simplex( s2),  s3, 0.0f, collision_t::SLIDING_COLLISION)},
                {po7, new tracking_info<mock_physics_object, mock_simplex>(po8, new mock_simplex( s8),  s9, 0.0f, collision_t::SLIDING_COLLISION)},
                {po8, new tracking_info<mock_physics_object, mock_simplex>(po5, new mock_simplex(s10), s11, 0.0f, collision_t::SLIDING_COLLISION)}};

        info[po1]->update(po0, new mock_simplex(s1), s0, 0.0f, collision_t::SLIDING_COLLISION);
        info[po2]->update(po1, new mock_simplex(s3), s2, 0.0f, collision_t::SLIDING_COLLISION);
        info[po3]->update(po2, new mock_simplex(s5), s4, 0.0f, collision_t::SLIDING_COLLISION);

        info[po6]->update(po5, new mock_simplex( s1),  s0, 0.0f, collision_t::SLIDING_COLLISION);
        info[po7]->update(po6, new mock_simplex( s3),  s2, 0.0f, collision_t::SLIDING_COLLISION);
        info[po8]->update(po7, new mock_simplex( s9),  s8, 0.0f, collision_t::SLIDING_COLLISION);
        info[po5]->update(po8, new mock_simplex(s11), s10, 0.0f, collision_t::SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

struct contact_graph_9_disjoint_multi_point_fixture : public contact_graph_base_fixture
{
    contact_graph_9_disjoint_multi_point_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s16), s17, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po2, new mock_simplex(s22), s23, 0.0f, collision_t::SLIDING_COLLISION)},
                {po2, new tracking_info<mock_physics_object, mock_simplex>(po3, new mock_simplex(s24), s25, 0.0f, collision_t::SLIDING_COLLISION)},
                {po3, new tracking_info<mock_physics_object, mock_simplex>(po4, new mock_simplex(s26), s27, 0.0f, collision_t::SLIDING_COLLISION)},
                {po4, new tracking_info<mock_physics_object, mock_simplex>(po3, new mock_simplex(s27), s26, 0.0f, collision_t::SLIDING_COLLISION)},

                {po5, new tracking_info<mock_physics_object, mock_simplex>(po6, new mock_simplex(s16), s17, 0.0f, collision_t::SLIDING_COLLISION)},
                {po6, new tracking_info<mock_physics_object, mock_simplex>(po7, new mock_simplex(s22), s23, 0.0f, collision_t::SLIDING_COLLISION)},
                {po7, new tracking_info<mock_physics_object, mock_simplex>(po8, new mock_simplex(s23), s22, 0.0f, collision_t::SLIDING_COLLISION)},
                {po8, new tracking_info<mock_physics_object, mock_simplex>(po5, new mock_simplex(s30), s31, 0.0f, collision_t::SLIDING_COLLISION)}};

        info[po1]->update(po0, new mock_simplex(s17), s16, 0.0f, collision_t::SLIDING_COLLISION);
        info[po2]->update(po1, new mock_simplex(s23), s22, 0.0f, collision_t::SLIDING_COLLISION);
        info[po3]->update(po2, new mock_simplex(s25), s24, 0.0f, collision_t::SLIDING_COLLISION);

        info[po6]->update(po5, new mock_simplex(s17), s16, 0.0f, collision_t::SLIDING_COLLISION);
        info[po7]->update(po6, new mock_simplex(s23), s22, 0.0f, collision_t::SLIDING_COLLISION);
        info[po8]->update(po7, new mock_simplex(s29), s28, 0.0f, collision_t::SLIDING_COLLISION);
        info[po5]->update(po8, new mock_simplex(s31), s30, 0.0f, collision_t::SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, info, po0);
    };
};

const float result_tolerance = 0.0005f;

BOOST_AUTO_TEST_SUITE( contact_graph_tests );


// BOOST_AUTO_TEST_CASE( default_ctor_test )
// {
//     /* Construct */
//     contact_graph<mock_physics_object, mock_simplex> uut;

//     /* Check graph */
//     BOOST_CHECK(uut.number_of_vertices() == 0);
//     BOOST_CHECK(uut.number_of_edges() == 0);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_ctor_test, contact_graph_2_contact_fixture )
// {
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 2);
//     BOOST_CHECK(uut->number_of_edges() == 1);
//     BOOST_CHECK(uut->vertices()[0] == po0);
//     BOOST_CHECK(uut->vertices()[1] == po1);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 2);
//     BOOST_CHECK(uut->number_of_edges() == 1);
//     BOOST_CHECK(uut->vertices()[0] == po1);
//     BOOST_CHECK(uut->vertices()[1] == po0);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_5_stacked_ctor_test, contact_graph_5_stacked_fixture )
// {
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 5);
//     BOOST_CHECK(uut->number_of_edges() == 4);
//     BOOST_CHECK(uut->vertices()[0] == po0);
//     BOOST_CHECK(uut->vertices()[1] == po1);
//     BOOST_CHECK(uut->vertices()[2] == po2);
//     BOOST_CHECK(uut->vertices()[3] == po3);
//     BOOST_CHECK(uut->vertices()[4] == po4);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
//     BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 5);
//     BOOST_CHECK(uut->number_of_edges() == 4);
//     BOOST_CHECK(uut->vertices()[0] == po1);
//     BOOST_CHECK(uut->vertices()[1] == po0);
//     BOOST_CHECK(uut->vertices()[2] == po2);
//     BOOST_CHECK(uut->vertices()[3] == po3);
//     BOOST_CHECK(uut->vertices()[4] == po4);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(0)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
//     BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_4_circle_ctor_test, contact_graph_4_circle_fixture )
// {
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 4);
//     BOOST_CHECK(uut->number_of_edges() == 4);
//     BOOST_CHECK(uut->vertices()[0] == po0);
//     BOOST_CHECK(uut->vertices()[1] == po1);
//     BOOST_CHECK(uut->vertices()[2] == po2);
//     BOOST_CHECK(uut->vertices()[3] == po6);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(3)[1].to() == 0);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[1].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 4);
//     BOOST_CHECK(uut->number_of_edges() == 4);
//     BOOST_CHECK(uut->vertices()[0] == po1);
//     BOOST_CHECK(uut->vertices()[1] == po0);
//     BOOST_CHECK(uut->vertices()[2] == po6);
//     BOOST_CHECK(uut->vertices()[3] == po2);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(3)[1].to() == 0);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[1].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_5_p_ctor_test, contact_graph_5_p_fixture )
// {
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 5);
//     BOOST_CHECK(uut->number_of_edges() == 5);
//     BOOST_CHECK(uut->vertices()[0] == po0);
//     BOOST_CHECK(uut->vertices()[1] == po1);
//     BOOST_CHECK(uut->vertices()[2] == po2);
//     BOOST_CHECK(uut->vertices()[3] == po4);
//     BOOST_CHECK(uut->vertices()[4] == po6);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(0)[1].to() == 4);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(2)[2].to() == 4);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(4)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(4)[1].to() == 0);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(4)[1].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[2].edge_id() == uut->adjacent(4)[0].edge_id());

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po4);
    
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 5);
//     BOOST_CHECK(uut->number_of_edges() == 5);
//     BOOST_CHECK(uut->vertices()[0] == po4);
//     BOOST_CHECK(uut->vertices()[1] == po2);
//     BOOST_CHECK(uut->vertices()[2] == po1);
//     BOOST_CHECK(uut->vertices()[3] == po0);
//     BOOST_CHECK(uut->vertices()[4] == po6);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(1)[2].to() == 4);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
//     BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
//     BOOST_CHECK(uut->adjacent(4)[1].to() == 1);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[2].edge_id() == uut->adjacent(4)[1].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_ctor_test, contact_graph_9_disjoint_fixture )
// {
//     /* Construct */
//     uut->rebuild(&no_friction, info, po2);

//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 5);
//     BOOST_CHECK(uut->number_of_edges() == 4);
//     BOOST_CHECK(uut->vertices()[0] == po2);
//     BOOST_CHECK(uut->vertices()[1] == po1);
//     BOOST_CHECK(uut->vertices()[2] == po0);
//     BOOST_CHECK(uut->vertices()[3] == po3);
//     BOOST_CHECK(uut->vertices()[4] == po4);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(3)[1].to() == 4);
//     BOOST_CHECK(uut->adjacent(4)[0].to() == 3);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(3)[1].edge_id() == uut->adjacent(4)[0].edge_id());

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po6);
    
//     /* Check graph */
//     BOOST_CHECK(uut->number_of_vertices() == 4);
//     BOOST_CHECK(uut->number_of_edges() == 4);
//     BOOST_CHECK(uut->vertices()[0] == po6);
//     BOOST_CHECK(uut->vertices()[1] == po5);
//     BOOST_CHECK(uut->vertices()[2] == po8);
//     BOOST_CHECK(uut->vertices()[3] == po7);
//     BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(0)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
//     BOOST_CHECK(uut->adjacent(1)[1].to() == 2);
//     BOOST_CHECK(uut->adjacent(2)[0].to() == 1);
//     BOOST_CHECK(uut->adjacent(2)[1].to() == 3);
//     BOOST_CHECK(uut->adjacent(3)[0].to() == 2);
//     BOOST_CHECK(uut->adjacent(3)[1].to() == 0);
//     BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(0)[1].edge_id() == uut->adjacent(3)[1].edge_id());
//     BOOST_CHECK(uut->adjacent(1)[1].edge_id() == uut->adjacent(2)[0].edge_id());
//     BOOST_CHECK(uut->adjacent(2)[1].edge_id() == uut->adjacent(3)[0].edge_id());
// }

// BOOST_FIXTURE_TEST_CASE ( contact_graph_void_collisions_test, contact_graph_9_disjoint_fixture )
// {
//     /* Void */
//     uut->void_collisions(&info);

//     /* Check the stack is void */
//     BOOST_CHECK(info[po0]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po0]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po0]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po1]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po1]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po1]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po2]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po2]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po2]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po3]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po3]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po3]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po4]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po4]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po4]->get_first_collision() == nullptr);

//     /* Check the circle isnt void */
//     BOOST_CHECK(info[po5]->get_first_collision_time() == 0.0f);
//     BOOST_CHECK(info[po5]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
//     BOOST_CHECK(info[po5]->get_first_collision() == po6);

//     BOOST_CHECK(info[po6]->get_first_collision_time() == 0.0f);
//     BOOST_CHECK(info[po6]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
//     BOOST_CHECK(info[po6]->get_first_collision() == po7);

//     BOOST_CHECK(info[po7]->get_first_collision_time() == 0.0f);
//     BOOST_CHECK(info[po7]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
//     BOOST_CHECK(info[po7]->get_first_collision() == po8);

//     BOOST_CHECK(info[po8]->get_first_collision_time() == 0.0f);
//     BOOST_CHECK(info[po8]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
//     BOOST_CHECK(info[po8]->get_first_collision() == po5);

//     /* Rebuild for the circle */
//     uut->rebuild(&no_friction, info, po7);
//     uut->void_collisions(&info);

//     /* Check the stack is void */
//     BOOST_CHECK(info[po0]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po0]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po0]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po1]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po1]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po1]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po2]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po2]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po2]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po3]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po3]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po3]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po4]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po4]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po4]->get_first_collision() == nullptr);

//     /* Check the circle is void */
//     BOOST_CHECK(info[po5]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po5]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po5]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po6]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po6]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po6]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po7]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po7]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po7]->get_first_collision() == nullptr);

//     BOOST_CHECK(info[po8]->get_first_collision_time() == std::numeric_limits<float>::max());
//     BOOST_CHECK(info[po8]->get_first_collision_type() == collision_t::NO_COLLISION);
//     BOOST_CHECK(info[po8]->get_first_collision() == nullptr);
// }

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po0->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po1->get_torque() == point_t(0.0f, 0.0f, 0.0f));

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po0->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po1->get_torque() == point_t(0.0f, 0.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, -2.0f));
    BOOST_CHECK(po0->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    
    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, -2.0f));
    BOOST_CHECK(po0->get_torque() == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_torque() == point_t(0.0f, 0.0f, 0.0f));
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    uut->rebuild(&dynamic_friction, info, po0);
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.525f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.475f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.525f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.475f,  0.0f, 0.0f))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po0->get_force()  - point_t(-0.707107f,  0.707107f, -2.0f))) < result_tolerance, po0->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po0->get_torque() - point_t( 0.671751f,  0.671751f,  0.0f))) < result_tolerance, po0->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po1->get_force()  - point_t( 0.707107f, -0.707107f, -1.0f))) < result_tolerance, po1->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po1->get_torque() - point_t( 0.742462f,  0.742462f,  0.0f))) < result_tolerance, po1->get_torque());
    
//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.707107f,  0.707107f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.671751f,  0.671751f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.707107f, -0.707107f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.742462f,  0.742462f,  0.0f))) < result_tolerance);
}

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_resting_force_test, contact_graph_2_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po0->set_torque(point_t(-1.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po1->set_torque(point_t(0.0f, 2.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, 0.0f));
//     BOOST_CHECK(po0->get_torque() == point_t(-1.0f, 0.0f, -1.0f));
//     BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, 0.0f));
//     BOOST_CHECK(po1->get_torque() == point_t(0.0f, 2.0f, 1.0f));

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, 0.0f));
//     BOOST_CHECK(po0->get_torque() == point_t(-1.0f, 0.0f, -1.0f));
//     BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, 0.0f));
//     BOOST_CHECK(po1->get_torque() == point_t(0.0f, 2.0f, 1.0f));

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -4.0f));
//     po0->set_torque(point_t(-3.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po1->set_torque(point_t(0.0f, 1.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, -2.0f));
//     BOOST_CHECK(po0->get_torque() == point_t(-3.0f, 0.0f, -1.0f));
//     BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, -1.0f));
//     BOOST_CHECK(po1->get_torque() == point_t(0.0f, 1.0f, 1.0f));

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, -2.0f));
//     BOOST_CHECK(po0->get_torque() == point_t(-3.0f, 0.0f, -1.0f));
//     BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, -1.0f));
//     BOOST_CHECK(po1->get_torque() == point_t(0.0f, 1.0f, 1.0f));
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_resting_force_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po11);

//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -4.0f));
//     po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -3.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8f,  0.0f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -3.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8f,  0.0f,  0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     uut->rebuild(&dynamic_friction, info, po10);
//     po10->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po10->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     po11->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.2f, -0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,   -0.4f,  0.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.2f,  0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.401f,  0.0f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.2f, -0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,   -0.4f,  0.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.2f,  0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.401f,  0.0f,  0.0f))) < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&dynamic_friction, info, po11);

//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -4.0f));
//     po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po10->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
//     po11->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.4f,   0.0f,   -3.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t( 0.0f,  -0.8f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(-0.4f,   0.0f,    0.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.8f,  -0.002f, -0.4f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.4f,   0.0f,   -3.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t( 0.0f,  -0.8f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(-0.4f,   0.0f,    0.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.8f,  -0.002f, -0.4f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_non_perp_resting_force_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(  0.0f, 2.0f, -1.0f));
//     po10->set_torque(point_t(-3.0f, 0.0f,  0.5f));
//     po11->set_force(point_t(  3.0f, 0.0f,  1.0f));
//     po11->set_torque(point_t( 0.0f, 0.7f,  2.0f));
//     uut->resolve_forces(1.0f);

//     /* Shows only forces and torques that cause the relative acceleration to increase matter */
//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.0f,  2.0f, -0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(-3.0f, -0.4f,  0.5f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t( 3.0f,  0.0f,  0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.4f,  0.7f,  2.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.0f,  2.0f, -0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(-3.0f, -0.4f,  0.5f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t( 3.0f,  0.0f,  0.6f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.4f,  0.7f,  2.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_resting_force_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po10->set_torque(point_t(0.0f, 3.0f, 0.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po11->set_torque(point_t(1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -0.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,  0.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -0.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,  0.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0f,  0.0f))) < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po11);

//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -4.0f));
//     po10->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po11->set_torque(point_t(-1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -2.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f, -0.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -2.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f, -0.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_friction_fight_resting_force_test, contact_graph_2_side_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     uut->rebuild(&dynamic_friction, info, po14);
//     po14->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po15->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po14->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
//     po15->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po14->get_force()  - point_t(-0.1081f,  0.0f,   -0.7838f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -0.0540f, 0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_force()  - point_t( 0.1081f,  0.0f,    0.7838f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.1621f, 0.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     po14->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
//     po15->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po14->get_force()  - point_t(-0.1081f,  0.0f,   -0.7838f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -0.0540f, 0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_force()  - point_t( 0.1081f,  0.0f,    0.7838f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.1621f, 0.0f   ))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_friction_help_resting_force_test, contact_graph_2_side_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     uut->rebuild(&dynamic_friction, info, po14);
//     po14->set_force(point_t(0.0f, 0.0f, -5.0f));
//     po15->set_force(point_t(0.0f, 0.0f,  5.0f));
//     po14->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
//     po15->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
//     po14->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po15->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po14->get_force()  - point_t( 1.2907f,  0.0f,    -2.4186f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -1.9360f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_force()  - point_t(-1.2907f,  0.0f,     2.4186f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.64535f, 0.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     po14->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
//     po15->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
//     po14->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po15->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po14->get_force()  - point_t( 1.2907f,  0.0f,    -2.4186f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -1.9360f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_force()  - point_t(-1.2907f,  0.0f,     2.4186f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.64535f, 0.0f   ))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_resting_force_test, contact_graph_9_disjoint_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po2->set_force(point_t(0.0f, 0.0f,  0.0f));
//     po3->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po4->set_force(point_t(0.0f, 0.0f,  1.0f));

//     po5->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po6->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po7->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po8->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po8->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
//     /* Rebuild, but from po6 */
//     uut->rebuild(&no_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f, 0.0f, -4.5f));
//     po6->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po7->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po8->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_tanjent_velocity_resting_force_test, contact_graph_9_disjoint_fixture )
// {
//     /* Resolve forces and check vgs */
//     uut->rebuild(&dynamic_friction, info, po0);
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po2->set_force(point_t(0.0f, 0.0f,  0.0f));
//     po3->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po4->set_force(point_t(0.0f, 0.0f,  1.0f));

//     po5->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po6->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po7->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po8->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

//     po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     po3->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,   -0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,    1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,   -1.9f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,    0.9f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.525f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.475f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 1.05f,   0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.005f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.855f,  0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,   -0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,    1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,   -1.9f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,    0.9f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.525f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.475f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 1.05f,   0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.005f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.855f,  0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
//     /* Rebuild, but from po6 */
//     uut->rebuild(&dynamic_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f, 0.0f, -4.5f));
//     po6->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po7->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po8->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

//     po6->set_velocity(point_t(1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.3898f, 0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-1.4419f, 0.0f,    -0.0234f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0520f, 0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.9766f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,   -0.1588f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,   -1.3578f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,   -0.1589f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,    0.0f,     0.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.3898f, 0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-1.4419f, 0.0f,    -0.0234f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0520f, 0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.9766f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,   -0.1588f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,   -1.3578f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,   -0.1589f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,    0.0f,     0.0f   ))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_rotational_force_resting_force_test, contact_graph_9_disjoint_fixture )
// {
//     /* Construct */
//     uut->rebuild(&no_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f,  0.0f, -4.5f));
//     po6->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po7->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po8->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f, -1.0f,  0.0f));
//     po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f,  1.0f,  0.0f));
//     po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    
//     /* Rebuild, but from po6 */
//     uut->rebuild(&no_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f,  0.0f, -4.5f));
//     po6->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po7->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po8->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f,  2.0f,  0.0f));
//     po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f, -1.0f,  0.0f));
//     po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f,   -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f,   -0.875)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f,   -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f,   -0.125)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.125,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,    0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.125,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,    0.0f)))    < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f,   -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f,   -0.875)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f,   -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f,   -0.125)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.125,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,    0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.125,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,    0.0f)))    < result_tolerance);
// }

//  Test not working
// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_linear_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po2->set_force(point_t(0.0f, 0.0f,  0.0f));
//     po3->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po4->set_force(point_t(0.0f, 0.0f,  1.0f));

//     po5->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po6->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po7->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po8->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po8->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
//     /* Rebuild, but from po6 */
//     uut->rebuild(&no_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f, 0.0f, -4.5f));
//     po6->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po7->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po8->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
// }

// Test causes painleve paradox
// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_linear_force_tanjent_velocity_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
// {
//     /* Resolve forces and check vgs */
//     // uut->rebuild(&dynamic_friction, info, po0);
//     // po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     // po1->set_force(point_t(0.0f, 0.0f, -1.0f));
//     // po2->set_force(point_t(0.0f, 0.0f,  0.0f));
//     // po3->set_force(point_t(0.0f, 0.0f,  1.0f));
//     // po4->set_force(point_t(0.0f, 0.0f,  1.0f));

//     // po5->set_force( point_t(0.0f, 0.0f, 1.1f));
//     // po6->set_force( point_t(0.0f, 0.0f, 1.2f));
//     // po7->set_force( point_t(0.0f, 0.0f, 1.3f));
//     // po8->set_force( point_t(0.0f, 0.0f, 1.4f));
//     // po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     // po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     // po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     // po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

//     // po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     // po3->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     // uut->resolve_forces(1.0f);

//     // BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,     0.5f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    -0.5f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,     1.0f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,    -1.9f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,     0.9f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.7533f, -0.0866f,  0.0433f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.3766f, -0.0433f, -0.0433f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.7099f,  0.29f,   -0.0799f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.26f,   -0.16f,    0.0799f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.9f,     0.0f,     0.0f   ))) < result_tolerance);

//     // BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     // /* Re run and check no change */
//     // uut->resolve_forces(1.0f);

//     // BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,     0.5f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    -0.5f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,     1.0f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,    -1.9f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,     0.9f,     0.0f   ))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.7533f, -0.0866f,  0.0433f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.3766f, -0.0433f, -0.0433f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.7099f,  0.29f,   -0.0799f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.26f,   -0.16f,    0.0799f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.9f,     0.0f,     0.0f   ))) < result_tolerance);

//     // BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     // BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
//     /* Rebuild, but from po6 */
//     uut->rebuild(&dynamic_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f, 0.0f, -4.5f));
//     po6->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po7->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po8->set_force( point_t(0.0f, 0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
//     po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

//     po6->set_velocity(point_t(1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_force()  - point_t( 1.3898f, 0.0f,    -0.25f  ))) < result_tolerance, po5->get_force());
//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_force()  - point_t(-1.4419f, 0.0f,    -0.0234f))) < result_tolerance, po6->get_force());
//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_force()  - point_t( 0.0520f, 0.0f,    -0.25f  ))) < result_tolerance, po7->get_force());
//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.9766f))) < result_tolerance, po8->get_force());
//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,   -0.1588f,  0.0f   ))) < result_tolerance, po5->get_torque());
//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,   -1.3578f,  0.0f   ))) < result_tolerance, po6->get_torque());
//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,   -0.1589f,  0.0f   ))) < result_tolerance, po7->get_torque());
//     BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,    0.0f,     0.0f   ))) < result_tolerance, po8->get_torque());

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.3898f, 0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-1.4419f, 0.0f,    -0.0234f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0520f, 0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.9766f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,   -0.1588f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,   -1.3578f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,   -0.1589f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,    0.0f,     0.0f   ))) < result_tolerance);
// }

// Test not working
// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_rotational_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
// {
//     /* Construct */
//     uut->rebuild(&no_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f,  0.0f, -4.5f));
//     po6->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po7->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po8->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f, -1.0f,  0.0f));
//     po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f,  1.0f,  0.0f));
//     po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    
//      Rebuild, but from po6 
//     uut->rebuild(&no_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_force( point_t(0.0f,  0.0f, -4.5f));
//     po6->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po7->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po8->set_force( point_t(0.0f,  0.0f,  1.0f));
//     po5->set_torque(point_t(0.0f,  2.0f,  0.0f));
//     po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
//     po7->set_torque(point_t(0.0f, -1.0f,  0.0f));
//     po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

//     po0->set_force( point_t(0.0f, 0.0f, 1.0f));
//     po1->set_force( point_t(0.0f, 0.0f, 1.1f));
//     po2->set_force( point_t(0.0f, 0.0f, 1.2f));
//     po3->set_force( point_t(0.0f, 0.0f, 1.3f));
//     po4->set_force( point_t(0.0f, 0.0f, 1.4f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.0f,    0.0f,    -0.7777f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.2222f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0370f, 0.1851f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.0740f, 0.0740f,  0.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.0f,    0.0f,    -0.7777f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.2222f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0370f, 0.1851f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.0740f, 0.0740f,  0.0f   ))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
// {
//     /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);

//     po0->set_force(point_t(0.0f, 0.0f, -5.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  11.0f));
//     po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_contact_4_point_fixture )
// {
//     /* Resolve forces and check vgs */
//     uut->rebuild(&dynamic_friction, info, po0);
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&dynamic_friction, info, po1);
    
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -4.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
//     po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.707107f,  0.707107f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.942809f,  0.942809f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.707107f, -0.707107f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.471403f,  0.471403f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.707107f,  0.707107f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.942809f,  0.942809f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.707107f, -0.707107f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.471403f,  0.471403f,  0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotation_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force( point_t( 0.0f, 0.0f, -1.0f));
//     po0->set_torque(point_t(-1.0f, 0.0f, -1.0f));
//     po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
//     po1->set_torque(point_t( 0.0f, 2.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Resolve forces and check vgs */
//     po0->set_force( point_t( 0.0f, 0.0f, -4.0f));
//     po0->set_torque(point_t(-3.0f, 0.0f, -1.0f));
//     po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
//     po1->set_torque(point_t( 0.0f, 1.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
// {
//     /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);

//     po0->set_force(point_t(0.0f, 0.0f, -5.0f));
//     po1->set_force(point_t(0.0f, 0.0f, 11.0f));
//     po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
// {
//     /* Resolve forces and check vgs */
//     uut->rebuild(&dynamic_friction, info, po0);
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&dynamic_friction, info, po1);
    
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -4.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
//     po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.707107f,  0.707107f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.942809f,  0.942809f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.707107f, -0.707107f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.471403f,  0.471403f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.707107f,  0.707107f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.942809f,  0.942809f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.707107f, -0.707107f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.471403f,  0.471403f,  0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotation_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force( point_t( 0.0f, 0.0f, -1.0f));
//     po0->set_torque(point_t(-1.0f, 0.0f, -1.0f));
//     po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
//     po1->set_torque(point_t( 0.0f, 2.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Resolve forces and check vgs */
//     po0->set_force( point_t( 0.0f, 0.0f, -4.0f));
//     po0->set_torque(point_t(-3.0f, 0.0f, -1.0f));
//     po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
//     po1->set_torque(point_t( 0.0f, 1.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_resting_impulse_test, contact_graph_2_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f)))      < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f)))      < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f)))      < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f)))      < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);

//     /* Resolve forces and check vgs */
//     po0->set_velocity(point_t(0.0f, 0.0f, -4.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f,  2.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -2.0f)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -2.0f)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f)))   < result_tolerance);
    
//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -2.0f)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -2.0f)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f)))   < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_resting_implulse_test, contact_graph_2_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po0->set_angular_velocity(point_t(-1.0f, 0.0f, -1.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po1->set_angular_velocity(point_t(0.0f, 2.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-1.0f, 0.0f, -1.0f)))      < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 2.0f,  1.0f)))      < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-1.0f, 0.0f, -1.0f)))      < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 2.0f,  1.0f)))      < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Resolve forces and check vgs */
//     po0->set_velocity(point_t(0.0f, 0.0f, -4.0f));
//     po0->set_angular_velocity(point_t(-3.0f, 0.0f, -1.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f,  2.0f));
//     po1->set_angular_velocity(point_t(0.0f, 1.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-3.0f, 0.0f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 1.0f,  1.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-3.0f, 0.0f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 1.0f,  1.0f))) < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_non_perp_resting_impulse_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po10->set_velocity(point_t(0.0f, 2.0f, -1.0f));
//     po10->set_angular_velocity(point_t(-3.0f, 0.0f, 0.5f));
//     po11->set_velocity(point_t(3.0f, 0.0f,  1.0f));
//     po11->set_angular_velocity(point_t(0.0f, 0.7, 2.0f));
//     uut->resolve_forces(1.0f);

//     /* Shows only forces and torques that cause the relative acceleration to increase matter */
//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  2.0f, -0.733333)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(-3.0f, -0.4f,  0.5f)))       < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 3.0f,  0.0f,  0.466666)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.7,  2.0f)))       < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  2.0f, -0.733333)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(-3.0f, -0.4f,  0.5f)))       < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 3.0f,  0.0f,  0.466666)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.7,  2.0f)))       < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_resting_impulse_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po10->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.733333)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f, -0.4f,  0.0f)))       < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f,  0.466666)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.0f,  0.0f)))       < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.733333)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f, -0.4f,  0.0f)))       < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f,  0.466666)))  < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,  0.0f,  0.0f)))       < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po11);

//     /* Resolve forces and check vgs */
//     po10->set_velocity(point_t(0.0f, 0.0f, -4.0f));
//     po10->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po11->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.33333)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -1.0f,  0.0f)))        < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.33333)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(2.0f,  0.0f,  0.0f)))        < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.33333)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -1.0f,  0.0f)))        < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.33333)))    < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(2.0f,  0.0f,  0.0f)))        < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_resting_impulse_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po10->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po10->set_angular_velocity(point_t(0.0f, 3.0f, 0.0f));
//     po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po11->set_angular_velocity(point_t(1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.4666666))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f,  2.2f,  0.0f)))       < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f, -0.06666)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 2.6,  0.0f,  0.0f)))       < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.4666666))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f,  2.2f,  0.0f)))       < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f, -0.06666)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 2.6,  0.0f,  0.0f)))       < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po11);

//     /* Resolve forces and check vgs */
//     po10->set_velocity(point_t(0.0f, 0.0f, -4.0f));
//     po10->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
//     po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po11->set_angular_velocity(point_t(-1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.06666))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -0.4f,  0.0f)))     < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.86666))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(1.8,  0.0f,  0.0f)))     < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.06666))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -0.4f,  0.0f)))     < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.86666))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(1.8,  0.0f,  0.0f)))     < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_resting_impulse_test, contact_graph_9_disjoint_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_velocity(point_t(0.0f, 0.0f, -0.5f));
//     po1->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po2->set_velocity(point_t(0.0f, 0.0f,  0.0f));
//     po3->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po4->set_velocity(point_t(0.0f, 0.0f,  1.0f));

//     po5->set_velocity(point_t(0.0f, 0.0f, 1.1f));
//     po6->set_velocity(point_t(0.0f, 0.0f, 1.2f));
//     po7->set_velocity(point_t(0.0f, 0.0f, 1.3f));
//     po8->set_velocity(point_t(0.0f, 0.0f, 1.4f));
//     po5->set_angular_velocity(point_t(0.0f, 1.1f, 0.0f));
//     po6->set_angular_velocity(point_t(0.0f, 1.2f, 0.0f));
//     po7->set_angular_velocity(point_t(0.0f, 1.3f, 0.0f));
//     po8->set_angular_velocity(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     /* Resolve forces and check vgs */
//     po0->set_velocity(point_t(0.0f, 0.0f,  2.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po2->set_velocity(point_t(0.0f, 0.0f,  0.0f));
//     po3->set_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po4->set_velocity(point_t(0.0f, 0.0f, -1.0f));

//     po5->set_velocity(point_t(0.0f, 0.0f, 1.1f));
//     po6->set_velocity(point_t(0.0f, 0.0f, 1.2f));
//     po7->set_velocity(point_t(0.0f, 0.0f, 1.3f));
//     po8->set_velocity(point_t(0.0f, 0.0f, 1.4f));
//     po5->set_angular_velocity(point_t(0.0f, 1.1f, 0.0f));
//     po6->set_angular_velocity(point_t(0.0f, 1.2f, 0.0f));
//     po7->set_angular_velocity(point_t(0.0f, 1.3f, 0.0f));
//     po8->set_angular_velocity(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f,  2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f,  1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f,  2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f,  1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
//     /* Rebuild, but from po6 */
//     uut->rebuild(&no_friction, info, po6);

//     /* Resolve forces and check vgs */
//     po5->set_velocity(point_t(0.0f, 0.0f, -4.5f));
//     po6->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po7->set_velocity(point_t(0.0f, 0.0f,  2.0f));
//     po8->set_velocity(point_t(0.0f, 0.0f,  1.0f));
//     po5->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));
//     po6->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));
//     po7->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));
//     po8->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));

//     po0->set_velocity(point_t(0.0f, 0.0f, 1.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f, 1.1f));
//     po2->set_velocity(point_t(0.0f, 0.0f, 1.2f));
//     po3->set_velocity(point_t(0.0f, 0.0f, 1.3f));
//     po4->set_velocity(point_t(0.0f, 0.0f, 1.4f));
//     po0->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_angular_velocity(point_t(0.0f, 1.1f, 0.0f));
//     po2->set_angular_velocity(point_t(0.0f, 1.2f, 0.0f));
//     po3->set_angular_velocity(point_t(0.0f, 1.3f, 0.0f));
//     po4->set_angular_velocity(point_t(0.0f, 1.4f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_resting_force_test, contact_graph_2_infinite_mass_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po12->set_force(point_t(0.0f,   0.0f, -1000.0f));
//     po13->set_force(point_t(0.0f,   0.0f,     0.0f));
//     po12->set_torque(point_t(0.0f, 250.0f,    0.0f));
//     po13->set_torque(point_t(0.0f,   0.0f,    0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t(0.0f,    0.0f, -450.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(0.0f, -300.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t(0.0f,    0.0f, -450.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(0.0f, -300.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_resting_impulse_test, contact_graph_2_infinite_mass_contact_fixture )
// {   
//     po12->set_velocity(point_t(0.0f, 0.0f, -4.0f));
//     po13->set_velocity(point_t(0.0f, 0.0f,  0.0f));
//     po12->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po13->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f,  0.0f, -2.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, -2.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);
 
//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f,  0.0f, -2.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, -2.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);

//     /* Rebuild, but from po12 */
//     uut->rebuild(&no_friction, info, po12);
    
//     po12->set_velocity(point_t(0.0f, 0.0f, -4.0f));
//     po13->set_velocity(point_t(0.0f, 0.0f,  0.0f));
//     po12->set_angular_velocity(point_t(0.0f, 6.0f, 0.0f));
//     po13->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
 
//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_resting_force_and_impulse_test, contact_graph_2_contact_fixture )
// {
//     /* No change for joint movement and movemnt in normal */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po0->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
//     po1->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
//     (*info[po0])[po1]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
//     (*info[po1])[po0]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    
//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);
    
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -160.0f));
//     po1->set_force(point_t(0.0f, 0.0f,   80.0f));
//     po0->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     po1->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -164.0f));
//     po1->set_force(point_t(0.0f, 0.0f,   81.0f));
//     po0->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     po1->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -162.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   79.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -162.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   79.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
// }


// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_resting_force_and_impulse_test, contact_graph_2_offset_contact_fixture )
// {
//     /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
//     po10->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po10->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
//     po11->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

//     /* Rebuild, but from po10 */
//     uut->rebuild(&no_friction, info, po10);

//     po10->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po10->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
//     po11->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,       0.0f,      -0.601897))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.398103,  0.0     ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,       0.0f,       0.601897))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.398103,  0.0f,       0.0     ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,       0.0f,      -0.601897))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.398103,  0.0     ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,       0.0f,       0.601897))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.398103,  0.0f,       0.0     ))) < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po11);

//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -160.0f));
//     po11->set_force(point_t(0.0f, 0.0f,   80.0f));
//     po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po10->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     po11->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     po10->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
//     (*info[po10])[po11]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
//     (*info[po11])[po10]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -164.0f));
//     po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_force(point_t(0.0f, 0.0f,   81.0f));
//     po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po10->set_velocity(point_t(0.0f,  1.0f, 0.0f));
//     po11->set_velocity(point_t(0.0f, -1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -163.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,   80.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0f,    0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -163.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,   80.2f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0f,    0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_resting_force_and_impulse_test, contact_graph_2_offset_contact_fixture )
// {
//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -161.0f));
//     po10->set_torque(point_t(0.0f, 3.0f, 0.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  81.0f));
//     po11->set_torque(point_t(1.0f, 0.0f, 0.0f));
//     po10->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
//     po11->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
//     (*info[po10])[po11]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(20.0f, 0.0f, 0.0f));
//     (*info[po11])[po10]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(20.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0f,    0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,    0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.4f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0f,    0.0f))) < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po11);

//     /* Resolve forces and check vgs */
//     po10->set_force(point_t(0.0f, 0.0f, -84.0f));
//     po10->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po11->set_force(point_t(0.0f, 0.0f,  41.0f));
//     po11->set_torque(point_t(-1.0f, 0.0f, 0.0f));
//     po10->set_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po11->set_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po10->set_angular_velocity(point_t(0.0f, 0.0f, -1.0f));
//     po11->set_angular_velocity(point_t(0.0f, 0.0f,  1.0f));
//     (*info[po10])[po11]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-10.0f, -10.0f, 0.0f));
//     (*info[po11])[po10]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-10.0f, -10.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -82.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  39.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,   0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -82.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  39.6))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,   0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_resting_force_and_impulse_test, contact_graph_2_contact_4_point_fixture )
// {
//     /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po0->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
//     po1->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);

//     po0->set_force(point_t(0.0f, 0.0f, -5.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  11.0f));
//     po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
//     po1->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotational_resting_force_test, contact_graph_2_contact_4_point_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(6.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     0.0f,       2.18182f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(1.09091f, 2.0f,       0.0f    ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,     0.0f,      -2.18182f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(4.90909f, 0.999997f,  0.0f    ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     0.0f,       2.18182f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(1.09091f, 2.0f,       0.0f    ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,     0.0f,      -2.18182f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(4.90909f, 0.999997f,  0.0f    ))) < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po1);

//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -160.0f));
//     po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  70.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 10.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotational_resting_force_and_impulse_test, contact_graph_2_contact_4_point_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -160.0f));
//     po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  70.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 10.0f));
//     po0->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
//     po1->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);

//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -84.0f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  41.0f));
//     po1->set_torque(point_t(-1.0f, 0.0f, 0.0f));
//     po0->set_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
//     po1->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
//     (*info[po0])[po1]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
//     (*info[po1])[po0]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_resting_force_and_impulse_test, contact_graph_2_contact_4_point_asym_fixture )
// {
//     /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
//     po0->set_force(point_t(0.0f, 0.0f, -1.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  1.0f));
//     po0->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
//     po1->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);

//     po0->set_force(point_t(0.0f, 0.0f, -5.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  11.0f));
//     po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
//     po1->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotational_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_torque(point_t(0.0f, -3.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_torque(point_t(6.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      0.0f,       1.10127f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(3.85443f, -2.44937f,   0.0f    ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.0f,      -1.10127f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.14557f, -0.550634f,  0.0f    ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      0.0f,       1.10127f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(3.85443f, -2.44937f,   0.0f    ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.0f,      -1.10127f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.14557f, -0.550634f,  0.0f    ))) < result_tolerance);

//     /* Rebuild, but from po11 */
//     uut->rebuild(&no_friction, info, po1);

//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -160.0f));
//     po0->set_torque(point_t(0.0f, -3.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  70.0f));
//     po1->set_torque(point_t(6.0f, 0.0f, 10.0f));
//     po0->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
//     po1->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,  0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(4.0f, -2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,  0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.0f, -1.0f,  10.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,  0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(4.0f, -2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,  0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.0f, -1.0f,  10.0f))) < result_tolerance);
// }

// BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotational_resting_force_and_impulse_test, contact_graph_2_contact_4_point_asym_fixture )
// {
//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -160.0f));
//     po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  70.0f));
//     po1->set_torque(point_t(0.0f, 0.0f, 10.0f));
//     po0->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
//     po1->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

//     /*  Rebuild, but from po1 */
//     uut->rebuild(&no_friction, info, po1);

//     /* Resolve forces and check vgs */
//     po0->set_force(point_t(0.0f, 0.0f, -84.0f));
//     po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
//     po1->set_force(point_t(0.0f, 0.0f,  41.0f));
//     po1->set_torque(point_t(-1.0f, 0.0f, 0.0f));
//     po0->set_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po1->set_velocity(point_t(0.0f, 0.0f, 0.0f));
//     po0->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
//     po1->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
//     (*info[po0])[po1]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
//     (*info[po1])[po0]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);

//     /* Re run and check no change */
//     uut->resolve_forces(1.0f);

//     BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
//     BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);
// }

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
