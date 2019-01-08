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
        s0( { point_t(2.5f, 0.5f,  1.45f) },  point_t(0.0f, 0.0f,  1.0f)),
        s1( { point_t(2.5f, 0.5f,  1.45f) },  point_t(0.0f, 0.0f, -1.0f)),
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
                { 0, new rigid_body_collider(0.0f, 0.0f, 0.0f )},
                { 1, new rigid_body_collider(0.0f, 0.0f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.0f, 0.0f )}},
            },
            { 1, new std::map<unsigned int, const collider*>{
                { 1, new rigid_body_collider(0.0f, 0.0f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.0f, 0.0f )}},
            },
            { 2, new std::map<unsigned int, const collider*>{
                { 2, new rigid_body_collider(0.0f, 0.0f, 0.0f )}},
            }
        },
        dynamic_friction{
            { 0, new std::map<unsigned int, const collider*>{
                { 0, new rigid_body_collider(0.0f, 0.0f, 0.1f )},
                { 1, new rigid_body_collider(0.0f, 0.0f, 0.5f )},
                { 2, new rigid_body_collider(0.0f, 0.0f, 0.9f )}},
            },
            { 1, new std::map<unsigned int, const collider*>{
                { 1, new rigid_body_collider(0.0f, 0.0f, 0.2f )},
                { 2, new rigid_body_collider(0.0f, 0.0f, 0.8f )}},
            },
            { 2, new std::map<unsigned int, const collider*>{
                { 2, new rigid_body_collider(0.0f, 0.0f, 0.7f )}},
            }
        },
        static_friction{
            { 0, new std::map<unsigned int, const collider*>{
                { 0, new rigid_body_collider(0.0f, 0.1f, 0.0f )},
                { 1, new rigid_body_collider(0.0f, 0.5f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.9f, 0.0f )}},
            },
            { 1, new std::map<unsigned int, const collider*>{
                { 1, new rigid_body_collider(0.0f, 0.2f, 0.0f )},
                { 2, new rigid_body_collider(0.0f, 0.8f, 0.0f )}},
            },
            { 2, new std::map<unsigned int, const collider*>{
                { 2, new rigid_body_collider(0.0f, 0.7f, 0.0f )}},
            }
        },
        default_collider(0.0f, 0.0f, 0.0f),
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

    std::map<mock_physics_object*, tracking_info<mock_physics_object, mock_simplex>*>   info;
    rigid_body_collider                                                                 default_collider;
    contact_graph<mock_physics_object, mock_simplex> *                                  uut;
};


struct contact_graph_2_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_contact_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s0), s1, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s1), s0, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
    };
};

struct contact_graph_2_offset_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_offset_contact_fixture()
    {
        info = {{po10, new tracking_info<mock_physics_object, mock_simplex>(po11, new mock_simplex(s12), s13, 0.0f, collision_t::SLIDING_COLLISION)},
                {po11, new tracking_info<mock_physics_object, mock_simplex>(po10, new mock_simplex(s13), s12, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po10);
    };
};

struct contact_graph_2_side_offset_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_side_offset_contact_fixture()
    {
        info = {{po14, new tracking_info<mock_physics_object, mock_simplex>(po15, new mock_simplex(s20), s21, 0.0f, collision_t::SLIDING_COLLISION)},
                {po15, new tracking_info<mock_physics_object, mock_simplex>(po14, new mock_simplex(s21), s20, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po14);
    };
};

struct contact_graph_2_contact_4_point_fixture : public contact_graph_base_fixture
{
    contact_graph_2_contact_4_point_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s16), s17, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s17), s16, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
    };
};

struct contact_graph_2_contact_4_point_asym_fixture : public contact_graph_base_fixture
{
    contact_graph_2_contact_4_point_asym_fixture()
    {
        info = {{po0, new tracking_info<mock_physics_object, mock_simplex>(po1, new mock_simplex(s18), s19, 0.0f, collision_t::SLIDING_COLLISION)},
                {po1, new tracking_info<mock_physics_object, mock_simplex>(po0, new mock_simplex(s19), s18, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
    };
};

struct contact_graph_2_infinite_mass_contact_fixture : public contact_graph_base_fixture
{
    contact_graph_2_infinite_mass_contact_fixture()
    {
        info = {{po12, new tracking_info<mock_physics_object, mock_simplex>(po13, new mock_simplex(s14), s15, 0.0f, collision_t::SLIDING_COLLISION)},
                {po13, new tracking_info<mock_physics_object, mock_simplex>(po12, new mock_simplex(s15), s14, 0.0f, collision_t::SLIDING_COLLISION)}};

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po12);
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

        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
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
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
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
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
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
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
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
                {po7, new tracking_info<mock_physics_object, mock_simplex>(po8, new mock_simplex(s28), s29, 0.0f, collision_t::SLIDING_COLLISION)},
                {po8, new tracking_info<mock_physics_object, mock_simplex>(po5, new mock_simplex(s30), s31, 0.0f, collision_t::SLIDING_COLLISION)}};

        info[po1]->update(po0, new mock_simplex(s17), s16, 0.0f, collision_t::SLIDING_COLLISION);
        info[po2]->update(po1, new mock_simplex(s23), s22, 0.0f, collision_t::SLIDING_COLLISION);
        info[po3]->update(po2, new mock_simplex(s25), s24, 0.0f, collision_t::SLIDING_COLLISION);

        info[po6]->update(po5, new mock_simplex(s17), s16, 0.0f, collision_t::SLIDING_COLLISION);
        info[po7]->update(po6, new mock_simplex(s23), s22, 0.0f, collision_t::SLIDING_COLLISION);
        info[po8]->update(po7, new mock_simplex(s29), s28, 0.0f, collision_t::SLIDING_COLLISION);
        info[po5]->update(po8, new mock_simplex(s31), s30, 0.0f, collision_t::SLIDING_COLLISION);
        
        uut = new contact_graph<mock_physics_object, mock_simplex>(&no_friction, &default_collider, info, po0);
    };
};

const float result_tolerance = 0.0005f;

BOOST_AUTO_TEST_SUITE( contact_graph_tests );


BOOST_AUTO_TEST_CASE( default_ctor_test )
{
    /* Construct */
    contact_graph<mock_physics_object, mock_simplex> uut;

    /* Check graph */
    BOOST_CHECK(uut.number_of_vertices() == 0);
    BOOST_CHECK_MESSAGE(uut.number_of_contacts() == 0, uut.number_of_contacts());
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_ctor_test, contact_graph_2_contact_fixture )
{
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 2);
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 1, uut->number_of_contacts());
    BOOST_CHECK(uut->vertices()[0] == po0);
    BOOST_CHECK(uut->vertices()[1] == po1);
    BOOST_CHECK(uut->adjacent(0)[0].to() == 1);
    BOOST_CHECK(uut->adjacent(1)[0].to() == 0);
    BOOST_CHECK(uut->adjacent(0)[0].edge_id() == uut->adjacent(1)[0].edge_id());

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 2);
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 1, uut->number_of_contacts());
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
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 4, uut->number_of_contacts());
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
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 4, uut->number_of_contacts());
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
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 4, uut->number_of_contacts());
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
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 4);
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 4, uut->number_of_contacts());
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
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 5, uut->number_of_contacts());
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
    uut->rebuild(&no_friction, &default_collider, info, po4);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 5, uut->number_of_contacts());
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
    uut->rebuild(&no_friction, &default_collider, info, po2);

    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 5);
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 4, uut->number_of_contacts());
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
    uut->rebuild(&no_friction, &default_collider, info, po6);
    
    /* Check graph */
    BOOST_CHECK(uut->number_of_vertices() == 4);
    BOOST_CHECK_MESSAGE(uut->number_of_contacts() == 4, uut->number_of_contacts());
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
    BOOST_CHECK(info[po0]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po0]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po1]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po1]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po1]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po2]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po2]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po2]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po3]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po3]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po3]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po4]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po4]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po4]->get_first_collision() == nullptr);

    /* Check the circle isnt void */
    BOOST_CHECK(info[po5]->get_first_collision_time() == 0.0f);
    BOOST_CHECK(info[po5]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(info[po5]->get_first_collision() == po6);

    BOOST_CHECK(info[po6]->get_first_collision_time() == 0.0f);
    BOOST_CHECK(info[po6]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(info[po6]->get_first_collision() == po7);

    BOOST_CHECK(info[po7]->get_first_collision_time() == 0.0f);
    BOOST_CHECK(info[po7]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(info[po7]->get_first_collision() == po8);

    BOOST_CHECK(info[po8]->get_first_collision_time() == 0.0f);
    BOOST_CHECK(info[po8]->get_first_collision_type() == collision_t::SLIDING_COLLISION);
    BOOST_CHECK(info[po8]->get_first_collision() == po5);

    /* Rebuild for the circle */
    uut->rebuild(&no_friction, &default_collider, info, po7);
    uut->void_collisions(&info);

    /* Check the stack is void */
    BOOST_CHECK(info[po0]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po0]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po0]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po1]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po1]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po1]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po2]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po2]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po2]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po3]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po3]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po3]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po4]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po4]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po4]->get_first_collision() == nullptr);

    /* Check the circle is void */
    BOOST_CHECK(info[po5]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po5]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po5]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po6]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po6]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po6]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po7]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po7]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po7]->get_first_collision() == nullptr);

    BOOST_CHECK(info[po8]->get_first_collision_time() == std::numeric_limits<float>::max());
    BOOST_CHECK(info[po8]->get_first_collision_type() == collision_t::NO_COLLISION);
    BOOST_CHECK(info[po8]->get_first_collision() == nullptr);
}

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
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, -2.0f));
    BOOST_CHECK(po0->get_torque() == point_t(0.0f, 0.0f,  0.0f));
    BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_torque() == point_t(0.0f, 0.0f,  0.0f));
    
    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t(0.0f, 0.0f, -2.0f));
    BOOST_CHECK(po0->get_torque() == point_t(0.0f, 0.0f,  0.0f));
    BOOST_CHECK(po1->get_force()  == point_t(0.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_torque() == point_t(0.0f, 0.0f,  0.0f));
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,  0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.5f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.5f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,  0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.5f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.5f,  0.0f, 0.0f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894427f,  0.447214f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894427f, -0.447214f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);
    
    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894427f,  0.447214f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894427f, -0.447214f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_halting_tanjent_velocity_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(3.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      0.355555f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.355555f, 0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,     -0.355555f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.355555f, 0.0f,      0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(3.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      0.355555f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.355555f, 0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,     -0.355555f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.355555f, 0.0f,      0.0f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po1);
    uut->resolve_forces(4.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.799999f,  0.4f,      -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.4f,       0.799999f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.799999f, -0.4f,      -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.4f,       0.799999f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(4.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.799999f,  0.4f,      -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.4f,       0.799999f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.799999f, -0.4f,      -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.4f,       0.799999f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_tanjent_velocity_tanjent_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(   point_t(0.0f, -1.0f, -2.0f));
    po1->set_force(   point_t(0.0f,  1.0f,  2.0f));
    po0->set_velocity(point_t(0.0f, -1.0f,  0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f,  0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(3.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     -0.244445f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.755555f, 0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.244445f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.755555f, 0.0f,      0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(3.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     -0.244445f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.755555f, 0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.244445f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.755555f, 0.0f,      0.0f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(   point_t(0.0f, -1.0f, -4.0f));
    po1->set_force(   point_t(0.0f,  2.0f,  1.0f));
    po0->set_torque(  point_t(0.0f,  0.0f,  0.0f));
    po1->set_torque(  point_t(0.0f,  0.0f,  0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f,  0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f,  0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po1);
    uut->resolve_forces(4.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894427f, -0.552786f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894427f,  1.55279f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(4.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894427f, -0.552786f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894427f,  1.55279f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.447214f,  0.894427f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_tanjent_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f,  1.0f, -1.0f));
    po1->set_force(point_t(0.0f, -1.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,  0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.4f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f, -0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.4f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,  0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.4f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f, -0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.4f,  0.0f, 0.0f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(7.0f, -1.0f, -4.0f));
    po1->set_force(point_t(1.0f,  2.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.33333f, -0.333334f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.666666f, 0.666666f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.66667f,  1.33333f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.666666f, 0.666666f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.33333f, -0.333334f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.666666f, 0.666666f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.66667f,  1.33333f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.666666f, 0.666666f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_starting_tanjent_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f,  1.0f, -1.0f));
    po1->set_force(point_t(0.0f, -3.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,  0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.5f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f, -2.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.5f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,  0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.5f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f, -2.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.5f,  0.0f, 0.0f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(9.0f, -5.0f, -2.0f));
    po1->set_force(point_t(1.0f,  3.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(8.64208f, -4.43756f,  -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.562441f, 0.357917f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.35792f,  2.43756f,  -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.562441f, 0.357917f,  0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(8.64208f, -4.43756f,  -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.562441f, 0.357917f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.35792f,  2.43756f,  -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.562441f, 0.357917f,  0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f, 0.0f, -1.0f));
    po0->set_torque(point_t(-1.0f, 0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, 0.0f,  1.0f));
    po1->set_torque(point_t( 0.0f, 2.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t( 0.0f, 0.0f,  0.0f));
    BOOST_CHECK(po0->get_torque() == point_t(-1.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_force()  == point_t( 0.0f, 0.0f,  0.0f));
    BOOST_CHECK(po1->get_torque() == point_t( 0.0f, 2.0f,  1.0f));

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t( 0.0f, 0.0f,  0.0f));
    BOOST_CHECK(po0->get_torque() == point_t(-1.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_force()  == point_t( 0.0f, 0.0f,  0.0f));
    BOOST_CHECK(po1->get_torque() == point_t( 0.0f, 2.0f,  1.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f, 0.0f, -4.0f));
    po0->set_torque(point_t(-3.0f, 0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, 0.0f,  1.0f));
    po1->set_torque(point_t( 0.0f, 1.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t( 0.0f, 0.0f, -2.0f));
    BOOST_CHECK(po0->get_torque() == point_t(-3.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_force()  == point_t( 0.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_torque() == point_t( 0.0f, 1.0f,  1.0f));

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(po0->get_force()  == point_t( 0.0f, 0.0f, -2.0f));
    BOOST_CHECK(po0->get_torque() == point_t(-3.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_force()  == point_t( 0.0f, 0.0f, -1.0f));
    BOOST_CHECK(po1->get_torque() == point_t( 0.0f, 1.0f,  1.0f));
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_tanjent_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f,  1.0f, -3.0f));
    po0->set_torque(point_t(-1.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, -1.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  2.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.800001f,  0.8f,      -1.33333f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.2f, -     0.800001f, -1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.800001f, -0.8f,      -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.2f,       1.2f,       1.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.800001f,  0.8f,      -1.33333f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.2f, -     0.800001f, -1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.800001f, -0.8f,      -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.2f,       1.2f,       1.0f     ))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  7.0f, -1.0f, -4.0f));
    po0->set_torque(point_t(-3.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  1.0f,  2.0f,  3.0f));
    po1->set_torque(point_t( 0.0f,  1.0f,  1.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.73333f, 0.266665f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.73334f, 0.266665f, -1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.26667f, 0.733335f, -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 1.26666f, 1.26667f,   1.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.73333f, 0.266665f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.73334f, 0.266665f, -1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.26667f, 0.733335f, -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 1.26666f, 1.26667f,   1.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_starting_tanjent_force_resting_force_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f,  1.0f, -1.0f));
    po0->set_torque(point_t(-1.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, -1.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  2.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.485071f,  0.878732f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.12127f,  -0.485071f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.485071f, -0.878732f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.121268f,  1.51493f,   1.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.485071f,  0.878732f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.12127f,  -0.485071f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.485071f, -0.878732f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.121268f,  1.51493f,   1.0f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  7.0f, -1.0f, -4.0f));
    po0->set_torque(point_t(-3.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  1.0f,  2.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  1.0f,  1.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.79399f, -0.0214502f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.02145f,  0.20601f,   -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.20601f,  1.02145f,   -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.97855f,  1.20601f,    1.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.79399f, -0.0214502f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.02145f,  0.20601f,   -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.20601f,  1.02145f,   -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.97855f,  1.20601f,    1.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -1.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -4.0f));
    po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -3.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8f,  0.0f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -3.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8f,  0.0f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -1.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    po10->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po11->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po10);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.2f, -0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,   -0.4f,  0.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.2f,  0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.401f,  0.0f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.2f, -0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,   -0.4f,  0.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.2f,  0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.401f,  0.0f,  0.0f))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -4.0f));
    po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po10->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
    po11->set_velocity(point_t( 1.0f, 0.0f, 0.0f));

    /* Rebuild, but from po11 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po11);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.4f,   0.0f,   -3.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t( 0.0f,  -0.8f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(-0.4f,   0.0f,    0.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.8f,  -0.002f, -0.4f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.4f,   0.0f,   -3.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t( 0.0f,  -0.8f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(-0.4f,   0.0f,    0.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.8f,  -0.002f, -0.4f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_halting_tanjent_velocity_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -1.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    po10->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po11->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po10);
    uut->resolve_forces(10.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.177774f, -0.600395f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,   -0.399605f,  0.177774f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.177774f,  0.600395f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.401f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(10.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.177774f, -0.600395f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,   -0.399605f,  0.177774f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.177774f,  0.600395f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.401f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -4.0f));
    po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po10->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
    po11->set_velocity(point_t( 1.0f, 0.0f, 0.0f));

    /* Rebuild, but from po11 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po11);
    uut->resolve_forces(10.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.133331f,  0.0f,         -3.2f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t( 0.0f,      -0.8f,          0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(-0.133331f,  0.0f,          0.200001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.799999f, -0.000740722f, -0.133331f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(10.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.133331f,  0.0f,         -3.2f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t( 0.0f,      -0.8f,          0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(-0.133331f,  0.0f,          0.200001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.799999f, -0.000740722f, -0.133331f))) < result_tolerance);
}

/* Note not completely stopping acceleration because the reactions force requires some to prevent inter penetration */
BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_tanjent_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f,  1.0f, -5.0f));
    po11->set_force(point_t(0.0f, -1.0f,  4.0f));
    uut->rebuild(&static_friction, &default_collider, info, po10);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,      0.333347f, -3.26519f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -1.73481f,  -0.666653f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,     -0.333347f,  2.26519f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.73111f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,      0.333347f, -3.26519f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -1.73481f,  -0.666653f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,     -0.333347f,  2.26519f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.73111f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t( 4.0f,  3.0f, -27.0f));
    po10->set_torque(point_t(0.0f,  0.0f,   0.0f));
    po11->set_force(point_t( 6.0f, -2.0f,   5.0f));
    po11->set_torque(point_t(0.0f,  0.0f,   0.0f));

    /* Rebuild, but from po11 */
    uut->rebuild(&static_friction, &default_collider, info, po11);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(5.49542f,  1.6915f,     -22.0638f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -4.93624f,     -1.3085f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(4.50458f, -0.691504f,     0.0637622f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(4.92897f, -0.00830785f,  -1.49542f  ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(5.49542f,  1.6915f,     -22.0638f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -4.93624f,     -1.3085f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(4.50458f, -0.691504f,     0.0637622f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(4.92897f, -0.00830785f,  -1.49542f  ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_starting_tanjent_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f,  1.0f, -1.0f));
    po11->set_force(point_t(0.0f, -1.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po10);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,      0.799778f, -0.599555f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -0.400445f, -0.200222f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,     -0.799778f,  0.599555f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.399332f, 0.0f,       0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,      0.799778f, -0.599555f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -0.400445f, -0.200222f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,     -0.799778f,  0.599555f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.399332f, 0.0f,       0.0f     ))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t( 0.0f,  7.0f, -4.0f));
    po10->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po11->set_force(point_t( 0.0f, -4.0f,  1.0f));
    po11->set_torque(point_t(0.0f,  0.0f,  0.0f));

    /* Rebuild, but from po11 */
    uut->rebuild(&static_friction, &default_collider, info, po11);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,       6.59956f,  -3.19911f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.800889f, -0.400445f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,      -3.59956f,   0.199111f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.798665f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,       6.59956f,  -3.19911f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.800889f, -0.400445f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,      -3.59956f,   0.199111f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.798665f,  0.0f,       0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_non_perp_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(  0.0f, 2.0f, -1.0f));
    po10->set_torque(point_t(-3.0f, 0.0f,  0.5f));
    po11->set_force(point_t(  3.0f, 0.0f,  1.0f));
    po11->set_torque(point_t( 0.0f, 0.7f,  2.0f));
    uut->resolve_forces(1.0f);

    /* Shows only forces and torques that cause the relative acceleration to increase matter */
    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.0f,  2.0f, -0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(-3.0f, -0.4f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t( 3.0f,  0.0f,  0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.4f,  0.7f,  2.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t( 0.0f,  2.0f, -0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(-3.0f, -0.4f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t( 3.0f,  0.0f,  0.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t( 0.4f,  0.7f,  2.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t( 0.0f, 0.0f, -1.0f));
    po10->set_torque(point_t(0.0f, 3.0f,  0.0f));
    po11->set_force(point_t( 0.0f, 0.0f,  1.0f));
    po11->set_torque(point_t(1.0f, 0.0f,  0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -0.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,  0.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6f, 0.0f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -0.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,  0.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6f, 0.0f,  0.0f))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(  0.0f, 0.0f, -4.0f));
    po10->set_torque(point_t( 0.0f, 1.0f,  0.0f));
    po11->set_force(point_t(  0.0f, 0.0f,  1.0f));
    po11->set_torque(point_t(-1.0f, 0.0f,  0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -2.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f, -0.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -2.6f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f, -0.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_tanjent_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t( 0.0f,  1.0f, -8.0f));
    po10->set_torque(point_t(0.0f,  3.0f,  0.0f));
    po11->set_force(point_t( 0.0f, -1.0f,  1.0f));
    po11->set_torque(point_t(1.0f,  0.0f,  0.0f));
    uut->rebuild(&static_friction, &default_collider, info, po10);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,     0.329643f, -6.46518f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     1.46518f,  -0.670357f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,    -0.329643f, -0.53482f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(2.5311f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,     0.329643f, -6.46518f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     1.46518f,  -0.670357f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,    -0.329643f, -0.53482f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(2.5311f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(  4.0f,  3.0f,  -4.0f));
    po10->set_torque(point_t( 0.0f,  1.0f,   0.0f));
    po11->set_force(point_t(  6.0f, -2.0f,  14.0f));
    po11->set_torque(point_t(-1.0f,  0.0f,   0.0f));

    /* Rebuild, but from po11 */
    uut->rebuild(&static_friction, &default_collider, info, po11);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(5.49493f,  1.69505f,     0.869564f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -3.86956f,    -1.30495f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(4.50507f, -0.695048f,    9.13044f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(3.86231f, -0.00830512f, -1.49493f ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(5.49493f,  1.69505f,     0.869564f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,     -3.86956f,    -1.30495f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(4.50507f, -0.695048f,    9.13044f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(3.86231f, -0.00830512f, -1.49493f ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_starting_tanjent_force_resting_force_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t( 0.0f,  1.0f, -1.0f));
    po10->set_torque(point_t(0.0f,  3.0f,  0.0f));
    po11->set_force(point_t( 0.0f, -1.0f,  1.0f));
    po11->set_torque(point_t(1.0f,  0.0f,  0.0f));
    uut->rebuild(&static_friction, &default_collider, info, po10);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.699667f, -0.399335f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,    2.39933f,  -0.300333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.699667f,  0.399335f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.599f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,    0.699667f, -0.399335f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,    2.39933f,  -0.300333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,   -0.699667f,  0.399335f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.599f,  0.0f,       0.0f     ))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(  4.0f,  3.0f, -4.0f));
    po10->set_torque(point_t( 0.0f,  1.0f,  0.0f));
    po11->set_force(point_t(  6.0f, -2.0f,  1.0f));
    po11->set_torque(point_t(-1.0f,  0.0f,  0.0f));

    /* Rebuild, but from po11 */
    uut->rebuild(&static_friction, &default_collider, info, po11);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(4.52773f,   2.53933f,    -2.59898f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.401024f,   -0.460667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(5.47227f,  -1.53933f,    -0.401024f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.398464f, -0.00293183f, -0.527733f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(4.52773f,   2.53933f,    -2.59898f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.401024f,   -0.460667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(5.47227f,  -1.53933f,    -0.401024f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.398464f, -0.00293183f, -0.527733f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_friction_fight_resting_force_test, contact_graph_2_side_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po14->set_force(point_t(0.0f, 0.0f, -1.0f));
    po15->set_force(point_t(0.0f, 0.0f,  1.0f));
    po14->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
    po15->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po14);
    uut->resolve_forces(0.1f);

    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po14->get_force()  - point_t(-0.1081f,  0.0f,   -0.7838f))) < result_tolerance, po14->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -0.0540f, 0.0f   ))) < result_tolerance, po14->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po15->get_force()  - point_t( 0.1081f,  0.0f,    0.7838f))) < result_tolerance, po15->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.1621f, 0.0f   ))) < result_tolerance, po15->get_torque());

    /* Re run and check no change */
    po14->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
    po15->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po14->get_force()  - point_t(-0.1081f,  0.0f,   -0.7838f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -0.0540f, 0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po15->get_force()  - point_t( 0.1081f,  0.0f,    0.7838f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.1621f, 0.0f   ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_friction_help_resting_force_test, contact_graph_2_side_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po14->set_force(point_t(0.0f, 0.0f, -5.0f));
    po15->set_force(point_t(0.0f, 0.0f,  5.0f));
    po14->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
    po15->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
    po14->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po15->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po14);
    uut->resolve_forces(0.5f);

    BOOST_CHECK(std::fabs(magnitude(po14->get_force()  - point_t( 1.2907f,  0.0f,    -2.4186f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -1.9360f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po15->get_force()  - point_t(-1.2907f,  0.0f,     2.4186f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.64535f, 0.0f   ))) < result_tolerance);

    /* Re run and check no change */
    po14->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
    po15->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
    po14->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po15->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(0.5f);

    BOOST_CHECK(std::fabs(magnitude(po14->get_force()  - point_t( 1.2907f,  0.0f,    -2.4186f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po14->get_torque() - point_t( 0.0f,    -1.9360f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po15->get_force()  - point_t(-1.2907f,  0.0f,     2.4186f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po15->get_torque() - point_t( 0.0f,     0.64535f, 0.0f   ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f, -1.0f));
    po2->set_force(point_t(0.0f, 0.0f,  0.0f));
    po3->set_force(point_t(0.0f, 0.0f,  1.0f));
    po4->set_force(point_t(0.0f, 0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(&no_friction, &default_collider, info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(0.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_tanjent_velocity_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f, -1.0f));
    po2->set_force(point_t(0.0f, 0.0f,  0.0f));
    po3->set_force(point_t(0.0f, 0.0f,  1.0f));
    po4->set_force(point_t(0.0f, 0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po3->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,    1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,   -1.9f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,    0.9f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.5f,    0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.5f,    0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 1.05f,   0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.005f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.855f,  0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,    1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,   -1.9f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,    0.9f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.5f,    0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.5f,    0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 1.05f,   0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.005f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.855f,  0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(0.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po6);
    po6->set_velocity(point_t(1.0f, 0.0f, 0.0f));
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.37021f,  0.0f,     -0.25f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-1.422f,    0.0f,     -0.00461823f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0520f,   0.0f,     -0.25f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,      0.0f,     -0.995382f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,     -0.165127,  0.0f       ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,     -1.26923,   0.0f       ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,     -0.165127,  0.0f       ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,      0.0f,      0.0f       ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.37021f,  0.0f,     -0.25f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-1.422f,    0.0f,     -0.00461823f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0520f,   0.0f,     -0.25f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,      0.0f,     -0.995382f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,     -0.165127,  0.0f       ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,     -1.26923,   0.0f       ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,     -0.165127,  0.0f       ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,      0.0f,      0.0f       ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_halting_tanjent_velocity_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f, -1.0f));
    po2->set_force(point_t(0.0f, 0.0f,  0.0f));
    po3->set_force(point_t(0.0f, 0.0f,  1.0f));
    po4->set_force(point_t(0.0f, 0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po3->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,        0.106667f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.106667f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.089155f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,       -0.180752f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.0915965f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.106667f,   0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.106667f,   0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.0936128f,  0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(-0.0114791f,  0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.0870167f,  0.0f,       0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,        0.106667f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.106667f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.089155f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,       -0.180752f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.0915965f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.106667f,   0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.106667f,   0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.0936128f,  0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(-0.0114791f,  0.0f,       0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.0870167f,  0.0f,       0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(0.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po6);
    po6->set_velocity(point_t(1.0f, 0.0f, 0.0f));
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.120697f,   0.0f,        -0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.179979f,   0.0f,        -0.475857f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0592817f,  0.0f,        -0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,        0.0f,        -0.524142f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,       -0.00804734f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,       -0.00509777f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,       -0.00804734f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,        0.0f,         0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.120697f,   0.0f,        -0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.179979f,   0.0f,        -0.475857f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0592817f,  0.0f,        -0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,        0.0f,        -0.524142f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,       -0.00804734f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,       -0.00509777f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,       -0.00804734f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,        0.0f,         0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_tanjent_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -1.0f, -1.0f));
    po1->set_force(point_t(0.0f,  0.0f, -1.0f));
    po2->set_force(point_t(0.0f,  0.0f,  0.0f));
    po3->set_force(point_t(0.0f,  1.0f,  1.0f));
    po4->set_force(point_t(0.0f,  0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       -0.866667f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.133333f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.222888f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,        0.548121f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.228991f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.133333f,   0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.133333f,   0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.234032f,   0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(-0.0286977f,  0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.217542f,   0.0f,      0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       -0.866667f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.133333f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.222888f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,        0.548121f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.228991f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.133333f,   0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.133333f,   0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.234032f,   0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(-0.0286977f,  0.0f,      0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.217542f,   0.0f,      0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(1.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  4.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.161224f,  0.0f,        0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.720924f,  0.0f,        0.518474f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.117851f,  0.0f,        0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,       0.0f,        0.481525f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,      -0.00615835f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,       0.0685856f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,      -0.00615826f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,       0.0f,        0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.161224f,  0.0f,        0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.720924f,  0.0f,        0.518474f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.117851f,  0.0f,        0.25f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,       0.0f,        0.481525f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0f,      -0.00615835f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0f,       0.0685856f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,      -0.00615826f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,       0.0f,        0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_starting_tanjent_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -4.0f, -1.0f));
    po1->set_force(point_t(0.0f,  0.0f, -1.0f));
    po2->set_force(point_t(0.0f,  0.0f,  0.0f));
    po3->set_force(point_t(0.0f,  6.0f,  1.0f));
    po4->set_force(point_t(0.0f,  0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       -3.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,        4.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.9f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.5f,        0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.5f,        0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 1.05f,       0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.00499958f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.855f,      0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       -3.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,        4.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.9f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.5f,        0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.5f,        0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 1.05f,       0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.00499958f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.855f,      0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -3.5f));
    po6->set_force( point_t(9.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(1.1014f,    0.0f,     -0.0833335f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(7.85321f,   0.0f,      0.230186f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0453961f, 0.0f,     -0.0833332f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f,       0.0f,     -0.563519f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f,      -0.132284f, 0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f,      -1.01288f,  0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f,      -0.132284f, 0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f,       0.0f,      0.0f      ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(  1.1014f,    0.0f,     -0.0833335f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(  7.85321f,   0.0f,      0.230186f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(  0.0453961f, 0.0f,     -0.0833332f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(  0.0f,       0.0f,     -0.563519f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(  0.0f,      -0.132284f, 0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(  0.0f,      -1.01288f,  0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(  0.0f,      -0.132284f, 0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(  0.0f,       0.0f,      0.0f      ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_rotational_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Construct */
    uut->rebuild(&no_friction, &default_collider, info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  0.0f, -4.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f,  0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f,  1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(&no_friction, &default_collider, info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  0.0f, -4.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f,  0.0f,  1.0f));
    po5->set_torque(point_t(0.0f,  2.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f,  -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f,  -0.875))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f,  -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f,  -0.125))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.125,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,   0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.125,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,   0.0f ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f,  -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f,  -0.875))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f,  -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f,  -0.125))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.125,  0.0f)))  < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,   0.0f)))  < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.125,  0.0f)))  < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,   0.0f)))  < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_rotational_force_tanjent_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  1.0f, -10.0f));
    po6->set_force( point_t(0.0f,  0.0f,   1.0f));
    po7->set_force( point_t(0.0f,  0.0f,   5.0f));
    po8->set_force( point_t(0.0f, -1.0f,   1.0f));
    po5->set_torque(point_t(0.0f, -1.0f,   0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,   0.0f));
    po7->set_torque(point_t(0.0f,  1.0f,   0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,   0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(-0.534865f,  0.558811f,  -0.500001f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.438845f,  0.159144f,  -1.04166f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(-0.430476f, -0.0808236f, -0.5f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.526496f, -0.637131f,  -0.958335f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.455292f,  0.0435024f,  0.122901f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.159144f, -0.151532f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0767824f, 0.0138888f,  0.0808236f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.110337f, -0.167838f,   0.0f      ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_force()  - point_t(-0.534865f,  0.558811f,  -0.500001f ))) < result_tolerance, po5->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_force()  - point_t( 0.438845f,  0.159144f,  -1.04166f  ))) < result_tolerance, po6->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_force()  - point_t(-0.430476f, -0.0808236f, -0.5f      ))) < result_tolerance, po7->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_force()  - point_t( 0.526496f, -0.637131f,  -0.958335f ))) < result_tolerance, po8->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_torque() - point_t(-0.455292f,  0.0435024f,  0.122901f ))) < result_tolerance, po5->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_torque() - point_t(-0.159144f, -0.151532f,   0.0f      ))) < result_tolerance, po6->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_torque() - point_t( 0.0767824f, 0.0138888f,  0.0808236f))) < result_tolerance, po7->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_torque() - point_t(-0.110337f, -0.167838f,   0.0f      ))) < result_tolerance, po8->get_torque());


    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  1.0f, -20.0f));
    po6->set_force( point_t(0.0f,  0.0f,   1.0f));
    po7->set_force( point_t(0.0f,  0.0f,   5.0f));
    po8->set_force( point_t(0.0f, -1.0f,   1.0f));
    po5->set_torque(point_t(0.0f,  2.0f,   0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,   0.0f));
    po7->set_torque(point_t(0.0f, -1.0f,   0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,   0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    
    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.05247f,    0.566047f,  -2.16667f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.698488f,   0.156849f,  -4.49821f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.487401f,  -0.091039f,  -2.16667f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(-0.841382f,  -0.631857f,  -4.16846f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.447809f,  -0.00323594f, 0.120255f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.156849f,  -0.0260686f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.086487f,   0.0549565f,  0.091039f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.0857228f, -0.0468735f,  0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_force()  - point_t( 1.05247f,    0.566047f,  -2.16667f ))) < result_tolerance, po5->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_force()  - point_t(-0.698488f,   0.156849f,  -4.49821f ))) < result_tolerance, po6->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_force()  - point_t( 0.487401f,  -0.091039f,  -2.16667f ))) < result_tolerance, po7->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_force()  - point_t(-0.841382f,  -0.631857f,  -4.16846f ))) < result_tolerance, po8->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_torque() - point_t(-0.447809f,  -0.00323594f, 0.120255f))) < result_tolerance, po5->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_torque() - point_t(-0.156849f,  -0.0260686f,  0.0f     ))) < result_tolerance, po6->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_torque() - point_t( 0.086487f,   0.0549565f,  0.091039f))) < result_tolerance, po7->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_torque() - point_t(-0.0857228f, -0.0468735f,  0.0f     ))) < result_tolerance, po8->get_torque());
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_rotational_force_starting_tanjent_force_resting_force_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  6.0f, -2.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f, -4.0f,  1.0f));
    po5->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f,  1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(-0.463201f,  3.88373f,   0.0833334f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.228097f,  0.876405f,  0.0861985f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(-0.248857f, -0.336647f,  0.0833334f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.48396f,  -2.42349f,   0.247135f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-2.17826f,   0.0512327f, 0.363459f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.876405f,  0.201561f,  0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.319814f,  0.0268227f, 0.336647f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.521409f, -0.235836f,  0.0f      ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_force()  - point_t(-0.463201f,  3.88373f,   0.0833334f))) < result_tolerance, po5->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_force()  - point_t( 0.228097f,  0.876405f,  0.0861985f))) < result_tolerance, po6->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_force()  - point_t(-0.248857f, -0.336647f,  0.0833334f))) < result_tolerance, po7->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_force()  - point_t( 0.48396f,  -2.42349f,   0.247135f ))) < result_tolerance, po8->get_force());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_torque() - point_t(-2.17826f,   0.0512327f, 0.363459f ))) < result_tolerance, po5->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_torque() - point_t(-0.876405f,  0.201561f,  0.0f      ))) < result_tolerance, po6->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_torque() - point_t( 0.319814f,  0.0268227f, 0.336647f ))) < result_tolerance, po7->get_torque());
    // BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_torque() - point_t(-0.521409f, -0.235836f,  0.0f      ))) < result_tolerance, po8->get_torque());


    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  6.0f, -0.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f, -4.0f,  1.0f));
    po5->set_torque(point_t(0.0f,  2.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    
    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.493179f,  5.01364f,   0.72531f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.576318f,  0.986355f,  0.606084f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0831394f, 0.0f,       0.168606f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,      -4.0f,       1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.986355f,  0.281511f, -0.986355f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.986355f, -0.331057f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,      -0.0896236f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,       0.0f,       0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.493179f,  5.01364f,   0.72531f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.576318f,  0.986355f,  0.606084f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0831394f, 0.0f,       0.168606f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,      -4.0f,       1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.986355f,  0.281511f, -0.986355f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.986355f, -0.331057f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0f,      -0.0896236f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0f,       0.0f,       0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_linear_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f, -1.0f));
    po2->set_force(point_t(0.0f, 0.0f,  0.0f));
    po3->set_force(point_t(0.0f, 0.0f,  1.0f));
    po4->set_force(point_t(0.0f, 0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(&no_friction, &default_collider, info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(0.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f ))) < result_tolerance);
}

/* Test can cause painleve paradox */
BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_linear_force_tanjent_velocity_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f, -1.0f));
    po2->set_force(point_t(0.0f, 0.0f,  0.0f));
    po3->set_force(point_t(0.0f, 0.0f,  1.0f));
    po4->set_force(point_t(0.0f, 0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po3->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       0.5f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      -0.499999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,       0.999999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,      -1.9f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,       0.9f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.920001f,  0.0800003f, -0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.459999f,  0.0399996f,  0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.459999f,  0.0399998f, -0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.259999f, -0.16f,       0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.9f,       0.0f,        0.0f      ))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       0.5f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      -0.499999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,       0.999999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,      -1.9f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,       0.9f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.920001f,  0.0800003f, -0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.459999f,  0.0399996f,  0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.459999f,  0.0399998f, -0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.259999f, -0.16f,       0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.9f,       0.0f,        0.0f      ))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(0.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po6);
    po6->set_velocity(point_t(1.0f, 0.0f, 0.0f));
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.17976f,    0.0f,      -0.25f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-1.22292f,    0.0f,       0.120664f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0431512f,  0.0f,      -0.25f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,        0.0f,      -1.12066f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.0413775f, -0.206888f,  0.0372395f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.0827554f, -0.413777f, -0.0496528f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.0413777f, -0.206888f,  0.0124133f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.165511f,  -0.165511f,  0.0f      ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(0.1f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.17976f,    0.0f,      -0.25f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-1.22292f,    0.0f,       0.120664f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0431512f,  0.0f,      -0.25f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,        0.0f,      -1.12066f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.0413775f, -0.206888f,  0.0372395f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.0827554f, -0.413777f, -0.0496528f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.0413777f, -0.206888f,  0.0124133f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.165511f,  -0.165511f,  0.0f      ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_linear_force_halting_tanjent_velocity_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -2.0f));
    po1->set_force(point_t(0.0f, 0.0f, -1.0f));
    po2->set_force(point_t(0.0f, 0.0f,  0.0f));
    po3->set_force(point_t(0.0f, 0.0f,  1.0f));
    po4->set_force(point_t(0.0f, 0.0f,  2.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po3->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,        0.106667f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.106667f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.0888888f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,       -0.177778f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.088889f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.136f,      0.0151118f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.0679999f,  0.00755533f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.0679995f,  0.0075556f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.0302225f, -0.0302227f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.088889f,   0.0f,        0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(5.0f);

    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,        0.106667f,   0.0f))) < result_tolerance, po0->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.106667f,   0.0f))) < result_tolerance, po1->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.0888888f,  0.0f))) < result_tolerance, po2->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,       -0.177778f,   0.0f))) < result_tolerance, po3->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.088889f,   0.0f))) < result_tolerance, po4->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po0->get_torque() - point_t( 0.136f,      0.0151118f,  0.0f))) < result_tolerance, po0->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po1->get_torque() - point_t( 0.0679999f,  0.00755533f, 0.0f))) < result_tolerance, po1->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po2->get_torque() - point_t( 0.0679995f,  0.0075556f,  0.0f))) < result_tolerance, po2->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po3->get_torque() - point_t( 0.0302225f, -0.0302227f,  0.0f))) < result_tolerance, po3->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po4->get_torque() - point_t(-0.088889f,   0.0f,        0.0f))) < result_tolerance, po4->get_torque());

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(0.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po6);
    po6->set_velocity(point_t(1.0f, 0.0f, 0.0f));
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.121149f,    0.0f,        -0.250002f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.179859f,    0.0f,        -0.47999f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.05871f,     0.0f,        -0.249998f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,         0.0f,        -0.52001f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.00133413f, -0.00667048f, -0.000134811f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.00266771f, -0.0133407f,  -0.000265444f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.00133456f, -0.00667099f,  0.000400255f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0053364f,  -0.0053364f,   0.0f        ))) < result_tolerance);

    /* Re run and check no change */                                                                                                                          
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.121149f,    0.0f,        -0.250002f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.179859f,    0.0f,        -0.47999f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.05871f,     0.0f,        -0.249998f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,         0.0f,        -0.52001f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.00133413f, -0.00667048f, -0.000134811f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.00266771f, -0.0133407f,  -0.000265444f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.00133456f, -0.00667099f,  0.000400255f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.0053364f,  -0.0053364f,   0.0f        ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_linear_force_tanjent_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -1.0f, -1.0f));
    po1->set_force(point_t(0.0f,  0.0f, -1.0f));
    po2->set_force(point_t(0.0f,  0.0f,  0.0f));
    po3->set_force(point_t(0.0f,  1.0f,  1.0f));
    po4->set_force(point_t(0.0f,  0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       -0.866667f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.133333f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.222222f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,        0.555556f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.222134f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.22f,       0.0244449f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.11f,       0.012222f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.11f,       0.0122221f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.0488887f, -0.048889f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.222134f,   0.0f,        0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,       -0.866667f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,       -0.133333f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,        0.222222f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,        0.555556f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,        0.222134f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.22f,       0.0244449f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.11f,       0.012222f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.11f,       0.0122221f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.0488887f, -0.048889f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.222134f,   0.0f,        0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -4.5f));
    po6->set_force( point_t(1.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  4.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.161395f,    0.0f,         0.249998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.724845f,    0.0f,         0.498022f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.113761f,    0.0f,         0.250002f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,         0.0f,         0.501978f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.000131696f, 0.000658274f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.000263751f, 0.00131746f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.000131458f, 0.000658277f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.000526905f, 0.000526905f, 0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.161395f,    0.0f,         0.249998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.724845f,    0.0f,         0.498022f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.113761f,    0.0f,         0.250002f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,         0.0f,         0.501978f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.000131696f, 0.000658274f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.000263751f, 0.00131746f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.000131458f, 0.000658277f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.000526905f, 0.000526905f, 0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_linear_force_starting_tanjent_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -4.0f, -1.0f));
    po1->set_force(point_t(0.0f,  0.0f, -1.0f));
    po2->set_force(point_t(0.0f,  0.0f,  0.0f));
    po3->set_force(point_t(0.0f,  6.0f,  1.0f));
    po4->set_force(point_t(0.0f,  0.0f,  1.0f));

    po5->set_force( point_t(0.0f, 0.0f, 1.1f));
    po6->set_force( point_t(0.0f, 0.0f, 1.2f));
    po7->set_force( point_t(0.0f, 0.0f, 1.3f));
    po8->set_force( point_t(0.0f, 0.0f, 1.4f));
    po5->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po6->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po7->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po8->set_torque(point_t(0.0f, 1.4f, 0.0f));

    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      -3.5f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      -0.499999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,       0.999999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,       4.1f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,       0.9f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.92f,      0.0800003f, -0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.459999f,  0.0399997f,  0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.459999f,  0.0399997f, -0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.259999f, -0.16f,       0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.899999f,  0.0f,        0.0f      ))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      -3.5f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      -0.499999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t( 0.0f,       0.999999f,   0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t( 0.0f,       4.1f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t( 0.0f,       0.9f,        0.0f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.92f,      0.0800003f, -0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.459999f,  0.0399997f,  0.0400001f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t( 0.459999f,  0.0399997f, -0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t( 0.259999f, -0.16f,       0.0799998f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(-0.899999f,  0.0f,        0.0f      ))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f, 0.0f, -3.5f));
    po6->set_force( point_t(9.0f, 0.0f,  1.0f));
    po7->set_force( point_t(0.0f, 0.0f,  1.0f));
    po8->set_force( point_t(0.0f, 0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po6->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, 0.0f,  0.0f));
    po8->set_torque(point_t(0.0f, 0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));

    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.948169f,   0.0f,      -0.0833337f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 8.01355f,    0.0f,       0.329258f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0382777f,  0.0f,      -0.083332f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,        0.0f,      -0.662592f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.0330614f, -0.165308f,  0.0297553f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.0661232f, -0.330618f, -0.0396736f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.0330622f, -0.165309f,  0.00991828f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.132247f,  -0.132247f,  0.0f       ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.948169f,   0.0f,      -0.0833337f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 8.01355f,    0.0f,       0.329258f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0382777f,  0.0f,      -0.083332f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,        0.0f,      -0.662592f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.0330614f, -0.165308f,  0.0297553f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.0661232f, -0.330618f, -0.0396736f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.0330622f, -0.165309f,  0.00991828f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.132247f,  -0.132247f,  0.0f       ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_rotational_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Construct */
    uut->rebuild(&no_friction, &default_collider, info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  0.0f, -4.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f,  0.0f,  1.0f));
    po5->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f,  1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(0.0f, 0.0f, -0.25f)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(0.0f, 0.0f, -0.5f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(0.0f, 0.0f,  0.0f)))    < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(&no_friction, &default_collider, info, po6);

    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  0.0f, -4.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f,  0.0f,  1.0f));
    po5->set_torque(point_t(0.0f,  2.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.0f,    0.0f,    -0.7777f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.2222f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0370f, 0.1851f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.0740f, 0.0740f,  0.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.0f,    0.0f,    -0.7777f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0f,    0.0f,    -0.25f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.0f,    0.0f,    -0.2222f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t( 0.0370f, 0.1851f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.0185f, 0.0925f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.0740f, 0.0740f,  0.0f   ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_rotational_force_tanjent_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  1.0f, -10.0f));
    po6->set_force( point_t(0.0f,  0.0f,   1.0f));
    po7->set_force( point_t(0.0f,  0.0f,   5.0f));
    po8->set_force( point_t(0.0f, -1.0f,   1.0f));
    po5->set_torque(point_t(0.0f, -1.0f,   0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,   0.0f));
    po7->set_torque(point_t(0.0f,  1.0f,   0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,   0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(-0.515732f,  0.547651f,  -0.499999f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.374132f,  0.126763f,  -0.952692f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(-0.434299f, -0.0834372f, -0.5f      ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.575899f, -0.590976f,  -1.04731f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.103993f, -0.0157724f,  0.0497037f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.207986f, -0.0315399f,  0.0994122f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.103995f, -0.0157695f,  0.0497071f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.246757f, -0.070305f,   0.0834375f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_force()  - point_t(-0.515732f,  0.547651f,  -0.499999f ))) < result_tolerance, po5->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_force()  - point_t( 0.374132f,  0.126763f,  -0.952692f ))) < result_tolerance, po6->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_force()  - point_t(-0.434299f, -0.0834372f, -0.5f      ))) < result_tolerance, po7->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_force()  - point_t( 0.575899f, -0.590976f,  -1.04731f  ))) < result_tolerance, po8->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_torque() - point_t(-0.103993f, -0.0157724f,  0.0497037f))) < result_tolerance, po5->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_torque() - point_t(-0.207986f, -0.0315399f,  0.0994122f))) < result_tolerance, po6->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_torque() - point_t(-0.103995f, -0.0157695f,  0.0497071f))) < result_tolerance, po7->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_torque() - point_t(-0.246757f, -0.070305f,   0.0834375f))) < result_tolerance, po8->get_torque());


    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  1.0f, -20.0f));
    po6->set_force( point_t(0.0f,  0.0f,   1.0f));
    po7->set_force( point_t(0.0f,  0.0f,   5.0f));
    po8->set_force( point_t(0.0f, -1.0f,   1.0f));
    po5->set_torque(point_t(0.0f,  2.0f,   0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,   0.0f));
    po7->set_torque(point_t(0.0f, -1.0f,   0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,   0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    
    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.02593f,   0.561689f,  -2.16667f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.663704f,  0.149303f,  -4.43013f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.488866f, -0.0910238f, -2.16666f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(-0.851088f, -0.619969f,  -4.23653f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.0986196f, 0.0321535f,  0.0399973f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.19724f,   0.0643087f,  0.0799684f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.0986188f, 0.032154f,   0.0368328f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.217775f,  0.0437709f,  0.0736647f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 1.02593f,   0.561689f,  -2.16667f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.663704f,  0.149303f,  -4.43013f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.488866f, -0.0910238f, -2.16666f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(-0.851088f, -0.619969f,  -4.23653f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.0986196f, 0.0321535f,  0.0399973f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.19724f,   0.0643087f,  0.0799684f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.0986188f, 0.032154f,   0.0368328f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.217775f,  0.0437709f,  0.0736647f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_multi_point_rotational_force_starting_tanjent_force_resting_force_test, contact_graph_9_disjoint_multi_point_fixture )
{
    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  6.0f, -2.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f, -4.0f,  1.0f));
    po5->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f,  1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t(-0.4304f,    4.01904f,    0.324455f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t( 0.301324f,  0.943516f,   0.149384f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t(-0.260986f, -0.260721f,  -0.306245f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t( 0.390062f, -2.70184f,    0.332406f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-1.22843f,  -0.1055f,     0.59389f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-1.36882f,  -0.211003f,  -0.532689f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t( 0.032968f,  0.314964f,   0.30339f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t(-0.641536f, -0.0775413f, -0.00994667f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_force()  - point_t(-0.4304f,    4.01904f,    0.324455f  ))) < result_tolerance, po5->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_force()  - point_t( 0.301324f,  0.943516f,   0.149384f  ))) < result_tolerance, po6->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_force()  - point_t(-0.260986f, -0.260721f,  -0.306245f  ))) < result_tolerance, po7->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_force()  - point_t( 0.390062f, -2.70184f,    0.332406f  ))) < result_tolerance, po8->get_force());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po5->get_torque() - point_t(-1.22843f,  -0.1055f,     0.59389f   ))) < result_tolerance, po5->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po6->get_torque() - point_t(-1.36882f,  -0.211003f,  -0.532689f  ))) < result_tolerance, po6->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po7->get_torque() - point_t( 0.032968f,  0.314964f,   0.30339f   ))) < result_tolerance, po7->get_torque());
    BOOST_CHECK_MESSAGE(std::fabs(magnitude(po8->get_torque() - point_t(-0.641536f, -0.0775413f, -0.00994667f))) < result_tolerance, po8->get_torque());


    /* Resolve forces and check vgs */
    po5->set_force( point_t(0.0f,  6.0f, -0.5f));
    po6->set_force( point_t(0.0f,  0.0f,  1.0f));
    po7->set_force( point_t(0.0f,  0.0f,  1.0f));
    po8->set_force( point_t(0.0f, -4.0f,  1.0f));
    po5->set_torque(point_t(0.0f,  2.0f,  0.0f));
    po6->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po7->set_torque(point_t(0.0f, -1.0f,  0.0f));
    po8->set_torque(point_t(0.0f,  0.0f,  0.0f));

    po0->set_force( point_t(0.0f, 0.0f, 1.0f));
    po1->set_force( point_t(0.0f, 0.0f, 1.1f));
    po2->set_force( point_t(0.0f, 0.0f, 1.2f));
    po3->set_force( point_t(0.0f, 0.0f, 1.3f));
    po4->set_force( point_t(0.0f, 0.0f, 1.4f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 1.1f, 0.0f));
    po2->set_torque(point_t(0.0f, 1.2f, 0.0f));
    po3->set_torque(point_t(0.0f, 1.3f, 0.0f));
    po4->set_torque(point_t(0.0f, 1.4f, 0.0f));
    
    /* Rebuild, but from po6 */
    uut->rebuild(&static_friction, &default_collider, info, po6);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.395618f,    5.20877f,     0.51797f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.465714f,    0.791234f,    0.682986f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0767641f,  -0.0088913f,   0.276815f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(-0.00666849f, -3.99111f,     1.02223f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.410662f,    0.0774262f,  -1.33297f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.821328f,   -0.0060486f,   0.576788f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.330473f,   -0.00302424f, -0.0183765f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.00577933f,  0.0237843f,  -0.00777989f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_force()  - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_force()  - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_force()  - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_torque() - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_torque() - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_torque() - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_force()  - point_t( 0.395618f,    5.20877f,     0.51797f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_force()  - point_t(-0.465714f,    0.791234f,    0.682986f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_force()  - point_t( 0.0767641f,  -0.0088913f,   0.276815f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_force()  - point_t(-0.00666849f, -3.99111f,     1.02223f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_torque() - point_t(-0.410662f,    0.0774262f,  -1.33297f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_torque() - point_t(-0.821328f,   -0.0060486f,   0.576788f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_torque() - point_t(-0.330473f,   -0.00302424f, -0.0183765f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_torque() - point_t( 0.00577933f,  0.0237843f,  -0.00777989f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    po0->set_force(point_t(0.0f, 0.0f, -5.0f));
    po1->set_force(point_t(0.0f, 0.0f,  11.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894427f,  0.447214f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.596286f,  1.19257f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894427f, -0.447214f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.298142f,  0.596283f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894427f,  0.447214f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.596286f,  1.19257f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894427f, -0.447214f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.298142f,  0.596283f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_force_halting_tanjent_velocity_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,       0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.284445f,  0.0f,     0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      -0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.142222f,  0.0f,     0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,       0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.284445f,  0.0f,     0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      -0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.142222f,  0.0f,     0.0f))) < result_tolerance);


    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po1);
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.64f,      0.32f,     -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.426667f,  0.853334f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.64f,     -0.32f,     -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.213333f,  0.426665f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.64f,      0.32f,     -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.426667f,  0.853334f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.64f,     -0.32f,     -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.213333f,  0.426665f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_force_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -1.0f, -1.0f));
    po1->set_force(point_t(0.0f,  1.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     -0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.533334f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.266666f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     -0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.533334f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.266666f, 0.0f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po0->set_force(point_t( 7.0f, -1.0f, -4.0f));
    po1->set_force(point_t( 1.0f,  2.0f,  1.0f));
    po0->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po1->set_torque(point_t(0.0f,  0.0f,  0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.33333f, -0.333334f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.888889f, 0.888889f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.66667f,  1.33333f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.444443f, 0.444443f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.33333f, -0.333334f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.888889f, 0.888889f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.66667f,  1.33333f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.444443f, 0.444443f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_force_starting_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -3.0f, -1.0f));
    po1->set_force(point_t(0.0f,  4.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      -2.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.666667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,       3.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333333f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      -2.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.666667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,       3.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333333f,  0.0f, 0.0f))) < result_tolerance);


    /* Resolve forces and check vgs */
    po0->set_force(point_t(7.0f, -1.0f, -2.0f));
    po1->set_force(point_t(1.0f,  2.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.5286f,  -0.528595f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.62854f,  0.62854f,   0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.4714f,   1.5286f,   -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.314269f, 0.314269f,  0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.5286f,  -0.528595f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.62854f,  0.62854f,   0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.4714f,   1.5286f,   -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.314269f, 0.314269f,  0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotation_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force( point_t( 0.0f, 0.0f, -1.0f));
    po0->set_torque(point_t(-1.0f, 0.0f, -1.0f));
    po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
    po1->set_torque(point_t( 0.0f, 2.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force( point_t( 0.0f, 0.0f, -4.0f));
    po0->set_torque(point_t(-3.0f, 0.0f, -1.0f));
    po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
    po1->set_torque(point_t( 0.0f, 1.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotation_force_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f,  1.0f, -3.0f));
    po0->set_torque(point_t(-1.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, -1.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  2.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.589342f,  0.589341f, -1.16776f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.21421f,   0.326779f, -0.761757f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.589342f, -0.589341f, -0.832241f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.607105f,  0.494537f,  0.761757f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.589342f,  0.589341f, -1.16776f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.21421f,   0.326779f, -0.761757f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.589342f, -0.589341f, -0.832241f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.607105f,  0.494537f,  0.761757f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  7.0f, -1.0f, -4.0f));
    po0->set_torque(point_t(-3.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  1.0f,  2.0f,  3.0f));
    po1->set_torque(point_t( 0.0f,  1.0f,  1.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.5015f,   0.0228381f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.636217f, 1.33134f,   -0.869385f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.4985f,   0.977162f,  -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.318107f, 0.665665f,   0.869385f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.5015f,   0.0228381f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.636217f, 1.33134f,   -0.869385f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.4985f,   0.977162f,  -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.318107f, 0.665665f,   0.869385f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotation_force_starting_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f,  1.0f, -1.0f));
    po0->set_torque(point_t(-1.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, -1.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  2.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.529257f,  0.590497f,  0.358318f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.21267f,   0.149902f, -0.840329f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.529257f, -0.590497f, -0.358318f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.606334f,  0.791584f,  0.840329f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.529257f,  0.590497f,  0.358318f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.21267f,   0.149902f, -0.840329f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.529257f, -0.590497f, -0.358318f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.606334f,  0.791584f,  0.840329f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  7.0f, -1.0f, -4.0f));
    po0->set_torque(point_t(-3.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  1.0f,  2.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  1.0f,  1.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.47089f, -0.0536445f, -1.82794f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.967612f, 1.37215f,   -1.09437f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.52911f,  1.05364f,   -1.17206f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.139677f, 0.686072f,   1.09437f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.47089f, -0.0536445f, -1.82794f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.967612f, 1.37215f,   -1.09437f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.52911f,  1.05364f,   -1.17206f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.139677f, 0.686072f,   1.09437f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    po0->set_force(point_t(0.0f, 0.0f, -5.0f));
    po1->set_force(point_t(0.0f, 0.0f, 11.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_force_tanjent_velocity_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,    0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,   -0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333f,  0.0f, 0.0f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894428f,  0.447214f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.596286f,  1.19257f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894428f, -0.447214f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.298142f,  0.596283f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.894428f,  0.447214f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.596286f,  1.19257f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.894428f, -0.447214f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.298142f,  0.596283f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_force_halting_tanjent_velocity_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po0);
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,       0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.284445f,  0.0f,     0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      -0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.142222f,  0.0f,     0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,       0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.284445f,  0.0f,     0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      -0.213333, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.142222f,  0.0f,     0.0f))) < result_tolerance);


    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -4.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(7.0f, -1.0f, 0.0f));
    po1->set_velocity(point_t(1.0f,  2.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&dynamic_friction, &default_collider, info, po1);
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.64f,      0.32f,     -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.426667f,  0.853334f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.64f,     -0.32f,     -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.213333f,  0.426665f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(5.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(-0.64f,      0.32f,     -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t( 0.426667f,  0.853334f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.64f,     -0.32f,     -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t( 0.213333f,  0.426665f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_force_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -1.0f, -1.0f));
    po1->set_force(point_t(0.0f,  1.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     -0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.533334f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.266666f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     -0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.533334f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.6f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.266666f, 0.0f, 0.0f))) < result_tolerance);
    

    /* Resolve forces and check vgs */
    po0->set_force(point_t( 7.0f, -1.0f, -4.0f));
    po1->set_force(point_t( 1.0f,  2.0f,  1.0f));
    po0->set_torque(point_t(0.0f,  0.0f,  0.0f));
    po1->set_torque(point_t(0.0f,  0.0f,  0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.33333f, -0.333334f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.888889f, 0.888889f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.66667f,  1.33333f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.444443f, 0.444443f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.33333f, -0.333334f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.888889f, 0.888889f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.66667f,  1.33333f,  -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.444443f, 0.444443f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_force_starting_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, -3.0f, -1.0f));
    po1->set_force(point_t(0.0f,  4.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      -2.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.666667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,       3.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333333f,  0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      -2.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.666667f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,       3.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.333333f,  0.0f, 0.0f))) < result_tolerance);


    /* Resolve forces and check vgs */
    po0->set_force(point_t(7.0f, -1.0f, -2.0f));
    po1->set_force(point_t(1.0f,  2.0f,  1.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.5286f,  -0.528595f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.62854f,  0.62854f,   0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.4714f,   1.5286f,   -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.314269f, 0.314269f,  0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(6.5286f,  -0.528595f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.62854f,  0.62854f,   0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(1.4714f,   1.5286f,   -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.314269f, 0.314269f,  0.0f     ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotation_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force( point_t( 0.0f, 0.0f, -1.0f));
    po0->set_torque(point_t(-1.0f, 0.0f, -1.0f));
    po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
    po1->set_torque(point_t( 0.0f, 2.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,     0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.6666f, 0.7272f, -1.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -0.4545f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.3333f, 1.2727f,  1.0f   ))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force( point_t( 0.0f, 0.0f, -4.0f));
    po0->set_torque(point_t(-3.0f, 0.0f, -1.0f));
    po1->set_force( point_t( 0.0f, 0.0f,  1.0f));
    po1->set_torque(point_t( 0.0f, 1.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,    0.0f,    -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-2.0f,    0.6666f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,    0.0f,    -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.9999f, 0.3333f,  1.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotation_force_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f,  1.0f, -3.0f));
    po0->set_torque(point_t(-1.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, -1.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  2.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.603763f,  0.641119f, -1.17038f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.14518f,   0.311046f, -0.116164f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.603763f, -0.641119f, -0.829619f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.572586f,  0.481427f,  0.116164f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.603763f,  0.641119f, -1.17038f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.14518f,   0.311046f, -0.116164f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.603763f, -0.641119f, -0.829619f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.572586f,  0.481427f,  0.116164f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  7.0f, -1.0f, -4.0f));
    po0->set_torque(point_t(-3.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  1.0f,  2.0f,  3.0f));
    po1->set_torque(point_t( 0.0f,  1.0f,  1.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.72109f,  0.223578f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.368563f, 1.03855f,  -0.15021f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.27891f,  0.776422f, -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.184281f, 0.519271f,  0.15021f ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.72109f,  0.223578f, -0.666667f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.368563f, 1.03855f,  -0.15021f ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.27891f,  0.776422f, -0.333333f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.184281f, 0.519271f,  0.15021f ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotation_force_starting_tanjent_force_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  0.0f,  1.0f, -1.0f));
    po0->set_torque(point_t(-1.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  0.0f, -1.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  2.0f,  1.0f));
    uut->rebuild(&static_friction, &default_collider, info, po0);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.413529f,  0.720982f,  0.379359f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.03869f,   0.276151f, -0.219719f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.413529f, -0.720982f, -0.379359f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.519344f,  0.896792f,  0.219719f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.413529f,  0.720982f,  0.379359f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-1.03869f,   0.276151f, -0.219719f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(-0.413529f, -0.720982f, -0.379359f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.519344f,  0.896792f,  0.219719f))) < result_tolerance);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(  7.0f, -1.0f, -4.0f));
    po0->set_torque(point_t(-3.0f,  0.0f, -1.0f));
    po1->set_force(point_t(  1.0f,  2.0f,  1.0f));
    po1->set_torque(point_t( 0.0f,  1.0f,  1.0f));

    /* Rebuild, but from po1 */
    uut->rebuild(&static_friction, &default_collider, info, po1);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.77269f, -0.161238f, -2.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.881652f, 0.969752f, -0.427351f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.22731f,  1.16124f,  -1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.440825f, 0.484874f,  0.427351f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 6.77269f, -0.161238f, -2.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.881652f, 0.969752f, -0.427351f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 1.22731f,  1.16124f,  -1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.440825f, 0.484874f,  0.427351f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_linear_force_resting_impulse_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po1->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f    ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f    ))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0f, 0.0f, -4.0f));
    po1->set_velocity(point_t(0.0f, 0.0f,  2.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    
    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t(0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t(0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_rotation_force_resting_implulse_test, contact_graph_2_contact_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po0->set_angular_velocity(point_t(-1.0f, 0.0f, -1.0f));
    po1->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po1->set_angular_velocity(point_t(0.0f, 2.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-1.0f, 0.0f, -1.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 2.0f,  1.0f    ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-1.0f, 0.0f, -1.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -0.333333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 2.0f,  1.0f    ))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0f, 0.0f, -4.0f));
    po0->set_angular_velocity(point_t(-3.0f, 0.0f, -1.0f));
    po1->set_velocity(point_t(0.0f, 0.0f,  2.0f));
    po1->set_angular_velocity(point_t(0.0f, 1.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-3.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 1.0f,  1.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity() - point_t(-3.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()         - point_t( 0.0f, 0.0f, -2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity() - point_t( 0.0f, 1.0f,  1.0f))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_non_perp_resting_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0f, 2.0f, -1.0f));
    po10->set_angular_velocity(point_t(-3.0f, 0.0f, 0.5f));
    po11->set_velocity(point_t(3.0f, 0.0f,  1.0f));
    po11->set_angular_velocity(point_t(0.0f, 0.7, 2.0f));
    uut->resolve_forces(1.0f);

    /* Shows only forces and torques that cause the relative acceleration to increase matter */
    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  2.0f, -0.733333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(-3.0f, -0.4f,  0.5f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 3.0f,  0.0f,  0.466666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,   0.7,   2.0f    ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  2.0f, -0.733333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(-3.0f, -0.4f,  0.5f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 3.0f,  0.0f,  0.466666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,   0.7,   2.0f    ))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_force_resting_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.733333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f, -0.4f,  0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f,  0.466666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,   0.0f,  0.0f    ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.733333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f, -0.4f,  0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f,  0.466666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 0.8,   0.0f,  0.0f    ))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po11);

    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0f, 0.0f, -4.0f));
    po10->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
    po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po11->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.33333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -1.0f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.33333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(2.0f,  0.0f,  0.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.33333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -1.0f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.33333))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(2.0f,  0.0f,  0.0f   ))) < result_tolerance);
}


BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_force_resting_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po10->set_angular_velocity(point_t(0.0f, 3.0f, 0.0f));
    po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po11->set_angular_velocity(point_t(1.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.4666666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f,  2.2f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f, -0.06666  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 2.6,   0.0f,  0.0f     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t( 0.0f,  0.0f, -0.4666666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t( 0.0f,  2.2f,  0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t( 0.0f,  0.0f, -0.06666  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t( 2.6,   0.0f,  0.0f     ))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po11);

    /* Resolve forces and check vgs */
    po10->set_velocity(point_t(0.0f, 0.0f, -4.0f));
    po10->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
    po11->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po11->set_angular_velocity(point_t(-1.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.06666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -0.4f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.86666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(1.8,   0.0f,  0.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_velocity()         - point_t(0.0f,  0.0f, -3.06666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_angular_velocity() - point_t(0.0f, -0.4f,  0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_velocity()         - point_t(0.0f,  0.0f, -0.86666))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_angular_velocity() - point_t(1.8,   0.0f,  0.0f   ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_9_disjoint_linear_force_resting_impulse_test, contact_graph_9_disjoint_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0f, 0.0f, -0.5f));
    po1->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po2->set_velocity(point_t(0.0f, 0.0f,  0.0f));
    po3->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po4->set_velocity(point_t(0.0f, 0.0f,  1.0f));

    po5->set_velocity(point_t(0.0f, 0.0f, 1.1f));
    po6->set_velocity(point_t(0.0f, 0.0f, 1.2f));
    po7->set_velocity(point_t(0.0f, 0.0f, 1.3f));
    po8->set_velocity(point_t(0.0f, 0.0f, 1.4f));
    po5->set_angular_velocity(point_t(0.0f, 1.1f, 0.0f));
    po6->set_angular_velocity(point_t(0.0f, 1.2f, 0.0f));
    po7->set_angular_velocity(point_t(0.0f, 1.3f, 0.0f));
    po8->set_angular_velocity(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Resolve forces and check vgs */
    po0->set_velocity(point_t(0.0f, 0.0f,  2.0f));
    po1->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po2->set_velocity(point_t(0.0f, 0.0f,  0.0f));
    po3->set_velocity(point_t(0.0f, 0.0f, -1.0f));
    po4->set_velocity(point_t(0.0f, 0.0f, -1.0f));

    po5->set_velocity(point_t(0.0f, 0.0f, 1.1f));
    po6->set_velocity(point_t(0.0f, 0.0f, 1.2f));
    po7->set_velocity(point_t(0.0f, 0.0f, 1.3f));
    po8->set_velocity(point_t(0.0f, 0.0f, 1.4f));
    po5->set_angular_velocity(point_t(0.0f, 1.1f, 0.0f));
    po6->set_angular_velocity(point_t(0.0f, 1.2f, 0.0f));
    po7->set_angular_velocity(point_t(0.0f, 1.3f, 0.0f));
    po8->set_angular_velocity(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f,  2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f,  2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);
    
    /* Rebuild, but from po6 */
    uut->rebuild(&no_friction, &default_collider, info, po6);

    /* Resolve forces and check vgs */
    po5->set_velocity(point_t(0.0f, 0.0f, -4.5f));
    po6->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po7->set_velocity(point_t(0.0f, 0.0f,  2.0f));
    po8->set_velocity(point_t(0.0f, 0.0f,  1.0f));
    po5->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));
    po6->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));
    po7->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));
    po8->set_angular_velocity(point_t(0.0f, 0.0f,  0.0f));

    po0->set_velocity(point_t(0.0f, 0.0f, 1.0f));
    po1->set_velocity(point_t(0.0f, 0.0f, 1.1f));
    po2->set_velocity(point_t(0.0f, 0.0f, 1.2f));
    po3->set_velocity(point_t(0.0f, 0.0f, 1.3f));
    po4->set_velocity(point_t(0.0f, 0.0f, 1.4f));
    po0->set_angular_velocity(point_t(0.0f, 1.0f, 0.0f));
    po1->set_angular_velocity(point_t(0.0f, 1.1f, 0.0f));
    po2->set_angular_velocity(point_t(0.0f, 1.2f, 0.0f));
    po3->set_angular_velocity(point_t(0.0f, 1.3f, 0.0f));
    po4->set_angular_velocity(point_t(0.0f, 1.4f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 1.1f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_velocity()          - point_t(0.0f, 0.0f, 1.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_velocity()          - point_t(0.0f, 0.0f, 1.3f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_velocity()          - point_t(0.0f, 0.0f, 1.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 1.1f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po2->get_angular_velocity()  - point_t(0.0f, 1.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po3->get_angular_velocity()  - point_t(0.0f, 1.3f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po4->get_angular_velocity()  - point_t(0.0f, 1.4f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po5->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_velocity()          - point_t(0.0f, 0.0f, 0.25f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po5->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po6->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po7->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po8->get_angular_velocity()  - point_t(0.0f, 0.0f,  0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_resting_force_test, contact_graph_2_infinite_mass_contact_fixture )
{
    /* Resolve forces and check vgs */
    po12->set_force(point_t(0.0f,   0.0f, -1000.0f));
    po13->set_force(point_t(0.0f,   0.0f,     0.0f));
    po12->set_torque(point_t(0.0f, 250.0f,    0.0f));
    po13->set_torque(point_t(0.0f,   0.0f,    0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t(0.0f,    0.0f, -450.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(0.0f, -300.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t(0.0f,    0.0f, -450.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(0.0f, -300.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t(0.0f,    0.0f,    0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_tanjent_velocity_resting_force_test, contact_graph_2_infinite_mass_contact_fixture )
{
    /* Resolve forces and check vgs */
    po12->set_force(point_t(   0.0f,   0.0f, -1000.0f));
    po13->set_force(point_t(   0.0f,   0.0f,     0.0f));
    po12->set_torque(point_t(  0.0f, 250.0f,     0.0f));
    po13->set_torque(point_t(  0.0f,   0.0f,     0.0f));
    po12->set_velocity(point_t(0.0f, 150.0f,     0.0f));
    po13->set_velocity(point_t(0.0f,   0.0f,     0.0f));
    uut->rebuild(&dynamic_friction, &default_collider, info, po12);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t( 0.0f,      -55.0f, -450.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(-2.75001f, -300.0f,  -55.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t( 0.0f,        0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t( 0.0f,        0.0f,    0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t( 0.0f,      -55.0f, -450.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(-2.75001f, -300.0f,  -55.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t( 0.0f,        0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t( 0.0f,        0.0f,    0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_tanjent_force_resting_force_test, contact_graph_2_infinite_mass_contact_fixture )
{
    /* Resolve forces and check vgs */
    po12->set_force(point_t( 0.0f, 100.0f, -1000.0f));
    po13->set_force(point_t( 0.0f,   0.0f,     0.0f));
    po12->set_torque(point_t(0.0f, 250.0f,     0.0f));
    po13->set_torque(point_t(0.0f,   0.0f,     0.0f));
    uut->rebuild(&static_friction, &default_collider, info, po12);
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t( 7.66777f,   59.1052, -449.77f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(-2.04474f, -300.614,   -40.8948f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t( 0.0f,        0.0f,      0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t( 0.0f,        0.0f,      0.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_force()  - point_t( 7.66777f,   59.1052, -449.77f  ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_torque() - point_t(-2.04474f, -300.614,   -40.8948f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_force()  - point_t( 0.0f,        0.0f,      0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_torque() - point_t( 0.0f,        0.0f,      0.0f   ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_infinite_mass_resting_impulse_test, contact_graph_2_infinite_mass_contact_fixture )
{   
    po12->set_velocity(point_t(0.0f, 0.0f, -4.0f));
    po13->set_velocity(point_t(0.0f, 0.0f,  0.0f));
    po12->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
    po13->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f,  0.0f, -2.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, -2.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);
 
    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f,  0.0f, -2.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, -2.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);

    /* Rebuild, but from po12 */
    uut->rebuild(&no_friction, &default_collider, info, po12);
    
    po12->set_velocity(point_t(0.0f, 0.0f, -4.0f));
    po13->set_velocity(point_t(0.0f, 0.0f,  0.0f));
    po12->set_angular_velocity(point_t(0.0f, 6.0f, 0.0f));
    po13->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
 
    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po12->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po12->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_velocity()         - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po13->get_angular_velocity() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_resting_force_and_impulse_test, contact_graph_2_contact_fixture )
{
    /* No change for joint movement and movemnt in normal */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
    po1->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
    (*info[po0])[po1]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
    (*info[po1])[po0]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    
    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);
    
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -160.0f));
    po1->set_force(point_t(0.0f, 0.0f,   80.0f));
    po0->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -164.0f));
    po1->set_force(point_t(0.0f, 0.0f,   81.0f));
    po0->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    po1->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -162.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   79.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -162.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f,   79.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_linear_resting_force_and_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
    po10->set_force(point_t(0.0f, 0.0f, -1.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    po10->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
    po11->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -0.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  0.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,  0.0f))) < result_tolerance);

    /* Rebuild, but from po10 */
    uut->rebuild(&no_friction, &default_collider, info, po10);

    po10->set_force(point_t(0.0f, 0.0f, -1.0f));
    po11->set_force(point_t(0.0f, 0.0f,  1.0f));
    po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po10->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
    po11->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,       0.0f,      -0.601897))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.398103,  0.0     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,       0.0f,       0.601897))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.398103,  0.0f,       0.0     ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,       0.0f,      -0.601897))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f,      -0.398103,  0.0     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,       0.0f,       0.601897))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.398103,  0.0f,       0.0     ))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -160.0f));
    po11->set_force(point_t(0.0f, 0.0f,   80.0f));
    po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po10->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    po11->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    po10->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
    po11->set_angular_velocity(point_t(0.0f, 0.0f, 0.0f));
    (*info[po10])[po11]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
    (*info[po11])[po10]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(0.0f, 20.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.0f, 0.0f,    0.0f))) < result_tolerance);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -164.0f));
    po10->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po11->set_force(point_t(0.0f, 0.0f,   81.0f));
    po11->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po10->set_velocity(point_t(0.0f,  1.0f, 0.0f));
    po11->set_velocity(point_t(0.0f, -1.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -163.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,   80.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0f,    0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -163.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.8,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,   80.2f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.8,  0.0f,    0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_offset_rotational_resting_force_and_impulse_test, contact_graph_2_offset_contact_fixture )
{
    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -161.0f));
    po10->set_torque(point_t(0.0f, 3.0f, 0.0f));
    po11->set_force(point_t(0.0f, 0.0f,  81.0f));
    po11->set_torque(point_t(1.0f, 0.0f, 0.0f));
    po10->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
    po11->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
    (*info[po10])[po11]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(20.0f, 0.0f, 0.0f));
    (*info[po11])[po10]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(20.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0f,    0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f, 0.0f, -160.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, 2.4f,    0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f, 0.0f,   80.4f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(1.6, 0.0f,    0.0f))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po11);

    /* Resolve forces and check vgs */
    po10->set_force(point_t(0.0f, 0.0f, -84.0f));
    po10->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po11->set_force(point_t(0.0f, 0.0f,  41.0f));
    po11->set_torque(point_t(-1.0f, 0.0f, 0.0f));
    po10->set_velocity(point_t(0.0f, 0.0f, 0.0f));
    po11->set_velocity(point_t(0.0f, 0.0f, 0.0f));
    po10->set_angular_velocity(point_t(0.0f, 0.0f, -1.0f));
    po11->set_angular_velocity(point_t(0.0f, 0.0f,  1.0f));
    (*info[po10])[po11]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-10.0f, -10.0f, 0.0f));
    (*info[po11])[po10]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-10.0f, -10.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -82.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  39.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,   0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po10->get_force()  - point_t(0.0f,  0.0f, -82.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po10->get_torque() - point_t(0.0f, -0.4f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_force()  - point_t(0.0f,  0.0f,  39.6))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po11->get_torque() - point_t(0.4f,  0.0f,   0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_linear_resting_force_and_impulse_test, contact_graph_2_contact_4_point_fixture )
{
    /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
    po1->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    po0->set_force(point_t(0.0f, 0.0f, -5.0f));
    po1->set_force(point_t(0.0f, 0.0f,  11.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
    po1->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotational_resting_force_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, 0.0f));
    po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(6.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     0.0f,       2.18182f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(1.09091f, 2.0f,       0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,     0.0f,      -2.18182f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(4.90909f, 0.999997f,  0.0f    ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,     0.0f,       2.18182f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(1.09091f, 2.0f,       0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,     0.0f,      -2.18182f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(4.90909f, 0.999997f,  0.0f    ))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -160.0f));
    po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f,  70.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 10.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_rotational_resting_force_and_impulse_test, contact_graph_2_contact_4_point_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -160.0f));
    po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f,  70.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 10.0f));
    po0->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
    po1->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -84.0f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f,  41.0f));
    po1->set_torque(point_t(-1.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(0.0f, 0.0f, 0.0f));
    po1->set_velocity(point_t(0.0f, 0.0f, 0.0f));
    po0->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
    (*info[po0])[po1]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
    (*info[po1])[po0]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_linear_resting_force_and_impulse_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* No change for joint movement, but collision in normal add angular rotation and slight change in forces */
    po0->set_force(point_t(0.0f, 0.0f, -1.0f));
    po1->set_force(point_t(0.0f, 0.0f,  1.0f));
    po0->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
    po1->set_velocity(point_t(-1.0f, 1.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    po0->set_force(point_t(0.0f, 0.0f, -5.0f));
    po1->set_force(point_t(0.0f, 0.0f,  11.0f));
    po0->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(-1.0f, 1.0f, -1.0f));
    po1->set_velocity(point_t(-1.0f, 1.0f,  1.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(-1.0f, 1.0f, -0.333333)))   < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f)))          < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, 4.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, 2.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotational_resting_force_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, 0.0f));
    po0->set_torque(point_t(0.0f, -3.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f, 0.0f));
    po1->set_torque(point_t(6.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      0.0f,       1.10127f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(3.85443f, -2.44937f,   0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.0f,      -1.10127f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.14557f, -0.550634f,  0.0f    ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,      0.0f,       1.10127f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(3.85443f, -2.44937f,   0.0f    ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,      0.0f,      -1.10127f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.14557f, -0.550634f,  0.0f    ))) < result_tolerance);

    /* Rebuild, but from po11 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -160.0f));
    po0->set_torque(point_t(0.0f, -3.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f,  70.0f));
    po1->set_torque(point_t(6.0f, 0.0f, 10.0f));
    po0->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
    po1->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,  0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(4.0f, -2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,  0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.0f, -1.0f,  10.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f,  0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(4.0f, -2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f,  0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(2.0f, -1.0f,  10.0f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( contact_graph_2_contact_4_point_asym_rotational_resting_force_and_impulse_test, contact_graph_2_contact_4_point_asym_fixture )
{
    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -160.0f));
    po0->set_torque(point_t(0.0f, 3.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f,  70.0f));
    po1->set_torque(point_t(0.0f, 0.0f, 10.0f));
    po0->set_velocity(point_t(-1.0f, 0.0f, 0.0f));
    po1->set_velocity(point_t( 1.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_velocity()          - point_t(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_velocity()          - point_t( 1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_angular_velocity()  - point_t( 0.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t(0.0f, 0.0f, -60.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(0.0f, 2.0f,   0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t(0.0f, 0.0f, -30.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(0.0f, 1.0f,  10.0f))) < result_tolerance);

    /*  Rebuild, but from po1 */
    uut->rebuild(&no_friction, &default_collider, info, po1);

    /* Resolve forces and check vgs */
    po0->set_force(point_t(0.0f, 0.0f, -84.0f));
    po0->set_torque(point_t(0.0f, 1.0f, 0.0f));
    po1->set_force(point_t(0.0f, 0.0f,  41.0f));
    po1->set_torque(point_t(-1.0f, 0.0f, 0.0f));
    po0->set_velocity(point_t(0.0f, 0.0f, 0.0f));
    po1->set_velocity(point_t(0.0f, 0.0f, 0.0f));
    po0->set_angular_velocity(point_t(0.0f, -1.0f, 0.0f));
    po1->set_angular_velocity(point_t(0.0f,  1.0f, 0.0f));
    (*info[po0])[po1]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
    (*info[po1])[po0]->get_simplex()->set_rate_of_change_of_normal_of_impact(point_t(-1.0f, 0.0f, 0.0f));
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);

    /* Re run and check no change */
    uut->resolve_forces(1.0f);

    BOOST_CHECK(std::fabs(magnitude(po0->get_force()  - point_t( 0.0f,      0.0f,      -29.6143f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po0->get_torque() - point_t(-0.666665f, 0.666668f,   0.0f   ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_force()  - point_t( 0.0f,      0.0f,      -13.3857f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(po1->get_torque() - point_t(-0.333335f, 0.333333f,   0.0f   ))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
