#ifdef STAND_ALONE
#define BOOST_TEST_MODULE inertia_tensor test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Common headers */
#include "quaternion_t.h"

/* Physics headers */
#include "inertia_tensor.h"


namespace raptor_physics
{
namespace test
{
BOOST_AUTO_TEST_SUITE( inertia_tensor_tests )

const float result_tolerance = 0.0005;


/* Test ctor for shapes with known tensor */
BOOST_AUTO_TEST_CASE( basic_test )
{
    const float m = 1.5;
    const point_t<> com(3.1, 4.7, 9.3);
    float *const it = new float [6] { 9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0 };
    const float it_inv[6] = { 0.223, 0.338, 0.259, -0.158, -0.151, -0.014 };
    
    inertia_tensor uut(it, com, m);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


BOOST_AUTO_TEST_CASE( inifinite_mass_basic_test )
{
    const float m = std::numeric_limits<float>::infinity();
    const point_t<> com(30.1, -4.7, 5.3);
    float *const it = new float [6] { m, m, m, m, m, m };
    const float it_inv[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
    
    inertia_tensor uut(it, com, m);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


BOOST_AUTO_TEST_CASE( zero_mass_basic_test )
{
    const float m = 0.0;
    const point_t<> com(10.1, 14.7, -1.3);
    float *const it = new float [6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
    const float it_inv[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
    
    inertia_tensor uut(it, com, m);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


/* Copy ctor */
BOOST_AUTO_TEST_CASE( copy_test )
{
    const float m = 1.5;
    const point_t<> com(3.1, 4.7, 9.3);
    float *const it = new float [6] { 9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0 };
    const float it_inv[6] = { 0.223, 0.338, 0.259, -0.158, -0.151, -0.014 };
    
    inertia_tensor copy(it, com, m);
    inertia_tensor uut(copy);
    BOOST_CHECK(std::equal(&it[0], &it[6], copy.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], copy.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(copy.center_of_mass() == com);
    BOOST_CHECK(copy.mass() == m);

    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


/* Move ctor */
BOOST_AUTO_TEST_CASE( move_test )
{
    const float m = 1.5;
    const point_t<> com(3.1, 4.7, 9.3);
    float *const it = new float [6] { 9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0 };
    const float it_inv[6] = { 0.223, 0.338, 0.259, -0.158, -0.151, -0.014 };
    
    inertia_tensor move(it, com, m);
    inertia_tensor uut(std::move(move));
    BOOST_CHECK(move.tensor() == nullptr);
    BOOST_CHECK(move.inverse_tensor() == nullptr);
    BOOST_CHECK(move.center_of_mass() == com);
    BOOST_CHECK(move.mass() == m);

    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


/* Test moving the center o mass */
BOOST_AUTO_TEST_CASE( move_com_test )
{
    const float m = 1.5;
    const point_t<> com0(3.1, 4.7, 9.3);
    float *const it = new float [6] { 9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0 };
    const float it_inv[6] = { 0.223, 0.338, 0.259, -0.158, -0.151, -0.014 };

    inertia_tensor uut(it, com0, m);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com0);
    BOOST_CHECK(uut.mass() == m);

    const point_t<> com1(4.2, 7.2, -0.2);

    uut.move_center_of_mass(point_t<>(1.1, 2.5, -9.5));
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(magnitude(uut.center_of_mass() - com1) < result_tolerance); /* These numbers match to within a numerical precision */
    BOOST_CHECK(uut.mass() == m);
}


/* Test multiply by inverse tensor */
BOOST_AUTO_TEST_CASE( multiply_divide_test )
{
    const float m = 1.5;
    const point_t<> com0(3.1, 4.7, 9.3);
    float *const it = new float [6] { 9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0 };
    const float it_inv[6] = { 0.223, 0.338, 0.259, -0.158, -0.151, -0.014 };

    inertia_tensor uut(it, com0, m);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com0);
    BOOST_CHECK(uut.mass() == m);

    const point_t<> vec(9.7, 8.8, 6.4);
    const point_t<> div(vec / uut);
    const point_t<> exp_div(0.678417, 0.473381, 0.188489);
    BOOST_CHECK(magnitude(div - exp_div) < result_tolerance);

    const point_t<> mul(uut * vec);
    const point_t<> exp_mul(165.7, 160.6, 127.6);
    BOOST_CHECK(magnitude(mul - exp_mul) < result_tolerance);
}


/* Test ctor for shapes with unknown tensors */
BOOST_AUTO_TEST_CASE( infinite_mass_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(-1.0f, -1.0f, -1.0f), 
            point_t<>( 1.0f, -1.0f, -1.0f),
            point_t<>( 1.0f,  1.0f, -1.0f),
            point_t<>(-1.0f,  1.0f, -1.0f),
            point_t<>(-1.0f, -1.0f,  1.0f), 
            point_t<>( 1.0f, -1.0f,  1.0f),
            point_t<>( 1.0f,  1.0f,  1.0f),
            point_t<>(-1.0f,  1.0f,  1.0f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4  /* Bottom face */
        });

    const float m = std::numeric_limits<float>::infinity();
    const point_t<> com(0.0f, 0.0f, 0.0f);
    const float tensor = std::numeric_limits<float>::infinity();
    const float it[6] = { tensor, tensor, tensor, tensor, tensor, tensor };
    const float it_inv[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };

    inertia_tensor uut(p, e, std::numeric_limits<float>::infinity());
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor()));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}



BOOST_AUTO_TEST_CASE( zero_mass_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(-1.0f, -1.0f, -1.0f), 
            point_t<>( 1.0f, -1.0f, -1.0f),
            point_t<>( 1.0f,  1.0f, -1.0f),
            point_t<>(-1.0f,  1.0f, -1.0f),
            point_t<>(-1.0f, -1.0f,  1.0f), 
            point_t<>( 1.0f, -1.0f,  1.0f),
            point_t<>( 1.0f,  1.0f,  1.0f),
            point_t<>(-1.0f,  1.0f,  1.0f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4  /* Bottom face */
        });

    const float m = 0.0;
    const point_t<> com(0.0f, 0.0f, 0.0f);
    const float it[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
    const float it_inv[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };

    inertia_tensor uut(p, e, 0.0f);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor()));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


BOOST_AUTO_TEST_CASE( origin_cube_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(-1.0f, -1.0f, -1.0f), 
            point_t<>( 1.0f, -1.0f, -1.0f),
            point_t<>( 1.0f,  1.0f, -1.0f),
            point_t<>(-1.0f,  1.0f, -1.0f),
            point_t<>(-1.0f, -1.0f,  1.0f), 
            point_t<>( 1.0f, -1.0f,  1.0f),
            point_t<>( 1.0f,  1.0f,  1.0f),
            point_t<>(-1.0f,  1.0f,  1.0f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4 /* Bottom face */
        });

    const float m = 8.0;
    const point_t<> com(0.0f, 0.0f, 0.0f);
    const float tensor = (8.0 / 12.0f) * m;
    const float it[6] = { tensor, tensor, tensor, 0.0f, 0.0f, 0.0 };
    const float tensor_inv = 0.1875;
    const float it_inv[6] = { tensor_inv, tensor_inv, tensor_inv, 0.0f, 0.0f, 0.0 };

    inertia_tensor uut(p, e);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


BOOST_AUTO_TEST_CASE( cube_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(3.0f, 2.0f, 4.0f), 
            point_t<>(6.0f, 2.0f, 4.0f),
            point_t<>(6.0f, 5.0f, 4.0f),
            point_t<>(3.0f, 5.0f, 4.0f),
            point_t<>(3.0f, 2.0f, 7.0f), 
            point_t<>(6.0f, 2.0f, 7.0f),
            point_t<>(6.0f, 5.0f, 7.0f),
            point_t<>(3.0f, 5.0f, 7.0f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4 /* Bottom face */
        });

    const float m = 27.0;
    const point_t<> com(4.5, 3.5, 5.5);
    const float tensor = (18.0 / 12.0f) * m;
    const float it[6] = { tensor, tensor, tensor, 0.0f, 0.0f, 0.0 };
    const float tensor_inv = 0.02469;
    const float it_inv[6] = { tensor_inv, tensor_inv, tensor_inv, 0.0f, 0.0f, 0.0 };

    inertia_tensor uut(p, e);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


BOOST_AUTO_TEST_CASE( cuboid_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(3.0f, 1.0f, 3.0f), 
            point_t<>(6.0f, 1.0f, 3.0f),
            point_t<>(6.0f, 5.0f, 3.0f),
            point_t<>(3.0f, 5.0f, 3.0f),
            point_t<>(3.0f, 1.0f, 8.0f), 
            point_t<>(6.0f, 1.0f, 8.0f),
            point_t<>(6.0f, 5.0f, 8.0f),
            point_t<>(3.0f, 5.0f, 8.0f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4 /* Bottom face */
        });

    const float m = 60.0f;
    const point_t<> com(4.5f, 3.0f, 5.5f);
    const float it[6] = { (41.0f / 12.0f) * m, (34.0f / 12.0f) * m, (25.0f / 12.0f) * m, 0.0f, 0.0f, 0.0f };
    const float it_inv[6] = { 0.00488f, 0.00588f, 0.008f, 0.0f, 0.0f, 0.0f };

    inertia_tensor uut(p, e);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


BOOST_AUTO_TEST_CASE( cuboid_with_density_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(3.0f, 1.0f, 3.0f), 
            point_t<>(6.0f, 1.0f, 3.0f),
            point_t<>(6.0f, 5.0f, 3.0f),
            point_t<>(3.0f, 5.0f, 3.0f),
            point_t<>(3.0f, 1.0f, 8.0f), 
            point_t<>(6.0f, 1.0f, 8.0f),
            point_t<>(6.0f, 5.0f, 8.0f),
            point_t<>(3.0f, 5.0f, 8.0f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4 /* Bottom face */
        });

    const float r = 2.7f;
    const float m = 60.0f * r;
    const point_t<> com(4.5f, 3.0f, 5.5f);
    const float it[6] = { (41.0f / 12.0f) * m, (34.0f / 12.0f) * m, (25.0f / 12.0f) * m, 0.0f, 0.0f, 0.0f };
    const float it_inv[6] = { 0.0018067f, 0.0021786f, 0.00296295f, 0.0f, 0.0f, 0.0f };

    inertia_tensor uut(p, e, r);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


BOOST_AUTO_TEST_CASE( unaligned_cuboid_with_density_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(1.67678f,  2.32322f,  3.48223f),
            point_t<>(4.23744f,  2.76256f,  1.98223f),
            point_t<>(4.82322f,  6.17678f,  3.98223f),
            point_t<>(2.26256f,  5.73744f,  5.48223f),
            point_t<>(4.17678f, -0.176777f, 7.01777f),
            point_t<>(6.73744f,  0.262563f, 5.51777f),
            point_t<>(7.32322f,  3.67678f,  7.51777f),
            point_t<>(4.76256f,  3.23744f,  9.01777f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4 /* Bottom face */
        });

    const float r = 2.7f;
    const float m = 60.0f * r;
    const point_t<> com(4.5f, 3.0f, 5.5f);
    const float it[6] = { 497.47f, 430.66f, 421.877f, 42.188f, 36.0414f, -83.2838f };
    const float it_inv[6] = { 0.00210374f, 0.00236671f, 0.0024778f, -0.000242576f, -0.000250078f, 0.000436028f };

    inertia_tensor uut(p, e, r);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(fabs(magnitude(uut.center_of_mass() - com)) < result_tolerance);
    BOOST_CHECK_CLOSE(uut.mass(), m, result_tolerance);
}


BOOST_AUTO_TEST_CASE( products_of_inertia_test )
{
    std::vector<point_t<>> p(
        {
            /* Cube about the origin */
            point_t<>(-2.0f, -2.0f, -2.0f), 
            point_t<>( 2.0f, -2.0f, -2.0f),
            point_t<>( 2.0f,  2.0f, -2.0f),
            point_t<>(-2.0f,  2.0f, -2.0f),
            point_t<>(-2.0f, -2.0f,  2.0f), 
            point_t<>( 2.0f, -2.0f,  2.0f),
            point_t<>( 2.0f,  2.0f,  2.0f),
            point_t<>(-2.0f,  2.0f,  2.0f),

            /* Cube on one octant o the original cube */
            point_t<>( 0.0f,  1.0f,  0.0f), 
            point_t<>( 2.0f,  1.0f,  0.0f),
            point_t<>( 2.0f,  2.0f,  0.0f),
            point_t<>( 0.0f,  2.0f,  0.0f),
            point_t<>( 0.0f,  1.0f,  4.0f), 
            point_t<>( 2.0f,  1.0f,  4.0f),
            point_t<>( 2.0f,  2.0f,  4.0f),
            point_t<>( 0.0f,  2.0f,  4.0f)
        });

    std::vector<int> e(
        {
            /* Cube about the origin */
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4, /* Bottom face */

            /* Cube on one octant o the original cube */
             8, 10,  9,  8, 11, 10, /* Front face */
            12, 13, 14, 12, 14, 15, /* Back face */
            12, 15,  8, 15, 11,  8, /* Left face */
             9, 10, 14,  9, 14, 13, /* Right face */
            11, 15, 14, 11, 14, 10, /* Top face */
             8,  9, 12,  9, 13, 12 /* Bottom face */
        });

    const float m = 72.0f;
    const point_t<> com(1.0f / 9.0f, 1.5f / 9.0f, 2.0f / 9.0f);
    const float it[6] = { 226.444f, 219.556f, 197.111f, -10.6667f, -14.2222f, -12.3333f };
    const float it_inv[6] = { 0.00444974f, 0.0046171f, 0.00515444f, 0.00025f, 0.00051775f, 0.00034812f };

    inertia_tensor uut(p, e);
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
    BOOST_CHECK(uut.center_of_mass() == com);
    BOOST_CHECK(uut.mass() == m);
}


/* Test rotated tensor */
BOOST_AUTO_TEST_CASE( rotated_cube_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(3.0f, 2.0f, 4.0f), 
            point_t<>(6.0f, 2.0f, 4.0f),
            point_t<>(6.0f, 5.0f, 4.0f),
            point_t<>(3.0f, 5.0f, 4.0f),
            point_t<>(3.0f, 2.0f, 7.0f), 
            point_t<>(6.0f, 2.0f, 7.0f),
            point_t<>(6.0f, 5.0f, 7.0f),
            point_t<>(3.0f, 5.0f, 7.0f)
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4 /* Bottom face */
        });

    const float m = 27.0f;
    const float t = (18.0f / 12.0f) * m;
    const float it[6] = { t, t, t, 0.0f, 0.0f, 0.0f };
    const float tensor_inv = 0.02469f;
    const float it_inv[6] = { tensor_inv, tensor_inv, tensor_inv, 0.0f, 0.0f, 0.0 };

    inertia_tensor tensor(p, e);
    BOOST_CHECK(std::equal(&it[0], &it[6], tensor.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], tensor.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    /* Build the rotated tensor and check */
    /* 90 degrees about y */
    inertia_tensor_view uut0(tensor, quaternion_t(0.7071067812f, 0.0f, 0.7071067812f, 0.0f));
    BOOST_CHECK(std::equal(&it[0], &it[6], uut0.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut0.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    /* 45 degrees about x and y */
    inertia_tensor_view uut1(tensor, quaternion_t(0.92388f, 0.270598f, 0.270598f, 0.0f));
    BOOST_CHECK(std::equal(&it[0], &it[6], uut1.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut1.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));


    /* 10 degrees about z */
    inertia_tensor_view uut2(tensor, quaternion_t(0.9961946981f, 0.0f, 0.0f, 0.08715574275f));
    BOOST_CHECK(std::equal(&it[0], &it[6], uut2.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut2.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));


    /* 181 degrees about something funky */
    inertia_tensor_view uut3(tensor, quaternion_t(-0.00872654f, 0.5488f, -0.32928f, -0.768321f));
    BOOST_CHECK(std::equal(&it[0], &it[6], uut3.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut3.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
}


BOOST_AUTO_TEST_CASE( rotated_cuboid_with_density_test )
{
    std::vector<point_t<>> p(
        {
            point_t<>(3.0f, 1.0f, 3.0f),
            point_t<>(6.0f, 1.0f, 3.0f),
            point_t<>(6.0f, 5.0f, 3.0f),
            point_t<>(3.0f, 5.0f, 3.0f),
            point_t<>(3.0f, 1.0f, 8.0f),
            point_t<>(6.0f, 1.0f, 8.0f),
            point_t<>(6.0f, 5.0f, 8.0f),
            point_t<>(3.0f, 5.0f, 8.0f) 
        });

    std::vector<int> e(
        {
            0, 2, 1, 0, 3, 2, /* Front face */
            4, 5, 6, 4, 6, 7, /* Back face */
            4, 7, 0, 7, 3, 0, /* Left face */
            1, 2, 6, 1, 6, 5, /* Right face */
            3, 7, 6, 3, 6, 2, /* Top face */
            0, 1, 4, 1, 5, 4  /* Bottom face */
        });

    const float r = 2.7f;
    const float m = 60.0f * r;
    const float it[6] = { (41.0f / 12.0f) * m, (34.0f / 12.0f) * m, (25.0f / 12.0f) * m, 0.0f, 0.0f, 0.0f };
    const float it_inv[6] = { 0.0018067f, 0.0021786f, 0.00296295f, 0.0f, 0.0f, 0.0 };
    
    inertia_tensor tensor(p, e, r);
    BOOST_CHECK(std::equal(&it[0], &it[6], tensor.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], tensor.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    /* Build the rotated tensor and check */
    /* 90 degrees about y */
    const float it0[6] = { 337.501f, 459.01f, 553.495f, 0.0f, 0.0f, 0.0 };
    const float it_inv0[6] = { 0.00296295f, 0.0021786f, 0.0018067f, 0.0f, 0.0f, 0.0f };
    inertia_tensor_view uut0(tensor, quaternion_t(0.7071067812f, 0.0f, 0.7071067812f, 0.0f));
    BOOST_CHECK(std::equal(&it0[0], &it0[6], uut0.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv0[0], &it_inv0[6], uut0.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    /* 45 degrees about x and y */
    const float it1[6] = { 497.47f, 430.66f, 421.877f, 42.188f, 36.0414f, -83.2838f };
    const float it_inv1[6] = { 0.00210374f, 0.00236671f, 0.0024778f, -0.000242576f, -0.000250078f, 0.000436028f };
    inertia_tensor_view uut1(tensor, quaternion_t(0.92388f, 0.270598f, 0.270598f, 0.0f));
    BOOST_CHECK(std::equal(&it1[0], &it1[6], uut1.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv1[0], &it_inv1[6], uut1.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));


    /* 10 degrees about z */
    const float it2[6] = { 550.648, 461.859, 337.501, 16.1579, 0.0f, 0.0 };
    const float it_inv2[6] = { 0.00181791, 0.00216739, 0.00296295, -0.0000635986, 0.0f, 0.0 };
    inertia_tensor_view uut2(tensor, quaternion_t(0.9961946981, 0.0f, 0.0f, 0.08715574275));
    BOOST_CHECK(std::equal(&it2[0], &it2[6], uut2.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv2[0], &it_inv2[6], uut2.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));


    /* 181 degrees about something funnky */
    const float it3[6] = { 388.699, 438.156, 523.154, 65.5395, 16.5929, 50.2863 };
    const float it_inv3[6] = { 0.00267007, 0.00234204, 0.00193613, -0.000390139, -0.0000367817, -0.000244277 };
    inertia_tensor_view uut3(tensor, quaternion_t(-0.00872654, 0.5488, -0.32928, -0.768321));
    BOOST_CHECK(std::equal(&it3[0], &it3[6], uut3.tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));

    BOOST_CHECK(std::equal(&it_inv3[0], &it_inv3[6], uut3.inverse_tensor(), [] (const float exp, const float act)
        {
            const float abs_diff = fabs(exp - act);
            return ((exp == 0.0f) ? abs_diff : (abs_diff / exp)) < result_tolerance;
        }));
}


BOOST_AUTO_TEST_CASE( inifinite_mass_view_test )
{
    const float m = std::numeric_limits<float>::infinity();
    const point_t<> com(30.1, -4.7, 5.3);
    float *const it = new float [6] { m, m, m, m, m, m };
    const float it_inv[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
    
    inertia_tensor tensor(it, com, m);
    BOOST_CHECK(std::equal(&it[0], &it[6], tensor.tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], tensor.inverse_tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));

    /* Rotation makes no different to an infinite mass object */
    inertia_tensor_view uut(tensor, quaternion_t(-0.00872654, 0.5488, -0.32928, -0.768321));
    BOOST_CHECK(std::equal(&it[0], &it[6], uut.tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));
    BOOST_CHECK(std::equal(&it_inv[0], &it_inv[6], uut.inverse_tensor(), [] (const float exp, const float act)
        {
            return exp == act;
        }));
}


BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
