/* Common headers */
#include "logging.h"

#ifdef STAND_ALONE
#define BOOST_TEST_MODULE circle_sampler test

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <chrono>

/* Common headers */
#include "common.h"

#include "circle_sampler.h"

#include "boost/test/unit_test.hpp"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.001f;

/* Support class shared_normal tests */
struct circle_sampler_fixture
{
    circle_sampler_fixture() :
        rand_unit_uut(point_t<>(0.0f, 1.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f)),
        rand_radius_uut(point_t<>(0.0f, 0.0f, 1.0f), point_t<>(5.0f, 0.0f, 0.0f)),
        sobol_unit_uut(point_t<>(0.0f, 1.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f), 64),
        sobol_radius_uut(point_t<>(0.0f, 0.0f, 1.0f), point_t<>(5.0f, 0.0f, 0.0f), 64),
        strat_unit_uut(point_t<>(0.0f, 1.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f), 48),
        strat_radius_uut(point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 6.0f), 48)
    {  }

    float radius(const point_t<> &pt, const point_t<> &c)
    {
        return magnitude(pt - c);
    }

    circle_sampler_random       rand_unit_uut;
    circle_sampler_random       rand_radius_uut;
    circle_sampler_sobol        sobol_unit_uut;
    circle_sampler_sobol        sobol_radius_uut;
    circle_sampler_stratified   strat_unit_uut;
    circle_sampler_stratified   strat_radius_uut;
};

BOOST_FIXTURE_TEST_SUITE( circle_sampler_tests, circle_sampler_fixture )

/* Hard to tell if random is really working but check the first few samples */
BOOST_AUTO_TEST_CASE( random_unit_test )
{
    circle_sampler::seed();
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>( 0.13154f, 0.0f,  0.0f    ), result_tolerance));
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>( 0.01615f, 0.0f,  0.45837f), result_tolerance));
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>(-0.21435f, 0.0f,  0.04476f), result_tolerance));
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>( 0.64942f, 0.0f, -0.19776f), result_tolerance));
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>(-0.40171f, 0.0f,  0.84397f), result_tolerance));
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>(-0.38637f, 0.0f, -0.34715f), result_tolerance));
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>( 0.01684f, 0.0f,  0.03020f), result_tolerance));
    BOOST_CHECK(rand_unit_uut.sample().close(point_t<>( 0.50010f, 0.0f, -0.17460f), result_tolerance));
}

/* Then give it a longer run to test the statistics */
BOOST_AUTO_TEST_CASE( random_unit_run_test )
{
    circle_sampler::seed();
    const int size = 1000000;
    point_t<> c(0.0f, 0.0f, 0.0f);
    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < size; ++i)
    {
        const point_t<> s(rand_unit_uut.sample());
        BOOST_CHECK(radius(s, point_t<>(0.0f, 0.0f, 0.0f)) < (1.0f + result_tolerance));

        c += s;
    }

    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: random_unit_run_test " << size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    c *= 1.0f / size;
    BOOST_CHECK(c.close(point_t<>(0.0f, 0.0f, 0.0f), result_tolerance));
}

BOOST_AUTO_TEST_CASE( random_radius_test )
{
    circle_sampler::seed();
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>( 0.65769f,  0.0f,     0.0f), result_tolerance));
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>( 0.08075f, -2.29183f, 0.0f), result_tolerance));
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>(-1.07167f, -0.22381f, 0.0f), result_tolerance));
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>( 3.24711f,  0.98878f, 0.0f), result_tolerance));
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>(-2.00854f, -4.21984f, 0.0f), result_tolerance));
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>(-1.93185f,  1.73574f, 0.0f), result_tolerance));
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>( 0.08420f, -0.15097f, 0.0f), result_tolerance));
    BOOST_CHECK(rand_radius_uut.sample().close(point_t<>( 2.50048f,  0.87302f, 0.0f), result_tolerance));
}

BOOST_AUTO_TEST_CASE( random_radius_run_test )
{
    circle_sampler::seed();
    const int size = 1000000;
    point_t<> c(0.0f, 0.0f, 0.0f);
    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < size; ++i)
    {
        const point_t<> s(rand_radius_uut.sample());
        BOOST_CHECK(radius(s, point_t<>(0.0f, 0.0f, 0.0f)) < (5.0f + result_tolerance));

        c += s;
    }

    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: random_radius_run_test " << size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    c *= 1.0f / size;
    BOOST_CHECK_MESSAGE(c.close(point_t<>(0.0f, 0.0f, 0.0f), result_tolerance), c);
}

/* A bit easier for sobol sampling, atleast we should be in the right sector */
BOOST_AUTO_TEST_CASE( sobol_unit_test )
{
    circle_sampler::seed();
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.0f,     0.0f,  0.0f    ), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.51129f, 0.0f,  0.02304f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.01630f, 0.0f, -0.75815f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.01670f, 0.0f,  0.25018f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.29651f, 0.0f, -0.24654f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.65390f, 0.0f,  0.59039f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.44959f, 0.0f, -0.45265f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.08423f, 0.0f,  0.09349f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.07590f, 0.0f, -0.18286f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.26959f, 0.0f,  0.63895f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.84584f, 0.0f, -0.41920f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.39539f, 0.0f,  0.20788f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.10892f, 0.0f, -0.30697f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.26260f, 0.0f,  0.77041f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>(-0.53942f, 0.0f, -0.18107f), result_tolerance));
    BOOST_CHECK(sobol_unit_uut.sample().close(point_t<>( 0.07288f, 0.0f,  0.02398f), result_tolerance));
}

/* Then give it a longer run to test the statistics */
BOOST_AUTO_TEST_CASE( sobol_unit_run_test )
{
    circle_sampler::seed();
    const int size = 6400;
    point_t<> c(0.0f, 0.0f, 0.0f);
    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < size / 64; ++i)
    {
        for (int j = 0; j < 64; ++j)
        {
            const point_t<> s(sobol_unit_uut.sample());
            BOOST_CHECK(radius(s, point_t<>(0.0f, 0.0f, 0.0f)) < (1.0f + result_tolerance));

            c += s;
        }
    
        sobol_unit_uut.reset(point_t<>(0.0f, 1.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f));
    }

    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: sobol_unit_run_test " << size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    c *= 1.0f / size;
    BOOST_CHECK_MESSAGE(c.close(point_t<>(0.0f, 0.0f, 0.0f), result_tolerance), c);
}

BOOST_AUTO_TEST_CASE( sobol_radius_test )
{
    circle_sampler::seed();
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 0.0f,      0.0f,     0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-2.55644f, -0.11519f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-0.08150f,  3.79075f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 0.08349f, -1.25089f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-1.48255f,  1.23269f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 3.26949f, -2.95196f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 2.24794f,  2.26326f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-0.42117f, -0.46742f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-0.37952f,  0.91429f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 1.34793f, -3.19473f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 4.22921f,  2.09601f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-1.97693f, -1.03939f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 0.54461f,  1.53485f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-1.31298f, -3.85207f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>(-2.69710f,  0.90536f, 0.0f), result_tolerance));
    BOOST_CHECK(sobol_radius_uut.sample().close(point_t<>( 0.36440f, -0.11990f, 0.0f), result_tolerance));
}

BOOST_AUTO_TEST_CASE( sobol_radius_run_test )
{
    circle_sampler::seed();
    const int size = 6400;
    point_t<> c(0.0f, 0.0f, 0.0f);
    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < size / 64; ++i)
    {
        for (int j = 0; j < 64; ++j)
        {
            const point_t<> s(sobol_radius_uut.sample());
            BOOST_CHECK(radius(s, point_t<>(0.0f, 0.0f, 0.0f)) < (5.0f + result_tolerance));

            c += s;
        }

        sobol_radius_uut.reset(point_t<>(0.0f, 0.0f, 1.0f), point_t<>(5.0f, 0.0f, 0.0f));
    }

    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: sobol_radius_run_test " << size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    c *= 1.0f / size;
    BOOST_CHECK_MESSAGE(c.close(point_t<>(0.0f, 0.0f, 0.0f), result_tolerance), c);
}

#define CHECK_NUMBER_OF_SAMPLES(s, r, t, n) \
    const circle_sampler_stratified uut ## s(point_t<>(0.0f, 1.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f), (s));     \
    BOOST_CHECK_MESSAGE(uut ## s.r_samples() == (r), "r samples " + std::to_string(uut ## s.r_samples()));                              \
    BOOST_CHECK_MESSAGE(uut ## s.t_samples() == (t), "t samples " + std::to_string(uut ## s.t_samples()));                              \
    BOOST_CHECK_MESSAGE(uut ## s.samples() == (n), "samples " + std::to_string(uut ## s.samples()));                                    \

BOOST_AUTO_TEST_CASE( strat_size_test )
{
    circle_sampler::seed();
    CHECK_NUMBER_OF_SAMPLES(1, 1, 1, 1);
    CHECK_NUMBER_OF_SAMPLES(2, 1, 2, 2);
    CHECK_NUMBER_OF_SAMPLES(3, 1, 3, 3);
    CHECK_NUMBER_OF_SAMPLES(4, 1, 4, 4);
    CHECK_NUMBER_OF_SAMPLES(5, 1, 5, 5);

    CHECK_NUMBER_OF_SAMPLES(6, 1, 6, 6);
    CHECK_NUMBER_OF_SAMPLES(7, 2, 3, 6);
    CHECK_NUMBER_OF_SAMPLES(8, 2, 4, 8);
    CHECK_NUMBER_OF_SAMPLES(9, 2, 4, 8);

    CHECK_NUMBER_OF_SAMPLES(19, 2,  9, 18);
    CHECK_NUMBER_OF_SAMPLES(20, 2, 10, 20);
    CHECK_NUMBER_OF_SAMPLES(21, 2, 10, 20);
    CHECK_NUMBER_OF_SAMPLES(22, 3,  7, 21);
    CHECK_NUMBER_OF_SAMPLES(23, 3,  7, 21);
    CHECK_NUMBER_OF_SAMPLES(24, 3,  8, 24);

    CHECK_NUMBER_OF_SAMPLES(45, 3, 15, 45);
    CHECK_NUMBER_OF_SAMPLES(46, 3, 15, 45);
    CHECK_NUMBER_OF_SAMPLES(47, 3, 15, 45);
    CHECK_NUMBER_OF_SAMPLES(48, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES(49, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES(50, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES(51, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES(52, 4, 13, 52);

    CHECK_NUMBER_OF_SAMPLES(76, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES(77, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES(78, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES(79, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES(80, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES(81, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES(82, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES(83, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES(84, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES(85, 5, 17, 85);
}

/* At least we can check they are covering all cells, even with a little randomness inside them */
#define CHECK_SAMPLE(uut, exp_pt, exp_rl, exp_ru)                       \
    do                                                                  \
    {                                                                   \
        const point_t<> pt((uut).sample());                               \
        BOOST_CHECK_MESSAGE(pt.close((exp_pt), result_tolerance), pt);  \
        BOOST_CHECK(magnitude(pt) >= (exp_rl));                         \
        BOOST_CHECK(magnitude(pt) <= (exp_ru));                         \
    } while (false)

BOOST_AUTO_TEST_CASE( strat_unit_test )
{
    circle_sampler::seed();
    CHECK_NUMBER_OF_SAMPLES(48, 4, 12, 48);

    /* Ring 0 */
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.0f,     0.0f,  0.0f    ), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.27287f, 0.0f, -0.26130f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.10593f, 0.0f, -0.24442f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.00819f, 0.0f, -0.02205f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.28817f, 0.0f, -0.17978f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.18571f, 0.0f, -0.04774f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.41542f, 0.0f,  0.00752f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.01861f, 0.0f,  0.01919f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.16662f, 0.0f,  0.29129f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.00671f, 0.0f,  0.19159f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.16131f, 0.0f,  0.13249f), 0.0f, 0.5f);
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.29429f, 0.0f,  0.01072f), 0.0f, 0.5f);
    
    /* Ring 1 */
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.64971f, 0.0f, -0.18395f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.33628f, 0.0f, -0.39538f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.09133f, 0.0f, -0.57900f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.26755f, 0.0f, -0.63443f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.28902f, 0.0f, -0.47305f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.61250f, 0.0f, -0.22479f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.58218f, 0.0f,  0.24344f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.53258f, 0.0f,  0.46232f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.00504f, 0.0f,  0.55114f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.24967f, 0.0f,  0.59978f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.33816f, 0.0f,  0.53739f), 0.5f, std::sqrt(0.5f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.62967f, 0.0f,  0.03806f), 0.5f, std::sqrt(0.5f));
    
    /* Ring 2 */
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.73094f, 0.0f, -0.17000f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.59293f, 0.0f, -0.57926f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.27606f, 0.0f, -0.69185f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.06654f, 0.0f, -0.76130f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.65735f, 0.0f, -0.42804f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.75063f, 0.0f, -0.40220f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.82136f, 0.0f,  0.22217f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.60831f, 0.0f,  0.50272f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.22623f, 0.0f,  0.83376f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.03559f, 0.0f,  0.74856f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.45722f, 0.0f,  0.72570f), std::sqrt(0.5f), std::sqrt(0.75f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.74613f, 0.0f,  0.24930f), std::sqrt(0.5f), std::sqrt(0.75f));
    
    /* Ring 3 */
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.80173f, 0.0f, -0.41583f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.67478f, 0.0f, -0.65011f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.47335f, 0.0f, -0.87189f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.37990f, 0.0f, -0.89039f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.54291f, 0.0f, -0.81219f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.85663f, 0.0f, -0.14099f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.92946f, 0.0f,  0.31798f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.59705f, 0.0f,  0.75769f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>(-0.05831f, 0.0f,  0.99823f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.14330f, 0.0f,  0.88575f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.65010f, 0.0f,  0.64112f), std::sqrt(0.75f), std::sqrt(1.0f));
    CHECK_SAMPLE(strat_unit_uut, point_t<>( 0.94215f, 0.0f,  0.07614f), std::sqrt(0.75f), std::sqrt(1.0f));
}

/* Then give it a longer run to test the statistics */
BOOST_AUTO_TEST_CASE( strat_unit_run_test )
{
    circle_sampler::seed();
    const int size = 9600;
    point_t<> c(0.0f, 0.0f, 0.0f);
    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < size / 48; ++i)
    {
        for (int j = 0; j < 48; ++j)
        {
            const point_t<> s(strat_unit_uut.sample());
            BOOST_CHECK(radius(s, point_t<>(0.0f, 0.0f, 0.0f)) < (1.0f + result_tolerance));

            c += s;
        }

        strat_unit_uut.reset(point_t<>(0.0f, 1.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f));
    }

    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: strat_unit_run_test " << size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    c *= 1.0f / size;
    BOOST_CHECK(c.close(point_t<>(0.0f, 0.0f, 0.0f), result_tolerance));
}


#define CHECK_NUMBER_OF_SAMPLES_RADIUS(s, r, t, n) \
    const circle_sampler_stratified uut ## s(point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 6.0f), (s));     \
    BOOST_CHECK_MESSAGE(uut ## s.r_samples() == (r), "r samples " + std::to_string(uut ## s.r_samples()));                              \
    BOOST_CHECK_MESSAGE(uut ## s.t_samples() == (t), "t samples " + std::to_string(uut ## s.t_samples()));                              \
    BOOST_CHECK_MESSAGE(uut ## s.samples() == (n), "samples " + std::to_string(uut ## s.samples()));                                    \

BOOST_AUTO_TEST_CASE( strat_radius_size_test )
{
    circle_sampler::seed();
    CHECK_NUMBER_OF_SAMPLES_RADIUS(1, 1, 1, 1);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(2, 1, 2, 2);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(3, 1, 3, 3);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(4, 1, 4, 4);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(5, 1, 5, 5);

    CHECK_NUMBER_OF_SAMPLES_RADIUS(6, 1, 6, 6);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(7, 2, 3, 6);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(8, 2, 4, 8);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(9, 2, 4, 8);

    CHECK_NUMBER_OF_SAMPLES_RADIUS(19, 2,  9, 18);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(20, 2, 10, 20);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(21, 2, 10, 20);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(22, 3,  7, 21);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(23, 3,  7, 21);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(24, 3,  8, 24);

    CHECK_NUMBER_OF_SAMPLES_RADIUS(45, 3, 15, 45);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(46, 3, 15, 45);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(47, 3, 15, 45);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(48, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(49, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(50, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(51, 4, 12, 48);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(52, 4, 13, 52);

    CHECK_NUMBER_OF_SAMPLES_RADIUS(76, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(77, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(78, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(79, 4, 19, 76);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(80, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(81, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(82, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(83, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(84, 5, 16, 80);
    CHECK_NUMBER_OF_SAMPLES_RADIUS(85, 5, 17, 85);
}

BOOST_AUTO_TEST_CASE( strat_radius_test )
{
    circle_sampler::seed();
    CHECK_NUMBER_OF_SAMPLES_RADIUS(48, 4, 12, 48);

    /* Ring 0 */
    CHECK_SAMPLE(strat_radius_uut, point_t<>( 0.0f,     0.0f,  0.0f    ), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -1.56780f,  1.63721f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -1.46650f,  0.63556f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -0.13231f, -0.04912f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -1.07869f, -1.72900f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -0.28646f, -1.11427f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  0.04512f, -2.49249f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  0.11516f, -0.11163f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  1.74774f, -0.99969f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  1.14954f,  0.04025f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  0.79492f,  0.96786f), 0.0f, 3.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  0.06434f,  1.76576f), 0.0f, 3.0f);
    
    /* Ring 1 */
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -1.10367f,  3.89826f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -2.37226f,  2.01770f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -3.47398f,  0.54801f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -3.80656f, -1.60529f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -2.83830f, -1.73413f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -1.34872f, -3.67501f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  1.46063f, -3.49306f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  2.77390f, -3.19549f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  3.30684f, -0.03021f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  3.59866f,  1.49803f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  3.22435f,  2.02895f), 3.0f, 6.0f * std::sqrt(0.5f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  0.22834f,  3.77800f), 3.0f, 6.0f * std::sqrt(0.5f));
    
    /* Ring 2 */
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -1.01995f,  4.38563f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -3.47555f,  3.55756f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -4.15110f,  1.65636f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -4.56779f, -0.39925f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -2.56821f, -3.94409f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -2.41321f, -4.50380f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  1.33301f, -4.92814f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  3.01631f, -3.64985f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  5.00254f, -1.35735f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  4.49134f,  0.21353f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  4.35421f,  2.74331f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  1.49580f,  4.47679f), 6.0f * std::sqrt(0.5f), 6.0f * std::sqrt(0.75f));
    
    /* Ring 3 */
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -2.49499f,  4.81034f), 6.0f * std::sqrt(0.75f ), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -3.90066f,  4.04866f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -5.23133f,  2.84008f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -5.34235f, -2.27938f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -4.87311f, -3.25746f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f, -0.84594f, -5.13976f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  1.90787f, -5.57677f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  4.54616f, -3.58231f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  5.98936f, -0.34984f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  5.31451f,  0.85978f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  3.84674f,  3.90058f), 6.0f * std::sqrt(0.75f), 6.0f);
    CHECK_SAMPLE(strat_radius_uut, point_t<>(0.0f,  0.45686f,  5.65289f), 6.0f * std::sqrt(0.75f), 6.0f);
}

BOOST_AUTO_TEST_CASE( strat_radius_run_test )
{
    circle_sampler::seed();
    const int size = 240000;
    point_t<> c(0.0f, 0.0f, 0.0f);
    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < size / 48; ++i)
    {
        for (int j = 0; j < 48; ++j)
        {
            const point_t<> s(strat_radius_uut.sample());
            BOOST_CHECK(radius(s, point_t<>(0.0f, 0.0f, 0.0f)) < (6.0f + result_tolerance));

            c += s;
        }

        strat_radius_uut.reset(point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 6.0f));
    }

    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: strat_radius_run_test " << size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    c *= 1.0f / size;
    BOOST_CHECK_MESSAGE(c.close(point_t<>(0.0f, 0.0f, 0.0f), result_tolerance), c);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
