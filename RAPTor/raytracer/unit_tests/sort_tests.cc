#ifdef STAND_ALONE
#define BOOST_TEST_MODULE sort test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <chrono>
#include <random>
#include <vector>

/* Common headers */
#include "logging.h"

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "sort.h"


namespace raptor_raytracer
{
namespace test
{
const int result_tolerance = 0.00001;

struct sort_fixture
{
    sort_fixture() : small_sample({ 1.0f, 10.9f, -1.0f, 5.4f, 3.7f, 10.9f, -4.3f, -1.5f }) { }

    std::vector<float> ready_sorted(const int n)
    {
        /* Get a vector of the right size */
        std::vector<float> ret;
        ret.resize(n);

        /* Fill it */
        float seq = 0.0f;
        std::generate(ret.begin(), ret.end(), [&seq]()
            {
                return seq++;
            });
        return ret;
    }

    std::vector<float> inverse_sorted(const int n)
    {
        /* Get a vector of the right size */
        std::vector<float> ret;
        ret.resize(n);

        /* Fill it */
        float seq = n;
        std::generate(ret.begin(), ret.end(), [&seq]()
            {
                return seq--;
            });
        return ret;
    }

    std::vector<float> random(const int n)
    {
        /* Get a vector of the right size */
        std::vector<float> ret;
        ret.resize(n);

        /* Fill it */
        std::default_random_engine gen;
        std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
        std::generate(ret.begin(), ret.end(), [&dist, &gen]()
            {
                return dist(gen);
            });
        return ret;
    }

    std::vector<float> small_sample;
#ifndef VALGRIND_TESTS
    const int large_test_size_from = 65536;
    const int large_test_size_to = 8388608;
#else
    const int large_test_size_from = 1024;
    const int large_test_size_to = 4096;
#endif /* #ifndef VALGRIND_TESTS */
};

BOOST_FIXTURE_TEST_SUITE( sort_tests, sort_fixture );


/* Standard sort */
BOOST_AUTO_TEST_CASE( large_ready_sorted_std_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(ready_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        std::sort(data.begin(), data.end());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_ready_sorted_std_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 0.0f; i < data.size(); ++i)
        {
            passed &= (data[i] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_inverse_sorted_std_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(inverse_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        std::sort(data.begin(), data.end());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_inverse_sorted_std_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i <= data.size(); ++i)
        {
            passed &= (data[i - 1] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_random_std_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(random(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        std::sort(data.begin(), data.end());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_random_std_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i < data.size(); ++i)
        {
            passed &= (data[i] >= data[i - 1]);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( l1_cache_random_std_sort_test )
{
    /* Generate data */
    const int test_size = 1024;
    auto data(random(large_test_size_to));

    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < large_test_size_to; i += test_size)
    {
        /* Sort the data */
        std::sort(&data[i], &data[i + test_size]);
    }
    auto t1(std::chrono::system_clock::now());
    
    /* Checks */
    bool passed = true;
    for (int i = 0; i < large_test_size_to; i += test_size)
    {
        for (float j = 1; j < test_size; ++j)
        {
            passed &= (data[i + j] >= data[i + j - 1]);
        }
    }
    BOOST_CHECK(passed);

    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: l1_cache_random_std_sort_test " << test_size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << (std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * (static_cast<float>(test_size) / large_test_size_to));
}

BOOST_AUTO_TEST_CASE( large_ready_sorted_std_stable_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(ready_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        std::stable_sort(data.begin(), data.end());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_ready_sorted_std_stable_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 0.0f; i < data.size(); ++i)
        {
            passed &= (data[i] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_inverse_sorted_std_stable_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(inverse_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        std::stable_sort(data.begin(), data.end());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_inverse_sorted_std_stable_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i <= data.size(); ++i)
        {
            passed &= (data[i - 1] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_random_std_stable_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(random(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        std::stable_sort(data.begin(), data.end());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_random_std_stable_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i < data.size(); ++i)
        {
            passed &= (data[i] >= data[i - 1]);
        }
        BOOST_CHECK(passed);
    }
}


/* Vector merge sort */
BOOST_AUTO_TEST_CASE( small_sample_vector_merge_sort_test )
{
    float output[8];
    vmerge_sort(small_sample.data(), &output[0], small_sample.size());  /* Actually just calls merge sort because there isnt enough data */
    BOOST_CHECK(small_sample[0] == -4.3f);
    BOOST_CHECK(small_sample[1] == -1.5f);
    BOOST_CHECK(small_sample[2] == -1.0f);
    BOOST_CHECK(small_sample[3] ==  1.0f);
    BOOST_CHECK(small_sample[4] ==  3.7f);
    BOOST_CHECK(small_sample[5] ==  5.4f);
    BOOST_CHECK(small_sample[6] ==  10.9f);
    BOOST_CHECK(small_sample[7] ==  10.9f);
}

BOOST_AUTO_TEST_CASE( small_ready_sorted_vector_merge_sort_test )
{
    auto data(ready_sorted(16));
    float output[16];
    vmerge_sort(data.data(), output, data.size());
    BOOST_CHECK(data[ 0] ==  0.0f);
    BOOST_CHECK(data[ 1] ==  1.0f);
    BOOST_CHECK(data[ 2] ==  2.0f);
    BOOST_CHECK(data[ 3] ==  3.0f);
    BOOST_CHECK(data[ 4] ==  4.0f);
    BOOST_CHECK(data[ 5] ==  5.0f);
    BOOST_CHECK(data[ 6] ==  6.0f);
    BOOST_CHECK(data[ 7] ==  7.0f);
    BOOST_CHECK(data[ 8] ==  8.0f);
    BOOST_CHECK(data[ 9] ==  9.0f);
    BOOST_CHECK(data[10] == 10.0f);
    BOOST_CHECK(data[11] == 11.0f);
    BOOST_CHECK(data[12] == 12.0f);
    BOOST_CHECK(data[13] == 13.0f);
    BOOST_CHECK(data[14] == 14.0f);
    BOOST_CHECK(data[15] == 15.0f);
}

BOOST_AUTO_TEST_CASE( small_inverse_sorted_vector_merge_sort_test )
{
    auto data(inverse_sorted(16));
    float output[16];
    vmerge_sort(data.data(), output, data.size());
    BOOST_CHECK_MESSAGE(data[ 0] ==  1.0f, data[ 0]);
    BOOST_CHECK_MESSAGE(data[ 1] ==  2.0f, data[ 1]);
    BOOST_CHECK_MESSAGE(data[ 2] ==  3.0f, data[ 2]);
    BOOST_CHECK_MESSAGE(data[ 3] ==  4.0f, data[ 3]);
    BOOST_CHECK_MESSAGE(data[ 4] ==  5.0f, data[ 4]);
    BOOST_CHECK_MESSAGE(data[ 5] ==  6.0f, data[ 5]);
    BOOST_CHECK_MESSAGE(data[ 6] ==  7.0f, data[ 6]);
    BOOST_CHECK_MESSAGE(data[ 7] ==  8.0f, data[ 7]);
    BOOST_CHECK_MESSAGE(data[ 8] ==  9.0f, data[ 8]);
    BOOST_CHECK_MESSAGE(data[ 9] == 10.0f, data[ 9]);
    BOOST_CHECK_MESSAGE(data[10] == 11.0f, data[10]);
    BOOST_CHECK_MESSAGE(data[11] == 12.0f, data[11]);
    BOOST_CHECK_MESSAGE(data[12] == 13.0f, data[12]);
    BOOST_CHECK_MESSAGE(data[13] == 14.0f, data[13]);
    BOOST_CHECK_MESSAGE(data[14] == 15.0f, data[14]);
    BOOST_CHECK_MESSAGE(data[15] == 16.0f, data[15]);
}

BOOST_AUTO_TEST_CASE( small_random_vector_merge_sort_test )
{
    auto data(random(16));
    float output[16];
    vmerge_sort(data.data(), output, data.size());
    BOOST_CHECK_CLOSE(data[ 0], -99.998436f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[ 1], -93.0855789f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 2], -90.5910797f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 3], -89.3076706f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 4], -73.6924438f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 5], -56.2081642f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 6], -23.2995834f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 7], -8.26997375f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 8],  3.88327789f, result_tolerance);
    BOOST_CHECK_CLOSE(data[ 9],  5.94004059f, result_tolerance);
    BOOST_CHECK_CLOSE(data[10],  6.55344391f, result_tolerance);
    BOOST_CHECK_CLOSE(data[11],  35.7729492f, result_tolerance);
    BOOST_CHECK_CLOSE(data[12],  35.8592834f, result_tolerance);
    BOOST_CHECK_CLOSE(data[13],  51.1210632f, result_tolerance);
    BOOST_CHECK_CLOSE(data[14],  66.1930695f, result_tolerance);
    BOOST_CHECK_CLOSE(data[15],  86.9385834f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( medium_random_vector_merge_sort_test )
{
    auto data(random(64));
    float output[64];
    vmerge_sort(data.data(), output, data.size());
    BOOST_CHECK_CLOSE(data[ 0], -99.998436f,    result_tolerance);
    BOOST_CHECK_CLOSE(data[ 1], -98.4603653f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[ 2], -93.0855789f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[ 3], -90.5910797f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[ 4], -90.5070953f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[ 5], -89.3076706f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[ 6], -87.8871307f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[ 7], -86.6315536f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[ 8], -85.462822f,    result_tolerance);
    BOOST_CHECK_CLOSE(data[ 9], -81.6070251f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[10], -73.6924438f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[11], -66.6985626f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[12], -56.2081642f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[13], -52.4451141f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[14], -50.5922241f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[15], -47.5094032f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[16], -45.4580078f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[17], -45.018631f,    result_tolerance);
    BOOST_CHECK_CLOSE(data[18], -36.1934128f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[19], -34.353157f,    result_tolerance);
    BOOST_CHECK_CLOSE(data[20], -28.1470032f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[21], -26.9322662f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[22], -23.3168716f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[23], -23.2995834f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[24], -16.8001328f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[25], -16.5028f,      result_tolerance);
    BOOST_CHECK_CLOSE(data[26], -12.71772f,     result_tolerance);
    BOOST_CHECK_CLOSE(data[27],  -8.26997375f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[28],  -4.4536438f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[29],  -2.69652557f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[30],   0.904586792f, result_tolerance);
    BOOST_CHECK_CLOSE(data[31],   3.25839233f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[32],   3.88327789f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[33],   5.38575745f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[34],   5.94004059f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[35],   6.55344391f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[36],  17.7953262f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[37],  26.3269424f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[38],  26.5277176f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[39],  30.3037109f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[40],  30.7837982f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[41],  34.2298737f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[42],  35.7729492f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[43],  35.8592834f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[44],  37.354538f,    result_tolerance);
    BOOST_CHECK_CLOSE(data[45],  40.2381134f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[46],  44.5320892f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[47],  47.2163849f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[48],  50.6711731f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[49],  51.1210632f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[50],  51.2820892f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[51],  52.4396057f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[52],  53.2989502f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[53],  66.1930695f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[54],  69.2333832f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[55],  76.9414368f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[56],  79.53125f,     result_tolerance);
    BOOST_CHECK_CLOSE(data[57],  80.9306183f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[58],  81.841629f,    result_tolerance);
    BOOST_CHECK_CLOSE(data[59],  82.0641632f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[60],  86.0872955f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[61],  86.9385834f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[62],  96.5100555f,   result_tolerance);
    BOOST_CHECK_CLOSE(data[63],  98.2074738f,   result_tolerance);
}

BOOST_AUTO_TEST_CASE( simd_width_m1_random_vector_merge_sort_test )
{
    auto data(random(23));
    float output[23];
    vmerge_sort(data.data(), output, data.size());
    BOOST_CHECK_CLOSE(output[ 0], -99.998436f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[ 1], -98.4603653f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 2], -93.0855789f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 3], -90.5910797f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 4], -89.3076706f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 5], -86.6315536f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 6], -73.6924438f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 7], -56.2081642f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 8], -23.3168716f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[ 9], -23.2995834f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[10], -16.5028f,      result_tolerance);
    BOOST_CHECK_CLOSE(output[11],  -8.26997375f,  result_tolerance);
    BOOST_CHECK_CLOSE(output[12],   3.88327789f,  result_tolerance);
    BOOST_CHECK_CLOSE(output[13],   5.94004059f,  result_tolerance);
    BOOST_CHECK_CLOSE(output[14],   6.55344391f,  result_tolerance);
    BOOST_CHECK_CLOSE(output[15],  17.7953262f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[16],  34.2298737f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[17],  35.7729492f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[18],  35.8592834f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[19],  37.354538f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[20],  51.1210632f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[21],  66.1930695f,   result_tolerance);
    BOOST_CHECK_CLOSE(output[22],  86.9385834f,   result_tolerance);
}

BOOST_AUTO_TEST_CASE( simd_width_p1_random_vector_merge_sort_test )
{
    auto data(random(25));
    float output[25];
    vmerge_sort(data.data(), output, data.size());
    BOOST_CHECK_CLOSE(output[ 0], -99.998436f,      result_tolerance);
    BOOST_CHECK_CLOSE(output[ 1], -98.4603653f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 2], -93.0855789f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 3], -90.5910797f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 4], -89.3076706f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 5], -86.6315536f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 6], -73.6924438f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 7], -56.2081642f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 8], -23.3168716f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 9], -23.2995834f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[10], -16.5028f,        result_tolerance);
    BOOST_CHECK_CLOSE(output[11],  -8.26997375f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[12],   3.88327789f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[13],   5.94004059f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[14],   6.55344391f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[15],  17.7953262f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[16],  34.2298737f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[17],  35.7729492f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[18],  35.8592834f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[19],  37.354538f,      result_tolerance);
    BOOST_CHECK_CLOSE(output[20],  51.1210632f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[21],  66.1930695f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[22],  69.2333832f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[23],  86.0872955f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[24],  86.9385834f,     result_tolerance);
}

BOOST_AUTO_TEST_CASE( not_in_register_block_random_vector_merge_sort_test )
{
    auto data(random(24));
    float output[24];
    vmerge_sort(data.data(), output, data.size());
    BOOST_CHECK_CLOSE(output[ 0], -99.998436f,      result_tolerance);
    BOOST_CHECK_CLOSE(output[ 1], -98.4603653f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 2], -93.0855789f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 3], -90.5910797f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 4], -89.3076706f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 5], -86.6315536f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 6], -73.6924438f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 7], -56.2081642f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 8], -23.3168716f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[ 9], -23.2995834f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[10], -16.5028f,        result_tolerance);
    BOOST_CHECK_CLOSE(output[11],  -8.26997375f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[12],   3.88327789f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[13],   5.94004059f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[14],   6.55344391f,    result_tolerance);
    BOOST_CHECK_CLOSE(output[15],  17.7953262f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[16],  34.2298737f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[17],  35.7729492f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[18],  35.8592834f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[19],  37.354538f,      result_tolerance);
    BOOST_CHECK_CLOSE(output[20],  51.1210632f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[21],  66.1930695f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[22],  86.0872955f,     result_tolerance);
    BOOST_CHECK_CLOSE(output[23],  86.9385834f,     result_tolerance);
}

BOOST_AUTO_TEST_CASE( odd_size_sweep_random_vector_merge_sort_test )
{
    for (int i = 16; i < 48; ++i)
    {
        auto data(random(i));
        std::vector<float> output(i);
        const bool check = vmerge_sort(data.data(), output.data(), data.size());

        /* Checks */
        bool passed = true;
        const auto &check_array = check ? data : output;
        for (int j = 1; j < i; ++j)
        {
            passed &= (check_array[j] >= check_array[j - 1]);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_ready_sorted_vector_merge_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(ready_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        const bool check = vmerge_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_ready_sorted_vector_merge_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        const auto &check_array = check ? data : output;
        for (float i = 0.0f; i < data.size(); ++i)
        {
            passed &= (check_array[i] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_inverse_sorted_vector_merge_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(inverse_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        const bool check = vmerge_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_inverse_sorted_vector_merge_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        const auto &check_array = check ? data : output;
        for (float i = 1.0f; i <= data.size(); ++i)
        {
            passed &= (check_array[i - 1] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_random_vector_merge_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
        {
        /* Generate data */
        auto data(random(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        const bool check = vmerge_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_random_vector_merge_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        const auto &check_array = check ? data : output;
        for (float i = 1.0f; i < data.size(); ++i)
        {
            passed &= (check_array[i] >= check_array[i - 1]);
        }
        BOOST_CHECK(passed);
    }
}


/* Merge sort tests */
BOOST_AUTO_TEST_CASE( small_sample_merge_sort_test )
{
    float output[8];
    merge_sort(small_sample.data(), &output[0], small_sample.size(), 2);
    BOOST_CHECK(small_sample[0] == -4.3f);
    BOOST_CHECK(small_sample[1] == -1.5f);
    BOOST_CHECK(small_sample[2] == -1.0f);
    BOOST_CHECK(small_sample[3] ==  1.0f);
    BOOST_CHECK(small_sample[4] ==  3.7f);
    BOOST_CHECK(small_sample[5] ==  5.4f);
    BOOST_CHECK(small_sample[6] ==  10.9f);
    BOOST_CHECK(small_sample[7] ==  10.9f);
}

BOOST_AUTO_TEST_CASE( small_ready_sorted_merge_sort_test )
{
    auto data(ready_sorted(10));
    float output[10];
    merge_sort(data.data(), output, data.size(), 2);
    BOOST_CHECK(output[0] == 0.0f);
    BOOST_CHECK(output[1] == 1.0f);
    BOOST_CHECK(output[2] == 2.0f);
    BOOST_CHECK(output[3] == 3.0f);
    BOOST_CHECK(output[4] == 4.0f);
    BOOST_CHECK(output[5] == 5.0f);
    BOOST_CHECK(output[6] == 6.0f);
    BOOST_CHECK(output[7] == 7.0f);
    BOOST_CHECK(output[8] == 8.0f);
    BOOST_CHECK(output[9] == 9.0f);
}

BOOST_AUTO_TEST_CASE( small_inverse_sorted_merge_sort_test )
{
    auto data(inverse_sorted(10));
    float output[10];
    merge_sort(data.data(), output, data.size(), 2);
    BOOST_CHECK(output[0] ==  1.0f);
    BOOST_CHECK(output[1] ==  2.0f);
    BOOST_CHECK(output[2] ==  3.0f);
    BOOST_CHECK(output[3] ==  4.0f);
    BOOST_CHECK(output[4] ==  5.0f);
    BOOST_CHECK(output[5] ==  6.0f);
    BOOST_CHECK(output[6] ==  7.0f);
    BOOST_CHECK(output[7] ==  8.0f);
    BOOST_CHECK(output[8] ==  9.0f);
    BOOST_CHECK(output[9] == 10.0f);
}

BOOST_AUTO_TEST_CASE( small_random_merge_sort_test )
{
    auto data(random(10));
    float output[10];
    merge_sort(data.data(), output, data.size(), 2);
    BOOST_CHECK_CLOSE(output[0], -99.998436f,  result_tolerance);
    BOOST_CHECK_CLOSE(output[1], -90.5910797f, result_tolerance);
    BOOST_CHECK_CLOSE(output[2], -73.6924438f, result_tolerance);
    BOOST_CHECK_CLOSE(output[3], -56.2081642f, result_tolerance);
    BOOST_CHECK_CLOSE(output[4], -8.26997375f, result_tolerance);
    BOOST_CHECK_CLOSE(output[5],  6.55344391f, result_tolerance);
    BOOST_CHECK_CLOSE(output[6],  35.7729492f, result_tolerance);
    BOOST_CHECK_CLOSE(output[7],  35.8592834f, result_tolerance);
    BOOST_CHECK_CLOSE(output[8],  51.1210632f, result_tolerance);
    BOOST_CHECK_CLOSE(output[9],  86.9385834f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( large_ready_sorted_merge_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(ready_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        const bool check = merge_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_ready_sorted_merge_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        const auto &check_array = check ? output : data;
        for (float i = 0.0f; i < data.size(); ++i)
        {
            passed &= (check_array[i] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_inverse_sorted_merge_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(inverse_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        const bool check = merge_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_inverse_sorted_merge_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        const auto &check_array = check ? output : data;
        for (float i = 1.0f; i <= data.size(); ++i)
        {
            passed &= (check_array[i - 1] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_random_merge_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(random(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        const bool check = merge_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_random_merge_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        const auto &check_array = check ? output : data;
        for (float i = 1.0f; i < data.size(); ++i)
        {
            passed &= (check_array[i] >= check_array[i - 1]);
        }
        BOOST_CHECK(passed);
    }
}


/* Radix sort tests */
BOOST_AUTO_TEST_CASE( small_sample_radix_sort_test )
{
    float output[8];
    radix_sort(small_sample.data(), &output[0], small_sample.size());
    BOOST_CHECK(output[0] == -4.3f);
    BOOST_CHECK(output[1] == -1.5f);
    BOOST_CHECK(output[2] == -1.0f);
    BOOST_CHECK(output[3] ==  1.0f);
    BOOST_CHECK(output[4] ==  3.7f);
    BOOST_CHECK(output[5] ==  5.4f);
    BOOST_CHECK(output[6] ==  10.9f);
    BOOST_CHECK(output[7] ==  10.9f);
}

BOOST_AUTO_TEST_CASE( small_ready_sorted_radix_sort_test )
{
    auto data(ready_sorted(10));
    float output[10];
    radix_sort(data.data(), output, data.size());
    BOOST_CHECK(output[0] == 0.0f);
    BOOST_CHECK(output[1] == 1.0f);
    BOOST_CHECK(output[2] == 2.0f);
    BOOST_CHECK(output[3] == 3.0f);
    BOOST_CHECK(output[4] == 4.0f);
    BOOST_CHECK(output[5] == 5.0f);
    BOOST_CHECK(output[6] == 6.0f);
    BOOST_CHECK(output[7] == 7.0f);
    BOOST_CHECK(output[8] == 8.0f);
    BOOST_CHECK(output[9] == 9.0f);
}

BOOST_AUTO_TEST_CASE( small_inverse_sorted_radix_sort_test )
{
    auto data(inverse_sorted(10));
    float output[10];
    radix_sort(data.data(), output, data.size());
    BOOST_CHECK(output[0] ==  1.0f);
    BOOST_CHECK(output[1] ==  2.0f);
    BOOST_CHECK(output[2] ==  3.0f);
    BOOST_CHECK(output[3] ==  4.0f);
    BOOST_CHECK(output[4] ==  5.0f);
    BOOST_CHECK(output[5] ==  6.0f);
    BOOST_CHECK(output[6] ==  7.0f);
    BOOST_CHECK(output[7] ==  8.0f);
    BOOST_CHECK(output[8] ==  9.0f);
    BOOST_CHECK(output[9] == 10.0f);
}

BOOST_AUTO_TEST_CASE( small_random_radix_sort_test )
{
    auto data(random(10));
    float output[10];
    radix_sort(data.data(), output, data.size());
    BOOST_CHECK_CLOSE(output[0], -99.998436f,  result_tolerance);
    BOOST_CHECK_CLOSE(output[1], -90.5910797f, result_tolerance);
    BOOST_CHECK_CLOSE(output[2], -73.6924438f, result_tolerance);
    BOOST_CHECK_CLOSE(output[3], -56.2081642f, result_tolerance);
    BOOST_CHECK_CLOSE(output[4], -8.26997375f, result_tolerance);
    BOOST_CHECK_CLOSE(output[5],  6.55344391f, result_tolerance);
    BOOST_CHECK_CLOSE(output[6],  35.7729492f, result_tolerance);
    BOOST_CHECK_CLOSE(output[7],  35.8592834f, result_tolerance);
    BOOST_CHECK_CLOSE(output[8],  51.1210632f, result_tolerance);
    BOOST_CHECK_CLOSE(output[9],  86.9385834f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( large_ready_sorted_radix_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(ready_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        radix_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_ready_sorted_radix_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 0.0f; i < data.size(); ++i)
        {
            passed &= (output[i] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_inverse_sorted_radix_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(inverse_sorted(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        radix_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_inverse_sorted_radix_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i <= data.size(); ++i)
        {
            passed &= (output[i - 1] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_random_radix_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(random(data_size));

        /* Sort the data */
        std::vector<float> output(data.size());
        auto t0(std::chrono::system_clock::now());
        radix_sort(data.data(), output.data(), data.size());
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_random_radix_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i < data.size(); ++i)
        {
            passed &= (output[i] >= output[i - 1]);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( l1_cache_random_radix_sort_test )
{
    /* Generate data */
    const int test_size = 1024;
    auto data(random(large_test_size_to));
    std::vector<float> output(data.size());

    auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < large_test_size_to; i += test_size)
    {
        /* Sort the data */
        radix_sort(&data[i], &output[i], test_size);
    }
    auto t1(std::chrono::system_clock::now());
    
    /* Checks */
    bool passed = true;
    for (int i = 0; i < large_test_size_to; i += test_size)
    {
        for (float j = 1; j < test_size; ++j)
        {
            passed &= (output[i + j] >= output[i + j - 1]);
        }
    }
    BOOST_CHECK(passed);

    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: l1_cache_random_radix_sort_test " << test_size;
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << (std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * (static_cast<float>(test_size) / large_test_size_to));
}


/* Insertion sort tests */
BOOST_AUTO_TEST_CASE( small_sample_insertion_sort_test )
{
    insertion_sort(small_sample.data(), small_sample.size() - 1);
    BOOST_CHECK(small_sample[0] == -4.3f);
    BOOST_CHECK(small_sample[1] == -1.5f);
    BOOST_CHECK(small_sample[2] == -1.0f);
    BOOST_CHECK(small_sample[3] ==  1.0f);
    BOOST_CHECK(small_sample[4] ==  3.7f);
    BOOST_CHECK(small_sample[5] ==  5.4f);
    BOOST_CHECK(small_sample[6] ==  10.9f);
    BOOST_CHECK(small_sample[7] ==  10.9f);
}

BOOST_AUTO_TEST_CASE( small_ready_sorted_insertion_sort_test )
{
    auto data(ready_sorted(10));
    insertion_sort(data.data(), data.size() - 1);
    BOOST_CHECK(data[0] == 0.0f);
    BOOST_CHECK(data[1] == 1.0f);
    BOOST_CHECK(data[2] == 2.0f);
    BOOST_CHECK(data[3] == 3.0f);
    BOOST_CHECK(data[4] == 4.0f);
    BOOST_CHECK(data[5] == 5.0f);
    BOOST_CHECK(data[6] == 6.0f);
    BOOST_CHECK(data[7] == 7.0f);
    BOOST_CHECK(data[8] == 8.0f);
    BOOST_CHECK(data[9] == 9.0f);
}

BOOST_AUTO_TEST_CASE( small_inverse_sorted_insertion_sort_test )
{
    auto data(inverse_sorted(10));
    insertion_sort(data.data(), data.size() - 1);
    BOOST_CHECK(data[0] ==  1.0f);
    BOOST_CHECK(data[1] ==  2.0f);
    BOOST_CHECK(data[2] ==  3.0f);
    BOOST_CHECK(data[3] ==  4.0f);
    BOOST_CHECK(data[4] ==  5.0f);
    BOOST_CHECK(data[5] ==  6.0f);
    BOOST_CHECK(data[6] ==  7.0f);
    BOOST_CHECK(data[7] ==  8.0f);
    BOOST_CHECK(data[8] ==  9.0f);
    BOOST_CHECK(data[9] == 10.0f);
}

BOOST_AUTO_TEST_CASE( small_random_insertion_sort_test )
{
    auto data(random(10));
    insertion_sort(data.data(), data.size() - 1);
    BOOST_CHECK_CLOSE(data[0], -99.998436f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[1], -90.5910797f, result_tolerance);
    BOOST_CHECK_CLOSE(data[2], -73.6924438f, result_tolerance);
    BOOST_CHECK_CLOSE(data[3], -56.2081642f, result_tolerance);
    BOOST_CHECK_CLOSE(data[4], -8.26997375f, result_tolerance);
    BOOST_CHECK_CLOSE(data[5],  6.55344391f, result_tolerance);
    BOOST_CHECK_CLOSE(data[6],  35.7729492f, result_tolerance);
    BOOST_CHECK_CLOSE(data[7],  35.8592834f, result_tolerance);
    BOOST_CHECK_CLOSE(data[8],  51.1210632f, result_tolerance);
    BOOST_CHECK_CLOSE(data[9],  86.9385834f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( large_ready_sorted_insertion_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(ready_sorted(data_size));

        /* Sort the data */
        auto t0(std::chrono::system_clock::now());
        insertion_sort(data.data(), data.size() - 1);
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_ready_sorted_insertion_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 0.0f; i < data.size(); ++i)
        {
            passed &= (data[i] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_inverse_sorted_insertion_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= (large_test_size_from << 1); data_size <<= 1)   /* You should be pretty bored by now */
    {
        /* Generate data */
        auto data(inverse_sorted(data_size));

        /* Sort the data */
        auto t0(std::chrono::system_clock::now());
        insertion_sort(data.data(), data.size() - 1);
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_inverse_sorted_insertion_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i <= data.size(); ++i)
        {
            passed &= (data[i - 1] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_random_insertion_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= (large_test_size_from << 1); data_size <<= 1)   /* You should be pretty bored by now */
    {
        /* Generate data */
        auto data(random(data_size));

        /* Sort the data */
        auto t0(std::chrono::system_clock::now());
        insertion_sort(data.data(), data.size() - 1);
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_random_insertion_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i < data.size(); ++i)
        {
            passed &= (data[i] >= data[i - 1]);
        }
        BOOST_CHECK(passed);
    }
}

/* Quick sort tests */
BOOST_AUTO_TEST_CASE( small_sample_quick_sort_test )
{
    quick_sort(small_sample.data(), 0, small_sample.size() - 1);
    BOOST_CHECK(small_sample[0] == -4.3f);
    BOOST_CHECK(small_sample[1] == -1.5f);
    BOOST_CHECK(small_sample[2] == -1.0f);
    BOOST_CHECK(small_sample[3] ==  1.0f);
    BOOST_CHECK(small_sample[4] ==  3.7f);
    BOOST_CHECK(small_sample[5] ==  5.4f);
    BOOST_CHECK(small_sample[6] ==  10.9f);
    BOOST_CHECK(small_sample[7] ==  10.9f);
}

BOOST_AUTO_TEST_CASE( small_ready_sorted_quick_sort_test )
{
    auto data(ready_sorted(10));
    quick_sort(data.data(), 0, data.size() - 1);
    BOOST_CHECK(data[0] == 0.0f);
    BOOST_CHECK(data[1] == 1.0f);
    BOOST_CHECK(data[2] == 2.0f);
    BOOST_CHECK(data[3] == 3.0f);
    BOOST_CHECK(data[4] == 4.0f);
    BOOST_CHECK(data[5] == 5.0f);
    BOOST_CHECK(data[6] == 6.0f);
    BOOST_CHECK(data[7] == 7.0f);
    BOOST_CHECK(data[8] == 8.0f);
    BOOST_CHECK(data[9] == 9.0f);
}

BOOST_AUTO_TEST_CASE( small_inverse_sorted_quick_sort_test )
{
    auto data(inverse_sorted(10));
    quick_sort(data.data(), 0, data.size() - 1);
    BOOST_CHECK(data[0] ==  1.0f);
    BOOST_CHECK(data[1] ==  2.0f);
    BOOST_CHECK(data[2] ==  3.0f);
    BOOST_CHECK(data[3] ==  4.0f);
    BOOST_CHECK(data[4] ==  5.0f);
    BOOST_CHECK(data[5] ==  6.0f);
    BOOST_CHECK(data[6] ==  7.0f);
    BOOST_CHECK(data[7] ==  8.0f);
    BOOST_CHECK(data[8] ==  9.0f);
    BOOST_CHECK(data[9] == 10.0f);
}

BOOST_AUTO_TEST_CASE( small_random_quick_sort_test )
{
    auto data(random(10));
    quick_sort(data.data(), 0, data.size() - 1);
    BOOST_CHECK_CLOSE(data[0], -99.998436f,  result_tolerance);
    BOOST_CHECK_CLOSE(data[1], -90.5910797f, result_tolerance);
    BOOST_CHECK_CLOSE(data[2], -73.6924438f, result_tolerance);
    BOOST_CHECK_CLOSE(data[3], -56.2081642f, result_tolerance);
    BOOST_CHECK_CLOSE(data[4], -8.26997375f, result_tolerance);
    BOOST_CHECK_CLOSE(data[5],  6.55344391f, result_tolerance);
    BOOST_CHECK_CLOSE(data[6],  35.7729492f, result_tolerance);
    BOOST_CHECK_CLOSE(data[7],  35.8592834f, result_tolerance);
    BOOST_CHECK_CLOSE(data[8],  51.1210632f, result_tolerance);
    BOOST_CHECK_CLOSE(data[9],  86.9385834f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( large_ready_sorted_quick_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_from; data_size <<= 1)   /* You should be pretty bored by now */
    {
        /* Generate data */
        auto data(ready_sorted(data_size));

        /* Sort the data */
        auto t0(std::chrono::system_clock::now());
        quick_sort(data.data(), 0, data.size() - 1);
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_ready_sorted_quick_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 0.0f; i < data.size(); ++i)
        {
            passed &= (data[i] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_inverse_sorted_quick_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_from; data_size <<= 1)   /* You should be pretty bored by now */
    {
        /* Generate data */
        auto data(inverse_sorted(data_size));

        /* Sort the data */
        auto t0(std::chrono::system_clock::now());
        quick_sort(data.data(), 0, data.size() - 1);
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_inverse_sorted_quick_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i <= data.size(); ++i)
        {
            passed &= (data[i - 1] == i);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_CASE( large_random_quick_sort_test )
{
    for (int data_size = large_test_size_from; data_size <= large_test_size_to; data_size <<= 1)
    {
        /* Generate data */
        auto data(random(data_size));

        /* Sort the data */
        auto t0(std::chrono::system_clock::now());
        quick_sort(data.data(), 0, data.size() - 1);
        auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: large_random_quick_sort_test " << data.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        /* Checks */
        bool passed = true;
        for (float i = 1.0f; i < data.size(); ++i)
        {
            passed &= (data[i] >= data[i - 1]);
        }
        BOOST_CHECK(passed);
    }
}

BOOST_AUTO_TEST_SUITE_END()
}; // namespace test
}; // namespace raptor_raytracer
