#ifndef __REGRESSION_CHECKER_H__
#define __REGRESSION_CHECKER_H__

/* Standard headers */
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"
#include "boost/filesystem.hpp"

/* Raytracer headers */
#include "camera.h"


using namespace raptor_raytracer;

const float result_tolerance            = 0.0005f;
const int count_error                   = 4;
const int pixel_error                   = 5;
const int failure_limit                 = 1000;
const std::string test_data_location    = "test_data/";


#define CREATE_REGRESSION_CHECKER(NAME)  regression_checker NAME( \
    boost::unit_test::framework::get<boost::unit_test::test_suite>(boost::unit_test::framework::current_test_case().p_parent_id).p_name, \
    boost::unit_test::framework::current_test_case().p_name);


/* Top level class to check the output of a regression test */
class regression_checker : private boost::noncopyable
{
    public :
        regression_checker(const std::string &suite, const std::string& test)
        : _expected(test_data_location + suite + "/" + test + "_exp.png"), _difference(test_data_location + suite + "/" +  test + "_diff.png"), _actual(test_data_location + suite + "/" +  test + "_act.png")
        {  }

        /* Allow default DTOR */

        regression_checker& check(const camera &c)
        {
            /* Write out actual picture */
            c.write_png_file(_actual);

            /* Get actual image */
            const unsigned int x_res = c.x_resolution();
            const unsigned int y_res = c.y_resolution();
            const unsigned int image_pixels = x_res * y_res;
            const unsigned int image_size = image_pixels * 3;

            std::unique_ptr<unsigned char []> actual(new unsigned char [image_size]);
            c.clip_image_to_rgb(actual.get());

            /* Load expected image */
            unsigned int exp_x_res;
            unsigned int exp_y_res;
            BOOST_REQUIRE(boost::filesystem::exists(_expected));
            std::unique_ptr<unsigned char []> expected(new unsigned char [image_size]);
            read_png_file(_expected, expected.get(), &exp_x_res, &exp_y_res);

            /* Check image dimensions, if this fails theres no point even trying */
            BOOST_REQUIRE(x_res == exp_x_res);
            BOOST_REQUIRE(y_res == exp_y_res);

            /* Compare */
            int failures = 0;
            std::unique_ptr<unsigned char []> difference(new unsigned char [image_size]);
            for (unsigned int i = 0; i < x_res; ++i)
            {
                for (unsigned int j = 0; j < y_res; ++j)
                {
                    /* Colour mismatch pixels red */
                    const unsigned int idx = ((i * y_res) + j) * 3;
                    if (test_pixel(actual.get(), expected.get(), idx))
                    {
                        difference[idx    ] = 255;
                        difference[idx + 1] = 0;
                        difference[idx + 2] = 0;
                    }
                    else
                    {
                        difference[idx    ] = actual[idx    ];
                        difference[idx + 1] = actual[idx + 1];
                        difference[idx + 2] = actual[idx + 2];
                    }

                    /* Check for an error a human might notice */
                    if (preceivable_error_check(actual.get(), expected.get(), i, j, x_res, y_res))
                    {
                        /* Check only the failing pixels or we get loads of team city logs */
                        if (failures < failure_limit)
                        {
                            ++failures;
                            BOOST_CHECK(actual[idx    ] == expected[idx    ]);
                            BOOST_CHECK(actual[idx + 1] == expected[idx + 1]);
                            BOOST_CHECK(actual[idx + 2] == expected[idx + 2]);
                        }
                        else if (failures == failure_limit)
                        {
                            ++failures;
                            BOOST_LOG_TRIVIAL(warning) << failure_limit << " errors detected, I'm not checking anymore. Go fix your code";
                        }
                    }
                }
            }

            /* Write out the difference image */
            write_png_file(_difference, difference.get(), x_res, y_res);

            return *this;
        }

    private :
        bool preceivable_error_check(const unsigned char *const actual, const unsigned char *const expected, const unsigned int i, const unsigned j, const unsigned int x_res, const unsigned int y_res)
        {
            /* Cant check the perimeter pixels */
            if ((i == 0) || (i == x_res - 1) || (j == 0) || (j == y_res - 1))
            {
                return false;
            }

            /* Check 3x3 surrounding pixels */
            const unsigned int c = (i * y_res) + j;
            int error_count = test_pixel(actual, expected, c * 3);
            if (!error_count)
            {
                return false;
            }

            const unsigned int u = c - y_res;
            error_count += test_pixel(actual, expected, u * 3);

            const unsigned int d = c + y_res;
            error_count += test_pixel(actual, expected, d * 3);

            const unsigned int l = c - 1;
            error_count += test_pixel(actual, expected, l * 3);

            const unsigned int r = c + 1;
            error_count += test_pixel(actual, expected, r * 3);

            const unsigned int ul = u - 1;
            error_count += test_pixel(actual, expected, ul * 3);

            const unsigned int ur = u + 1;
            error_count += test_pixel(actual, expected, ur * 3);

            const unsigned int dl = d - 1;
            error_count += test_pixel(actual, expected, dl * 3);

            const unsigned int dr = d + 1;
            error_count += test_pixel(actual, expected, dr * 3);

            return error_count > count_error;
        }

        bool test_pixel(const unsigned char *const actual, const unsigned char *const expected, const unsigned int idx)
        {
            return ((abs(actual[idx    ] - expected[idx    ]) > pixel_error) ||
                    (abs(actual[idx + 1] - expected[idx + 1]) > pixel_error) ||
                    (abs(actual[idx + 2] - expected[idx + 2]) > pixel_error));
        }

        const std::string _expected;
        const std::string _difference;
        const std::string _actual;
};

#endif /* #ifndef __REGRESSION_CHECKER_H__ */
