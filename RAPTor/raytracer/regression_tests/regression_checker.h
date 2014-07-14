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

const float result_tolerance = 0.0005;
const int failure_limit = 1000;
const std::string test_data_location = "test_data/";


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
            std::unique_ptr<unsigned char []> expected(new unsigned char [image_size]);
            read_png_file(_expected, expected.get(), &exp_x_res, &exp_y_res);

            /* Check image dimensions, if this fails theres no point even trying */
            BOOST_ASSERT(x_res == exp_x_res);
            BOOST_ASSERT(y_res == exp_y_res);

            /* Compare */
            int failures = 0;
            std::unique_ptr<unsigned char []> difference(new unsigned char [image_size]);
            for (unsigned int i = 0; i < image_pixels; ++i)
            {
                /* Check */
                const unsigned int idx = i * 3;
                if (failures < failure_limit)
                {
                    BOOST_CHECK(actual[idx    ] == expected[idx    ]);
                    BOOST_CHECK(actual[idx + 1] == expected[idx + 1]);
                    BOOST_CHECK(actual[idx + 2] == expected[idx + 2]);
                }
                else if (failures == failure_limit)
                {
                    BOOST_LOG_TRIVIAL(warning) << failure_limit << " errors detected, I'm not checking anymore. Go fix your code";
                }

                /* Colour mismatch pixels red */
                if ((actual[idx] != expected[idx]) || (actual[idx + 1] != expected[idx + 1]) || (actual[idx + 2] != expected[idx + 2]))
                {
                    ++failures;
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
            }

            /* Write out the difference image */
            write_png_file(_difference, difference.get(), x_res, y_res);

            return *this;
        }

    private :
        const std::string _expected;
        const std::string _difference;
        const std::string _actual;
};

#endif /* #ifndef __REGRESSION_CHECKER_H__ */
