#ifdef STAND_ALONE
#define BOOST_TEST_MODULE clipping test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <list>
#include <memory>
#include <vector>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"
#include "physics_common.h"


namespace raptor_physics
{
namespace test
{
BOOST_AUTO_TEST_SUITE( clipping_tests )

const float result_tolerance = 0.0001;

BOOST_AUTO_TEST_CASE( clipping_square0_test )
{
    /* Square test 0 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 1.5,  1.5, 1.0),
        point_t( 1.5, -0.5, 1.0),
        point_t(-0.5, -0.5, 1.0),
        point_t(-0.5,  1.5, 1.0)
    }));    

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  1.0, 0.0),
        point_t( 1.0, -1.0, 0.0),
        point_t(-1.0, -1.0, 0.0),
        point_t(-1.0,  1.0, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t( 1.0,  1.0, 1.0),
        point_t( 1.0, -0.5, 1.0),
        point_t(-0.5, -0.5, 1.0),
        point_t(-0.5,  1.0, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_square1_test )
{
    /* Square test 1 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 0.0,  1.0, 1.0),
        point_t( 0.0, -1.0, 1.0),
        point_t(-2.0, -1.0, 1.0),
        point_t(-2.0,  1.0, 1.0)
    }));    

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  1.0, 0.0),
        point_t( 1.0, -1.0, 0.0),
        point_t(-1.0, -1.0, 0.0),
        point_t(-1.0,  1.0, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(-1.0,  1.0, 1.0),
        point_t( 0.0,  1.0, 1.0),
        point_t( 0.0, -1.0, 1.0),
        point_t(-1.0, -1.0, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_square2_test )
{
    /* Square test 2 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 0.0,  1.1, 1.0),
        point_t( 0.0, -1.1, 1.0),
        point_t(-2.0, -1.1, 1.0),
        point_t(-2.0,  1.1, 1.0)
    }));    

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  1.0, 0.0),
        point_t( 1.0, -1.0, 0.0),
        point_t(-1.0, -1.0, 0.0),
        point_t(-1.0,  1.0, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(-1.0,  1.0, 1.0),
        point_t( 0.0,  1.0, 1.0),
        point_t( 0.0, -1.0, 1.0),
        point_t(-1.0, -1.0, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_square3_test )
{
    /* Square test 3 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 0.5,  1.5, 1.0),
        point_t( 0.5, -1.5, 1.0),
        point_t(-0.5, -1.5, 1.0),
        point_t(-0.5,  1.5, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  1.0, 0.0),
        point_t( 1.0, -1.0, 0.0),
        point_t(-1.0, -1.0, 0.0),
        point_t(-1.0,  1.0, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t( 0.5,  1.0, 1.0),
        point_t( 0.5, -1.0, 1.0),
        point_t(-0.5, -1.0, 1.0),
        point_t(-0.5,  1.0, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_square4_test )
{
    /* Square test 4, poly a inside poly b */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 0.5,  0.5, 1.0),
        point_t( 0.5, -0.5, 1.0),
        point_t(-0.5, -0.5, 1.0),
        point_t(-0.5,  0.5, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  1.0, 0.0),
        point_t( 1.0, -1.0, 0.0),
        point_t(-1.0, -1.0, 0.0),
        point_t(-1.0,  1.0, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t( 0.5,  0.5, 1.0),
        point_t( 0.5, -0.5, 1.0),
        point_t(-0.5, -0.5, 1.0),
        point_t(-0.5,  0.5, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_square5_test )
{
    /* Square test 5, poly b inside poly a */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 1.0,  1.0, 1.0),
        point_t( 1.0, -1.0, 1.0),
        point_t(-1.0, -1.0, 1.0),
        point_t(-1.0,  1.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 0.5,  0.5, 0.0),
        point_t( 0.5, -0.5, 0.0),
        point_t(-0.5, -0.5, 0.0),
        point_t(-0.5,  0.5, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(-0.5,  0.5, 1.0),
        point_t( 0.5,  0.5, 1.0),
        point_t( 0.5, -0.5, 1.0),
        point_t(-0.5, -0.5, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_square6_test )
{
    /* Square test 5, poly b inside poly a */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 1.0,  1.0, 1.0),
        point_t( 1.0, -1.0, 1.0),
        point_t(-1.0, -1.0, 1.0),
        point_t(-1.0,  1.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 0.0,  1.1, 0.0),
        point_t( 1.1,  0.0, 0.0),
        point_t( 0.0, -1.1, 0.0),
        point_t(-1.1,  0.0, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(-0.1,  1.0, 1.0),
        point_t( 0.1,  1.0, 1.0),
        point_t( 1.0,  0.1, 1.0),
        point_t( 1.0, -0.1, 1.0),
        point_t( 0.1, -1.0, 1.0),
        point_t(-0.1, -1.0, 1.0),
        point_t(-1.0, -0.1, 1.0),
        point_t(-1.0,  0.1, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_triangle_square0_test )
{
    /* Square test 3 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 0.5,  1.5, 1.0),
        point_t( 0.5, -1.5, 1.0),
        point_t(-0.5, -1.5, 1.0),
        point_t(-0.5,  1.5, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 0.0,  1.0, 0.0),
        point_t( 0.5, -2.0, 0.0),
        point_t(-0.5, -2.0, 0.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(  0.0,                 1.0, 1.0),
        point_t(  0.5 - (1.0 / 12.0), -1.5, 1.0),
        point_t( -0.5 + (1.0 / 12.0), -1.5, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_octogon0_test )
{
    /* Octogon test 0 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 2.0,  5.0, 1.0),
        point_t( 3.0,  4.0, 1.0),
        point_t( 3.0,  2.0, 1.0),
        point_t( 2.0,  1.0, 1.0),
        point_t( 0.0,  1.0, 1.0),
        point_t(-1.0,  2.0, 1.0),
        point_t(-1.0,  4.0, 1.0),
        point_t( 0.0,  5.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  2.0, 1.0),
        point_t( 2.0,  1.0, 1.0),
        point_t( 2.0, -1.0, 1.0),
        point_t( 1.0, -2.0, 1.0),
        point_t(-1.0, -2.0, 1.0),
        point_t(-2.0, -1.0, 1.0),
        point_t(-2.0,  1.0, 1.0),
        point_t(-1.0,  2.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(-1.0,  2.0, 1.0),
        point_t( 1.0,  2.0, 1.0),
        point_t( 2.0,  1.0, 1.0),
        point_t( 0.0,  1.0, 1.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_octogon1_test )
{
    /* Octogon test 1 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 0.25,  4.0, 2.0),
        point_t( 0.50,  3.0, 2.0),
        point_t( 0.50, -3.0, 2.0),
        point_t( 0.25, -4.0, 2.0),
        point_t(-0.25, -4.0, 2.0),
        point_t(-0.50, -3.0, 2.0),
        point_t(-0.50,  3.0, 2.0),
        point_t(-0.25,  4.0, 2.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  2.0, 1.0),
        point_t( 2.0,  1.0, 1.0),
        point_t( 2.0, -1.0, 1.0),
        point_t( 1.0, -2.0, 1.0),
        point_t(-1.0, -2.0, 1.0),
        point_t(-2.0, -1.0, 1.0),
        point_t(-2.0,  1.0, 1.0),
        point_t(-1.0,  2.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t( 0.5,  2.0, 2.0),
        point_t( 0.5, -2.0, 2.0),
        point_t(-0.5, -2.0, 2.0),
        point_t(-0.5,  2.0, 2.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 2.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_octogon2_test )
{
    /* Octogon test 2 */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 4.25,  4.0, 2.0),
        point_t( 4.50,  3.0, 2.0),
        point_t( 4.50, -3.0, 2.0),
        point_t( 4.25, -4.0, 2.0),
        point_t(-0.25, -4.0, 2.0),
        point_t(-0.50, -3.0, 2.0),
        point_t(-0.50,  3.0, 2.0),
        point_t(-0.25,  4.0, 2.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  2.0, 1.0),
        point_t( 2.0,  1.0, 1.0),
        point_t( 2.0, -1.0, 1.0),
        point_t( 1.0, -2.0, 1.0),
        point_t(-1.0, -2.0, 1.0),
        point_t(-2.0, -1.0, 1.0),
        point_t(-2.0,  1.0, 1.0),
        point_t(-1.0,  2.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t( 1.0,  2.0, 2.0),
        point_t( 2.0,  1.0, 2.0),
        point_t( 2.0, -1.0, 2.0),
        point_t( 1.0, -2.0, 2.0),
        point_t(-0.5, -2.0, 2.0),
        point_t(-0.5,  2.0, 2.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_octogon3_test )
{
    /* Octogon test 3, poly a inside poly b */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 4.25,  4.0, 2.0),
        point_t( 4.50,  3.0, 2.0),
        point_t( 4.50, -3.0, 2.0),
        point_t( 4.25, -4.0, 2.0),
        point_t(-0.25, -4.0, 2.0),
        point_t(-0.50, -3.0, 2.0),
        point_t(-0.50,  3.0, 2.0),
        point_t(-0.25,  4.0, 2.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 5.25,  5.0, 1.0),
        point_t( 5.50,  4.0, 1.0),
        point_t( 5.50, -4.0, 1.0),
        point_t( 5.25, -5.0, 1.0),
        point_t(-1.25, -5.0, 1.0),
        point_t(-1.50, -4.0, 1.0),
        point_t(-1.50,  4.0, 1.0),
        point_t(-1.25,  5.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t( 4.25,  4.0, 2.0),
        point_t( 4.50,  3.0, 2.0),
        point_t( 4.50, -3.0, 2.0),
        point_t( 4.25, -4.0, 2.0),
        point_t(-0.25, -4.0, 2.0),
        point_t(-0.50, -3.0, 2.0),
        point_t(-0.50,  3.0, 2.0),
        point_t(-0.25,  4.0, 2.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_octogon4_test )
{
    /* Octogon test 4, poly b inside poly a */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 5.25,  5.0, 2.0),
        point_t( 5.50,  4.0, 2.0),
        point_t( 5.50, -4.0, 2.0),
        point_t( 5.25, -5.0, 2.0),
        point_t(-1.25, -5.0, 2.0),
        point_t(-1.50, -4.0, 2.0),
        point_t(-1.50,  4.0, 2.0),
        point_t(-1.25,  5.0, 2.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 4.25,  4.0, 1.0),
        point_t( 4.50,  3.0, 1.0),
        point_t( 4.50, -3.0, 1.0),
        point_t( 4.25, -4.0, 1.0),
        point_t(-0.25, -4.0, 1.0),
        point_t(-0.50, -3.0, 1.0),
        point_t(-0.50,  3.0, 1.0),
        point_t(-0.25,  4.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(-0.50,  3.0, 2.0),
        point_t(-0.25,  4.0, 2.0),
        point_t( 4.25,  4.0, 2.0),
        point_t( 4.50,  3.0, 2.0),
        point_t( 4.50, -3.0, 2.0),
        point_t( 4.25, -4.0, 2.0),
        point_t(-0.25, -4.0, 2.0),
        point_t(-0.50, -3.0, 2.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_octogon_square0_test )
{
    /* Octogon test 4, poly b inside poly a */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 5.25,  5.0, 2.0),
        point_t( 5.50,  4.0, 2.0),
        point_t( 5.50, -4.0, 2.0),
        point_t( 5.25, -5.0, 2.0),
        point_t(-1.25, -5.0, 2.0),
        point_t(-1.50, -4.0, 2.0),
        point_t(-1.50,  4.0, 2.0),
        point_t(-1.25,  5.0, 2.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 1.0,  1.0, 1.0),
        point_t( 1.0, -1.0, 1.0),
        point_t(-1.0, -1.0, 1.0),
        point_t(-1.0,  1.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t(-1.0,  1.0, 2.0),
        point_t( 1.0,  1.0, 2.0),
        point_t( 1.0, -1.0, 2.0),
        point_t(-1.0, -1.0, 2.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}


BOOST_AUTO_TEST_CASE( clipping_octogon_square1_test )
{
    /* Octogon test 4, poly b inside poly a */
    std::unique_ptr<std::vector<point_t>> poly_a(new std::vector<point_t>({
        point_t( 5.25,  5.0, 2.0),
        point_t( 5.50,  4.0, 2.0),
        point_t( 5.50, -4.0, 2.0),
        point_t( 5.25, -5.0, 2.0),
        point_t(-1.25, -5.0, 2.0),
        point_t(-1.50, -4.0, 2.0),
        point_t(-1.50,  4.0, 2.0),
        point_t(-1.25,  5.0, 2.0)
    }));

    std::unique_ptr<std::vector<point_t>> poly_b(new std::vector<point_t>({
        point_t( 6.0,  6.0, 1.0),
        point_t( 6.0, -6.0, 1.0),
        point_t(-6.0, -6.0, 1.0),
        point_t(-6.0,  6.0, 1.0)
    }));

    std::unique_ptr<std::vector<point_t>> exp(new std::vector<point_t>({
        point_t( 5.25,  5.0, 2.0),
        point_t( 5.50,  4.0, 2.0),
        point_t( 5.50, -4.0, 2.0),
        point_t( 5.25, -5.0, 2.0),
        point_t(-1.25, -5.0, 2.0),
        point_t(-1.50, -4.0, 2.0),
        point_t(-1.50,  4.0, 2.0),
        point_t(-1.25,  5.0, 2.0)
    }));

    clip_polygon(poly_a.get(), *poly_b, point_t(0.0, 0.0, 1.0));

    /* Checks */
    BOOST_CHECK(poly_a->size() == exp->size());

    std::vector<point_t>::const_iterator i = poly_a->cbegin();
    for (auto &p : (*exp))
    {
        BOOST_CHECK(fabs(i->x - p.x) < result_tolerance);
        BOOST_CHECK(fabs(i->y - p.y) < result_tolerance);
        BOOST_CHECK(fabs(i->z - p.z) < result_tolerance);
        ++i;
    }
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
