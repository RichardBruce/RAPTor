#ifdef STAND_ALONE
#define BOOST_TEST_MODULE mgf test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Test headers */
#include "regression_fixture.h"


BOOST_AUTO_TEST_SUITE( mgf_tests )

BOOST_AUTO_TEST_CASE( mgf_blkchair_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/blkchair.mgf", mgf, point_t(0.0, 1.43007, 1.21751), point_t(0.987689, 0.110616, 0.110616), point_t(0.0, -0.707107, 0.707107), point_t(0.156435, -0.698401, -0.698401), ext_colour_t(255.0, 255.0, 255.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_bluchair_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/bluchair.mgf", mgf, point_t(-0.719024, 1.9063, 2.08393), point_t(0.999504, 0.0236753, 0.0207694), point_t(0.00206248, -0.608845, 0.793286), point_t(0.0314266, -0.792936, -0.608495));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_book_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/book.mgf", mgf, point_t(39.7714, 28.5208, -7.42805), point_t(0.516746, -0.126593, 0.846728), point_t(0.276531, 0.911316, 0.305013), point_t(0.81025, -0.391761, 0.435912));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 200.0,  150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_bookbox_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/bookbox.mgf", mgf, point_t(-0.537066, 0.770168, 0.879083), point_t(0.730342, 0.658092, 0.183074), point_t(0.198922, -0.4613, 0.864658), point_t(0.653476, -0.595078, -0.467816));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_butterfly_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/butterfly.mgf", mgf, point_t( 0.0, -2.0, 1.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.0, 1.0), point_t(0.0, 1.0, 0.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, -150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_cabin_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/cabin.mgf", mgf, point_t(-3.70266, 7.68787, 3.25664), point_t(0.453991, 0.891006, 2.6261e-08), point_t(0.194367, -0.099035, 0.975917), point_t(0.869548, -0.443057, -0.218143));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, -150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_chair1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/chair1.mgf", mgf, point_t(14.6596, -35.9896, 41.0), point_t(-0.876307, 0.429246, -0.218712), point_t(0.0, 0.45399, 0.89100), point_t(0.481754, 0.780795, -0.397835), ext_colour_t(255.0, 255.0, 255.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, -150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_chair2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/chair2.mgf", mgf, point_t(26.1625, 5.59313, 45.7324), point_t(-0.0294118, 0.999444, 0.0157273), point_t(0.48177, 0.000388113, 0.876298), point_t(0.875804, 0.0333504, -0.481513));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_coatrack_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/coatrack.mgf", mgf, point_t(-1.87929, -0.00197327, 1.76666), point_t(0.061678, 0.998027, -0.0117658), point_t(0.338435, -0.0098226, 0.940939), point_t(0.938966, -0.0620175, -0.338373), ext_colour_t(255.0, 255.0, 255.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_conftabl_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/conftabl.mgf", mgf, point_t(-2.22321, 3.0, 2.07992), point_t(0.484831, -0.228144, 0.425779), point_t(0.0, 0.904827, 0.763971), point_t(0.763971, -0.535827, -0.359497));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 200.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_coord_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/coord.mgf", mgf, point_t(0.0, 0.0, -2.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, 1.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_corndesk_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/corndesk.mgf", mgf, point_t(0.214223, 2.11083, 1.82584), point_t(0.786019, 0.608753, -0.107678), point_t(0.453443, -0.449331, 0.769734), point_t(0.420195, -0.653851, -0.629217));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_curtain_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/curtain.mgf", mgf, point_t(0.0, 10.8309, -2.70448), point_t(0.987688, 0.117343, -0.103452), point_t(0.0, 0.661312, 0.750111), point_t(0.156434, -0.740876, 0.65317), ext_colour_t(255.0, 255.0, 255.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_dafvase_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/dafvase.mgf", mgf, point_t(-5.25516, 34.9719, 21.3347), point_t(0.890567, 0.453766, 0.0314108), point_t(-0.0279872, -0.0142602, 0.999507), point_t(0.45399, -0.891007, 0.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_desk_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/desk.mgf", mgf, point_t(-0.547533, 1.82708, 1.29237), point_t(0.710502, 0.688418, 0.145836), point_t(0.0936528, -0.297905, 0.94999), point_t(0.697436, -0.661312, -0.276134));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_desklamp_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/desklamp.mgf", mgf, point_t(45.8079, -1.47508, 48.6225), point_t(0.0, -1.0, 0.0), point_t(-0.425779, 0.0, 0.904827), point_t(-0.904827, 0.0, -0.425779));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_doorclsr_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/doorclsr.mgf", mgf, point_t(0.204108, -0.338688, 0.304267), point_t(-0.101397, 0.587785, 0.802638), point_t(0.992115, 0.0, 0.125333), point_t(0.073669, 0.809017, -0.58315));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, -150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_dresser_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/dresser.mgf", mgf, point_t(-0.0106744, 1.89196, 3.71793), point_t(0.755893, -0.00910099, 0.0125264), point_t(-0.0185809, -0.197908, 0.977455), point_t(0.00574466, -0.609557, -0.789523));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_exitsign_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/exitsign.mgf", mgf, point_t(0.366792, 0.469927, 0.606283), point_t(0.965389, -0.214648, -0.148154), point_t(-0.0214258, -0.631396, 0.775165), point_t(-0.259931, -0.745162, -0.614142));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_exting_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/exting.mgf", mgf, point_t(-0.388678, 1.12059, 0.821471), point_t(0.905996, 0.423278, -0.00227795), point_t(0.144844, -0.304963, 0.941285), point_t(0.397731, -0.853131, -0.337605));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_medcab_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/medcab.mgf", mgf, point_t(-0.861509, 1.93334, -0.523312), point_t(-0.0210184, 0.481269, 0.876321), point_t(-0.769469, -0.567423, 0.293169), point_t(0.638338, -0.66814, 0.382248));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 200.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_mug_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/mug.mgf", mgf, point_t(17.7387, 13.4915, -2.61691), point_t(0.156434, 0.0, -0.987688), point_t(0.305212, 0.951057, 0.048340), point_t(0.939347, -0.309017, 0.148778));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_office_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/office.mgf", mgf, point_t(1.0, 2.91919, -0.27595), point_t(0.187381, 0.0, 0.982287), point_t(-0.184062, 0.982287, 0.035111), point_t(-0.964888, -0.187381, 0.184062));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-1.0, 3.5, -0.27595), 0.0, 0.1);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_openbook_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/openbook.mgf", mgf, point_t(18.0, 36.6897, 28.1936), point_t(1.0, 0.0, 0.0), point_t(0.0, -0.790155, 0.612907), point_t(0.0, -0.612907, -0.790155));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

              
BOOST_AUTO_TEST_CASE( mgf_orgchair_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/orgchair.mgf", mgf, point_t(-0.305172, 1.51552, 1.35142), point_t(0.892794, 0.449583, 0.0281915), point_t(0.215373, -0.480984, 0.849864), point_t(0.395644, -0.752681, -0.526248));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_pencil_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/pencil.mgf", mgf, point_t(15.0, 15.0, -25.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, 1.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_podium_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/podium.mgf", mgf, point_t(-0.25651, -1.22294, 1.695), point_t(-0.919978, 0.369745, -0.130109), point_t(0.0770206, 0.495989, 0.864906), point_t(0.384328, 0.785674, -0.484777));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, -150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_redchair_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/redchair.mgf", mgf, point_t(0.0, 2.00789, 1.12533), point_t(0.992115, 0.124777, 0.0117949), point_t(0.0, -0.0941083, 0.995562), point_t(0.125333, -0.987712, -0.0933662));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_soda_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/soda.mgf", mgf, point_t(1.0, 1.0, -2.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, 1.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(2.0, 2.0, 0.0), 0.0,  0.1);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_speaker_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/speaker.mgf", mgf, point_t(0.5, 0.5, -1.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, 1.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_terminal_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/terminal.mgf", mgf, point_t(0.45, 0.75012, 0.5), point_t(1.0, 0.0, 0.0), point_t(0.0, -0.368125, 0.929776), point_t(0.0, -0.929776, -0.368125));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_toilet_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/toilet.mgf", mgf, point_t(0.0, 1.84864, 0.923168), point_t(0.975917, 0.207467, 0.06741), point_t(0.0, -0.309017, 0.951057), point_t(0.21814, -0.928152, -0.301575));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-100.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_vanity_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/vanity.mgf", mgf, point_t(0.0, 1.0747, 1.58528), point_t(1.0, 0.0, 0.0), point_t(0.0, -0.684547, 0.728969), point_t(0.0, -0.728969, -0.684547));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_vent_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/vent.mgf", mgf, point_t(24.0, 25.3508, -1.17269), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.425779, 0.904827), point_t(0.0, -0.904827, 0.425779), ext_colour_t(255.0, 255.0, 255.0));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_whale2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/whale2.mgf", mgf, point_t(9.35143, 7.03141, -15.8402), point_t(0.0184628, -0.999507, 0.0254118), point_t(0.587495, 0.0314108, 0.808618), point_t(-0.809017, 0.0, 0.587785));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 150.0, -500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( mgf_wstbskt_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/mgf_scenes/wstbskt.mgf", mgf, point_t(-0.292811, 0.545782, 1.47107), point_t(0.407077, 0.906157, -0.114752), point_t(0.779144, -0.278929, 0.561367), point_t(0.476679, -0.317928, -0.819572));
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-200.0, 150.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_SUITE_END()

// ./raytracer --mgf scenes/mgf_scenes/cruiser/shipbatl.mgf --cam 11.5974 6.94461 1.23065 --dx 0.0167102 0.999227 0.0355783 --dy 0.0428478 -0.00725486 0.999056 --dz -0.994753 0.0130228 0.101481 --light 6.0 6.94461 1.23065 0.001 255 255 255 0.0 --light 12.0 6.94461 1.23065 0.001 255 255 255 0.0 --res 1920 1080
// ./raytracer --mgf scenes/mgf_scenes/cruiser/shipemer.mgf --cam 1.70543 6.82824 1.25482 --dx 0.0115531 -0.999512 0.0290114 --dy 0.0 0.0 1.0 --dz 0.99819 0.0132403 0.0586675 --light 3.0 6.82824 1.2 0.00001 255 255 255 0.0 --res 1920 1080
// ./raytracer --mgf scenes/mgf_scenes/cruiser/shipfull.mgf --cam 12.6852 7.17522 1.27253 --dx -0.443838 -0.895868 -0.0206808 --dy 0.022483 -0.06655 0.99753 --dz 0.895941 -0.444082 0.00899294 --light 14.0 7.09337 1.40625 0.0001 255 255 255 0.0 --res 1920 1080
// ./raytracer --mgf scenes/mgf_scenes/furn.mgf --cam 11.5296 6.90595 2.77921 --dx 0.675349 -0.719947 0.15993 --dy -0.456826 -0.238132 0.857087 --dz -0.578974 -0.651894 -0.489713 --light 11 10.0 15.9031 10 255 255 255 0.0 --light -11 -10.0 15.9031 10 255 255 255 0.0 --bg 255 255 255
// ./raytracer --mgf scenes/mgf_scenes/bathroom.mgf --cam 0.690096 2.25 1.72998 --dx 0.984685 -0.173215 0.0196139 --dy -0.067297 -0.273931 0.95939 --dz -0.160808 -0.946019 -0.281393 --light 0.690096 1.5 1.72998 0.0001 255 255 255 0.0 --res 1920 1080
// ./raytracer --mgf ./scenes/mgf_scenes/conf.mgf --cam 0.755185 1.96957 0.999216 --dx -0.244815 0.969566 -0.000783627 --dy -0.125365 -0.0308523 0.991629 --dz 0.961429 0.242865 0.129102 --light 5.0 2.0 1.5 0.00001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene0.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.9 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene1.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.9 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene2.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.9 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene3.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 1.5 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene4.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 1.5 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene5.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.0 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene6.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.0 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene7.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.0 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene8.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.9 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene9.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.0 0 0.001 255 255 255 0.0
// ./raytracer --mgf scenes/mgf_scenes/ERW5/scene10.mgf --cam 2.60096 2.04336 -2.79948 --dx 0.728969 0 0.684547 --dy -0.212815 0.950448 0.226625 --dz -0.650626 -0.310884 0.692847 --light 0 2.0 0 0.001 255 255 255 0.0
