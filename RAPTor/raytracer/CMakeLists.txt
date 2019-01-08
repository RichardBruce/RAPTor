project (raptor_raytracer)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/spatial_sub_division ${PROJECT_SOURCE_DIR}/materials ${PROJECT_SOURCE_DIR}/parsers ${PROJECT_SOURCE_DIR}/parsers/mgflib)
include_directories(
    "${HEADERS}"
    $ENV{RAPTOR_HOME}/sdl_wrappers
    ${Boost_INCLUDE_DIRS}
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/include
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/include/SDL2
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/include/tbb
    $ENV{LIBARYS_PATH}/fftw-$ENV{FFTW_VER}/include
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/include)

# Source
set(SOURCE
    raytracer.cc
    ray.cc
    scene.cc
    sort.cc
    line.cc
    packet_ray.cc
    camera.cc
    raytracer_event_handler_factory.cc
    ../sdl_wrappers/sdl_wrapper.cc
    ../sdl_wrappers/sdl_event_handler_factory.cc
    spatial_sub_division/kd_tree.cc
    spatial_sub_division/kdt_builder.cc
    spatial_sub_division/voxel.cc
    spatial_sub_division/bih.cc
    spatial_sub_division/bih_builder.cc
    spatial_sub_division/bvh.cc
    spatial_sub_division/bvh_builder.cc
    materials/phong_shader.cc
    materials/cook_torrance_cxy.cc
    materials/mandelbrot_shader.cc
    materials/mapper_shader.cc
    materials/mapper_falloff.cc
    materials/gradient_mapper.cc
    materials/checker_board_mapper.cc
    materials/perlin_noise_2d_mapper.cc
    materials/perlin_noise_3d_mapper.cc
    materials/planar_mapper.cc
    materials/cylindrical_mapper.cc
    materials/cubic_mapper.cc
    materials/coloured_mapper_shader.cc
    parsers/picture_functions.cc
    parsers/normal_calculator.cc
    parsers/cfg_parser.cc
    parsers/nff_parser.cc
    parsers/mgf_parser.cc
    parsers/lwo_parser.cc
    parsers/lwo1_parser.cc
    parsers/lwo2_parser.cc
    parsers/lwo_surf.cc
    parsers/lwo_bloks.cc
    parsers/obj_parser.cc
    parsers/off_parser.cc
    parsers/ply_parser.cc
    parsers/vrml_parser.cc
    parsers/mgflib/parser.c
    parsers/mgflib/badarg.c
    parsers/mgflib/lookup.c
    parsers/mgflib/context.c
    parsers/mgflib/vect.c
    parsers/mgflib/words.c
    parsers/mgflib/fvect.c
    parsers/mgflib/object.c
    parsers/mgflib/xf.c)

# Libraries
set (LIBS raptor_common::headers raptor_common::static SDL2 SDL2_ttf tbb tbbmalloc jpeg png tga pthread ${Boost_LIBRARIES} fftw3f)
link_directories( 
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/lib
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/build/build_release
    $ENV{LIBARYS_PATH}/fftw-$ENV{FFTW_VER}/lib
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/lib)

# Tests
set(UNIT_TESTS
    bih_block_tests
    bvh_node_tests
    simd_tests
    vint_tests
    bih_builder_tests
    sort_tests
    voxel_tests
    bih_node_tests
    normal_calculator_tests
    texture_mapper_tests
    bih_tests
    primitive_store_tests
    vfp_tests)
add_unit_test("${UNIT_TESTS}")

set(REGRESSION_TESTS
    cfg_tests
    lwo_tests
    mgf_tests
    nff_tests
    obj_tests
    off_tests
    ply_tests
    vrml_tests)
add_regression_test("${REGRESSION_TESTS}")

# Generate Outputs
add_exe_and_libs("${HEADERS}" "${SOURCE}" "${LIBS}")

# Cpp checks
add_analysis("${SOURCE}" "${HEADERS}")
