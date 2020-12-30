project (raptor_convex_decomposition)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR})
include_directories(
    "${HEADERS}"
    ${Boost_INCLUDE_DIRS}
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/include/tbb)

# Source
set(SOURCE
    convex_decomposition.cc
    dac_convex_hull.cc
    voxel_set.cc
    tetrahedron_set.cc
    volume.cc)

# Libraries
set(LIBS raptor_raytracer::shared raptor_raytracer::headers raptor_parsers::headers  SDL2 SDL2_ttf tbb ${Boost_LIBRARIES} fftw3f)
link_directories( 
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/lib
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/build/build_release
    $ENV{LIBARYS_PATH}/fftw-$ENV{FFTW_VER}/lib
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/lib)

# Tests
set(UNIT_TESTS
    convex_decomposition_tests
    convex_mesh_tests
    dac_convex_hull_tests
    dac_convex_hull_2d_tests
    dac_convex_hull_3d_tests
    incremental_convex_hull_tests
    primitive_set_tests
    tetrahedron_set_tests
    tm_mesh_tests
    volume_tests
    voxel_set_tests)
add_unit_test("${UNIT_TESTS}")

set(REGRESSION_TESTS
    dac_convex_hull_3d_regression_tests
    tetrahedron_no_pca_no_hull_approx_tests
    tetrahedron_no_pca_hull_approx_tests
    tetrahedron_pca_hull_approx_tests
    voxel_no_pca_no_hull_approx_tests
    voxel_no_pca_hull_approx_simplify_tests
    voxel_pca_hull_approx_tests
    voxel_no_pca_hull_approx_tests)
add_regression_test("${REGRESSION_TESTS}")

# Generate Outputs
add_exe_and_libs("${HEADERS}" "${SOURCE}" "${LIBS}")

# Cpp checks
add_analysis("${SOURCE}" "${HEADERS}")
