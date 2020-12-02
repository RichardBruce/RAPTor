project (raptor_physics)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/colliders ${PROJECT_SOURCE_DIR}/integrators ${PROJECT_SOURCE_DIR}/forces)
include_directories(
    "${HEADERS}"
    $ENV{RAPTOR_HOME}/sdl_wrappers
    $ENV{RAPTOR_HOME}/networking
    ${Boost_INCLUDE_DIRS}
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/include
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/include/SDL2
    $ENV{LIBARYS_PATH}/SDL2_image-$ENV{SDLIMAGE_VER}/include/SDL2
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/include/tbb
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/include
    $ENV{LIBARYS_PATH}/fftw-$ENV{FFTW_VER}/include)

# Source
set(SOURCE
    isosurface.cc
    physics_engine.cc
    polygon.cc
    simulation_environment.cc
    gjk.cc
    lcp_solver.cc
    physics_common.cc
    physics_object.cc
    simplex.cc
    vertex_group.cc)

# Libraries
set (LIBS raptor_raytracer::shared raptor_raytracer::headers SDL2 SDL2_ttf SDL2_image tbb pthread ${Boost_LIBRARIES} fftw3f)
link_directories( 
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_image-$ENV{SDLIMAGE_VER}/lib
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/build/build_release
    $ENV{LIBARYS_PATH}/fftw-$ENV{FFTW_VER}/lib
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/lib)

# Tests
set(UNIT_TESTS
    collision_info_tests
    contact_graph_tests
    exact_collision_time_tests
    force_tests
    gjk_tests
    inertia_tensor_tests
    integrator_tests
    lcp_solver_tests
    object_bound_tests
    pair_manager_tests
    particle_tests
    physics_engine_tests
    physics_object_tests
    physics_options_tests
    polygon_tests
    rigid_body_collider_tests
    simplex_tests
    simulation_environment_tests
    spatial_sub_division_tests
    tracking_info_tests
    vertex_group_tests)
add_unit_test("${UNIT_TESTS}")

set(REGRESSION_TESTS
    dropping_tests
    free_motion_tests  
    stacking_tests)
add_regression_test("${REGRESSION_TESTS}")

# Generate Outputs
add_exe_and_libs("${HEADERS}" "${SOURCE}" "${LIBS}")

# Cpp checks
add_analysis("${SOURCE}" "${HEADERS}")
