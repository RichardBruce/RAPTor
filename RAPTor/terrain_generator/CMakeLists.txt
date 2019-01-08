project (raptor_terrain)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR})
include_directories(
    "${HEADERS}"
    $ENV{RAPTOR_HOME}/sdl_wrappers
    ${Boost_INCLUDE_DIRS}
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/include
    $ENV{LIBARYS_PATH}/SDL2_image-$ENV{SDLIMAGE_VER}/include/SDL2
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/include)

# Source
set(SOURCE)

# Libraries
set (LIBS raptor_raytracer::shared raptor_raytracer::headers raptor_physics::shared raptor_physics::headers SDL2 SDL2_ttf tbb ${Boost_LIBRARIES} fftw3f)
link_directories( 
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_image-$ENV{SDLIMAGE_VER}/lib
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/build/build_release
    $ENV{LIBARYS_PATH}/fftw-$ENV{FFTW_VER}/lib
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/lib)

# Tests
add_unit_test(grid_cell_tests)


# Generate Outputs
add_exe_and_libs("${HEADERS}" "${SOURCE}" "${LIBS}")
