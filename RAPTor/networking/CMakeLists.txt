project (raptor_networking)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR})
include_directories(
    "${HEADERS}"
    $ENV{RAPTOR_HOME}/sdl_wrappers
    ${Boost_INCLUDE_DIRS}
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/include
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/include/SDL2
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/include)

# Source
set(SOURCE
    protocol_stack.cc
    ../sdl_wrappers/sdl_wrapper.cc
    ../sdl_wrappers/sdl_event_handler_factory.cc)

# Libraries
set (LIBS raptor_raytracer::shared raptor_raytracer::headers SDL2 SDL2_ttf tbb ${Boost_LIBRARIES} fftw3f)
link_directories( 
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_image-$ENV{SDLIMAGE_VER}/lib
    $ENV{LIBARYS_PATH}/tbb$ENV{TBB_VER}/build/build_release
    $ENV{LIBARYS_PATH}/fftw-$ENV{FFTW_VER}/lib
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/lib)

# Tests
set(UNIT_TESTS
    ack_retransmission_tests
    batcher_tests
    drop_late_sequencer_tests
    fragmenter_tests
    group_tests
    message_delivery_tests
    msg_data_tests
    msg_header_tests
    nack_retransmission_tests
    rate_limiter_tests
    reorder_sequencer_tests
    stack_controller_tests
    tcp_connection_tests
    udp_connection_tests
    vector_stream_tests)
add_unit_test("${UNIT_TESTS}")

# Generate Outputs
add_exe_and_libs("${HEADERS}" "${SOURCE}" "${LIBS}")

# Cpp checks
add_analysis("${SOURCE}" "${HEADERS}")
