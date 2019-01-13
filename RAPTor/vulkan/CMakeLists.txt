project (raptor_vulkan)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR})
include_directories(
    "${HEADERS}"
    $ENV{RAPTOR_HOME}/sdl_wrappers
    ${Boost_INCLUDE_DIRS}
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/include
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/include/SDL2)

# Source
set(SOURCE
    ../sdl_wrappers/sdl_wrapper.cc
    ../sdl_wrappers/sdl_event_handler_factory.cc)

# Libraries
set (LIBS raptor_common::headers raptor_common::static SDL2 SDL2_ttf ${Boost_LIBRARIES} vulkan)
link_directories( 
    $ENV{LIBARYS_PATH}/SDL2-$ENV{SDL_VER}/lib
    $ENV{LIBARYS_PATH}/SDL2_ttf-$ENV{SDLTTF_VER}/lib)

# Tests

# Generate Outputs
add_exe_and_libs("${HEADERS}" "${SOURCE}" "${LIBS}")

# Shaders
set(SHADERS
	shaders/light.vert
	shaders/light.frag
	shaders/shader.vert
	shaders/shader.frag)
add_shaders("${SHADERS}")

install(DIRECTORY "textures" DESTINATION ${PROJECT_NAME}/bin)
install(DIRECTORY "models" DESTINATION ${PROJECT_NAME}/bin)

# Cpp checks
add_analysis("${SOURCE}" "${HEADERS}")
