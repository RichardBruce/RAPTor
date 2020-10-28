project (raptor_parsers)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR})
include_directories(
    "${HEADERS}"
    ${Boost_INCLUDE_DIRS}
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/include)

# Source
set(SOURCE)

# Libraries
set (LIBS raptor_common::headers jpeg png tga)
link_directories( 
    $ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/lib)

# Tests

# Generate Outputs
add_interface("${HEADERS}" "${SOURCE}" "${LIBS}")

# Cpp checks
add_analysis("${SOURCE}" "${HEADERS}")
