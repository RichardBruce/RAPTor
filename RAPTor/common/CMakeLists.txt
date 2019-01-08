project (raptor_common)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR})
include_directories(
    "${HEADERS}")

# Source
set(SOURCE
    common.cc
    perlin_noise_3d.cc
    simplex_noise_2d.cc
    perlin_noise_2d.cc
    simd.cc
    simplex_noise_3d.cc)

# Libraries
set (LIBS)
link_directories()

# Tests


# Generate Outputs
add_libs("${HEADERS}" "${SOURCE}" "${LIBS}")
