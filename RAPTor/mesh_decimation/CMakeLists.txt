project (raptor_mesh_decimation)

# Headers
set(HEADERS ${PROJECT_SOURCE_DIR})
include_directories(
    "${HEADERS}"
    ${Boost_INCLUDE_DIRS})

# Source
set(SOURCE
    mesh_decimation.cc)

# Libraries
set (LIBS raptor_parsers::headers raptor_common::static raptor_common::headers jpeg png tga ${Boost_LIBRARIES})
link_directories($ENV{LIBARYS_PATH}/libtga-$ENV{LIBTGA_VER}/lib )

# Tests
set(REGRESSION_TESTS
    cumulative_cost_1_0_tests
    cumulative_cost_0_1_tests
    cumulative_cost_0_01_tests
    cumulative_cost_0_001_tests
    cumulative_cost_0_00001_tests
    merge_cost_1_0_tests
    merge_cost_0_1_tests
    merge_cost_0_001_tests
    merge_cost_0_00001_tests
    merge_cost_0_000000001_tests
    vertex_target_0_2_tests
    vertex_target_0_5_tests
    vertex_target_0_8_tests
    vertex_target_0_9_tests
    vertex_target_0_95_tests)
add_regression_test("${REGRESSION_TESTS}")

# Generate Outputs
add_libs("${HEADERS}" "${SOURCE}" "${LIBS}")

# Cpp checks
add_analysis("${SOURCE}" "${HEADERS}")
