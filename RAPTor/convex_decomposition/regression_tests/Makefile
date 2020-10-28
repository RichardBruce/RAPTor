# Include project info makefile
include ../Project.mk

# Uut objects
UUT_OBJECTS = $(filter-out %main.o, $(C_OBJECTS:build/%.o=%.o) $(CPP_OBJECTS:build/%.o=%.o))

# Source
TEST_SOURCE = tetrahedron_no_pca_no_hull_approx_tests.cc tetrahedron_no_pca_hull_approx_tests.cc tetrahedron_pca_hull_approx_tests.cc \
	voxel_no_pca_no_hull_approx_tests.cc voxel_no_pca_hull_approx_simplify_tests.cc voxel_pca_hull_approx_tests.cc voxel_no_pca_hull_approx_tests.cc \
	teamcity_boost.cc teamcity_messages.cc
LIBRARY += boost_filesystem

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out tetrahedron_no_pca_no_hull_approx_tests.out tetrahedron_no_pca_hull_approx_tests.out tetrahedron_pca_hull_approx_tests.out \
	voxel_no_pca_no_hull_approx_tests.out voxel_no_pca_hull_approx_simplify_tests.out voxel_pca_hull_approx_tests.out voxel_no_pca_hull_approx_tests.out

$(eval $(call test_suite_template, tetrahedron_no_pca_no_hull_approx_tests.out, tetrahedron_no_pca_no_hull_approx_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, tetrahedron_no_pca_hull_approx_tests.out, tetrahedron_no_pca_hull_approx_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, tetrahedron_pca_hull_approx_tests.out, tetrahedron_pca_hull_approx_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, voxel_no_pca_no_hull_approx_tests.out, voxel_no_pca_no_hull_approx_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, voxel_no_pca_hull_approx_simplify_tests.out, voxel_no_pca_hull_approx_simplify_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, voxel_pca_hull_approx_tests.out, voxel_pca_hull_approx_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, voxel_no_pca_hull_approx_tests.out, voxel_no_pca_hull_approx_tests.o $(UUT_OBJECTS)))

