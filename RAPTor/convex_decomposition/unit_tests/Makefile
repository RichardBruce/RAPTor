# Include project info makefile
include ../Project.mk

# Source
TEST_SOURCE = convex_decomposition_tests.cc incremental_convex_hull_tests.cc tetrahedron_set_tests.cc voxel_set_tests.cc \
	convex_mesh_tests.cc tm_mesh_tests.cc dac_convex_hull_tests.cc primitive_set_tests.cc volume_tests.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out convex_decomposition_tests.out incremental_convex_hull_tests.out tetrahedron_set_tests.out voxel_set_tests.out \
	convex_mesh_tests.out tm_mesh_tests.out dac_convex_hull_tests.out primitive_set_tests.out volume_tests.out

$(eval $(call test_suite_template, convex_decomposition_tests.out, convex_decomposition.o dac_convex_hull.o tetrahedron_set.o voxel_set.o volume.o))
$(eval $(call test_suite_template, convex_mesh_tests.out, dac_convex_hull.o))
$(eval $(call test_suite_template, dac_convex_hull_tests.out, dac_convex_hull.o))
$(eval $(call test_suite_template, incremental_convex_hull_tests.out, ))
$(eval $(call test_suite_template, primitive_set_tests.out, dac_convex_hull.o voxel_set.o))
$(eval $(call test_suite_template, tetrahedron_set_tests.out, tetrahedron_set.o dac_convex_hull.o))
$(eval $(call test_suite_template, tm_mesh_tests.out, ))
$(eval $(call test_suite_template, volume_tests.out, volume.o tetrahedron_set.o voxel_set.o dac_convex_hull.o))
$(eval $(call test_suite_template, voxel_set_tests.out, voxel_set.o dac_convex_hull.o))
