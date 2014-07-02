# Include project info makefile
include ../Project.mk

# Source
SOURCE  += convex_decomposition.cc \
    teamcity_boost.cc teamcity_messages.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out convex_decomposition_tests.out

#$(eval $(call test_suite_template, clipping_tests.out, physics_common.o))
$(eval $(call test_suite_template, convex_decomposition_tests.out, graph.o convex_decomposition.o convex_hull.o manifold_mesh.o mesh_decimator.o allocator.o raycast_mesh.o))
