# Include project info makefile
include ../Project.mk

# Source
TEST_SOURCE = grid_cell_tests.cc \
	teamcity_boost.cc teamcity_messages.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out grid_cell_tests.out

#$(eval $(call test_suite_template, clipping_tests.out, physics_common.o))
$(eval $(call test_suite_template, grid_cell_tests.out, ))
