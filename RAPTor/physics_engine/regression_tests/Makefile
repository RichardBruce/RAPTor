# Include project info makefile
include ../Project.mk

# Uut objects
UUT_OBJECTS = $(filter-out %main.o, $(C_OBJECTS:build/%.o=%.o) $(CPP_OBJECTS:build/%.o=%.o))

# Source
TEST_SOURCE = dropping_tests.cc free_motion_tests.cc stacking_tests.cc \
	teamcity_boost.cc teamcity_messages.cc

LIBRARY += boost_filesystem

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out dropping_tests.out free_motion_tests.out stacking_tests.out

$(eval $(call test_suite_template, dropping_tests.out, dropping_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, free_motion_tests.out, free_motion_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, stacking_tests.out, stacking_tests.o $(UUT_OBJECTS)))
