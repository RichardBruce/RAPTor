# Include project info makefile
include ../Project.mk

# Uut objects
UUT_OBJECTS = $(filter-out %main.o, $(C_OBJECTS:build/%.o=%.o) $(CPP_OBJECTS:build/%.o=%.o))

# Source
TEST_SOURCE = cfg_tests.cc lwo_tests.cc mgf_tests.cc nff_tests.cc obj_tests.cc ply_tests.cc \
	teamcity_boost.cc teamcity_messages.cc

LIBRARY += boost_filesystem

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out cfg_tests.out lwo_tests.out mgf_tests.out nff_tests.out obj_tests.out ply_tests.out

$(eval $(call test_suite_template, cfg_tests.out, cfg_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, lwo_tests.out, lwo_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, mgf_tests.out, mgf_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, nff_tests.out, nff_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, obj_tests.out, obj_tests.o $(UUT_OBJECTS)))
$(eval $(call test_suite_template, ply_tests.out, ply_tests.o $(UUT_OBJECTS)))
