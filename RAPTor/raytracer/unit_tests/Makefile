# Include project info makefile
include ../Project.mk

# Source
TEST_SOURCE = simd_tests.cc \
	teamcity_boost.cc teamcity_messages.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out simd_tests.out

$(eval $(call test_suite_template, simd_tests.out, simd.o))