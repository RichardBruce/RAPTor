# Include project info makefile
include ../Project.mk

# Source
TEST_SOURCE = bih_builder_tests.cc simd_tests.cc \
	teamcity_boost.cc teamcity_messages.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out bih_builder_tests.out simd_tests.out

$(eval $(call test_suite_template, bih_builder_tests.out, bih_builder.o bih.o triangle.o common.o simd.o))
$(eval $(call test_suite_template, simd_tests.out, simd.o))