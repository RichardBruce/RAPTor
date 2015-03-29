# Include project info makefile
include ../Project.mk

# Source
TEST_SOURCE = bih_tests.cc bih_block_tests.cc bih_builder_tests.cc bih_node_tests.cc normal_calculator_tests.cc simd_tests.cc sort_tests.cc \
	texture_mapper_tests.cc vfp_tests.cc vint_tests.cc voxel_tests.cc \
	teamcity_boost.cc teamcity_messages.cc

DEFINES += SIMD_PACKET_TRACING FRUSTRUM_CULLING

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out bih_tests.out bih_block_tests.out bih_builder_tests.out bih_node_tests.out normal_calculator_tests.out simd_tests.out sort_tests.out texture_mapper_tests.out vfp_tests.out vint_tests.out voxel_tests.out

$(eval $(call test_suite_template, bih_tests.out, bih.o bih_builder.o common.o triangle.o phong_shader.o ray.o raytracer.o packet_ray.o simd.o))
$(eval $(call test_suite_template, bih_block_tests.out, ))
$(eval $(call test_suite_template, bih_builder_tests.out, bih_builder.o triangle.o common.o simd.o))
$(eval $(call test_suite_template, bih_node_tests.out, common.o triangle.o phong_shader.o ray.o raytracer.o packet_ray.o simd.o))
$(eval $(call test_suite_template, normal_calculator_tests.out, normal_calculator.o common.o))
$(eval $(call test_suite_template, simd_tests.out, simd.o))
$(eval $(call test_suite_template, sort_tests.out, sort.o simd.o))
$(eval $(call test_suite_template, texture_mapper_tests.out, ))
$(eval $(call test_suite_template, vfp_tests.out, simd.o))
$(eval $(call test_suite_template, vint_tests.out, simd.o))
$(eval $(call test_suite_template, voxel_tests.out, voxel.o simd.o common.o))
