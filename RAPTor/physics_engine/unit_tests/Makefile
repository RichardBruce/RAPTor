# Include project info makefile
include ../Project.mk

# Source
TEST_SOURCE = physics_options_tests.cc lcp_solver_tests.cc inertia_tensor_tests.cc gjk_tests.cc clipping_tests.cc \
	exact_collision_time_tests.cc vertex_group_tests.cc rigid_body_collider_tests.cc simplex_tests.cc physics_engine_tests.cc \
	collision_info_tests.cc tracking_info_tests.cc physics_object_tests.cc contact_graph_tests.cc simulation_environment_tests.cc \
	integrator_tests.cc force_tests.cc object_bound_tests.cc spatial_sub_division_tests.cc \
	teamcity_boost.cc teamcity_messages.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out clipping_tests.out collision_info_tests.out contact_graph_tests.out exact_collision_time_tests.out force_tests.out gjk_tests.out \
	inertia_tensor_tests.out integrator_tests.out lcp_solver_tests.out object_bound_tests.out physics_engine_tests.out physics_options_tests.out \
	rigid_body_collider_tests.out simplex_tests.out simulation_environment_tests.out spatial_sub_division_tests.out tracking_info_tests.out \
	vertex_group_tests.out pair_manager_tests.out physics_object_tests.out

$(eval $(call test_suite_template, clipping_tests.out, physics_common.o))
$(eval $(call test_suite_template, collision_info_tests.out, simplex.o))
$(eval $(call test_suite_template, contact_graph_tests.out, simplex.o))
$(eval $(call test_suite_template, exact_collision_time_tests.out, physics_common.o common.o)) # misses some if's in cubic root finder
$(eval $(call test_suite_template, force_tests.out, ))
$(eval $(call test_suite_template, gjk_tests.out, gjk.o physics_common.o))
$(eval $(call test_suite_template, inertia_tensor_tests.out, ))
$(eval $(call test_suite_template, integrator_tests.out, ))
$(eval $(call test_suite_template, lcp_solver_tests.out, lcp_solver.o)) # misses no solution during main loop
$(eval $(call test_suite_template, object_bound_tests.out, ))
$(eval $(call test_suite_template, pair_manager_tests.out, ))
$(eval $(call test_suite_template, physics_engine_tests.out, physics_engine.o vertex_group.o physics_object.o physics_common.o gjk.o simplex.o))
$(eval $(call test_suite_template, physics_object_tests.out, physics_object.o physics_common.o gjk.o simplex.o)) # rotating collision not well covered
$(eval $(call test_suite_template, physics_options_tests.out, ))
$(eval $(call test_suite_template, rigid_body_collider_tests.out, physics_common.o))
$(eval $(call test_suite_template, simplex_tests.out, simplex.o))
$(eval $(call test_suite_template, simulation_environment_tests.out, simulation_environment.o physics_engine.o vertex_group.o physics_object.o physics_common.o gjk.o simplex.o common.o phong_shader.o sdl_wrapper.o sdl_event_handler_factory.o)) # rendering not covered
$(eval $(call test_suite_template, spatial_sub_division_tests.out, spatial_sub_division.o))
$(eval $(call test_suite_template, tracking_info_tests.out, simplex.o))
$(eval $(call test_suite_template, vertex_group_tests.out, physics_common.o))
# find support vertex not covered in physics common
