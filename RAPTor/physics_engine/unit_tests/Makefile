CPP = g++
CC = gcc

# Source
SOURCE  = main.cc physics_options_tests.cc lcp_solver_tests.cc inertia_tensor_tests.cc gjk_tests.cc clipping_tests.cc \
	exact_collision_time_tests.cc vertex_group_tests.cc rigid_body_collider_tests.cc simplex_tests.cc physics_engine_tests.cc \
	collision_info_tests.cc tracking_info_tests.cc physics_object_tests.cc contact_graph_tests.cc simulation_environment_tests.cc \
	integrator_tests.cc force_tests.cc \
	simulation_environment.cc physics_engine.cc physics_common.cc lcp_solver.cc gjk.cc vertex_group.cc simplex.cc physics_object.cc \
	common.cc phong_shader.cc \
	sdl_wrapper.cc sdl_event_handler_factory.cc \
	teamcity_boost.cc teamcity_messages.cc

# Includes
INCLUDE = . ../ ../colliders/ ../integrators/ ../forces/ ../../sdl_wrappers/ \
	$(RAYTRACER_HOME) $(RAYTRACER_HOME)/materials $(RAYTRACER_HOME)/parsers \
	$(RAPTOR_HOME)/common \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/include/tbb/ \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(BOOST_INCLUDE_PATH) \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
	$(LIBARYS_PATH)/teamcity
vpath %.cc $(INCLUDE)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib $(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib $(RAYTRACER_HOME) $(BOOST_LIB_PATH)
LIBRARY = raytracer SDL2 SDL2_ttf fftw3f boost_unit_test_framework pthread boost_thread boost_filesystem boost_system boost_log

# Defines
DEFINES = BOOST_TEST_DYN_LINK BOOST_LOG_DYN_LINK BOOST_LOG_LEVEL=boost::log::trivial::trace

# Compile and linker options 
CC_OPT = -g3 -O0 --coverage -std=c++1y -fPIC -Wno-unused-local-typedefs -Wall -pipe
LD_OPT = --coverage

# Derived files
TEST_OBJECTS = $(SOURCE:%.cc=build/%.o)
EXES = $(SOURCE:.cc=.out)

# All
all:: main.out clipping_tests.out collision_info_tests.out contact_graph_tests.out exact_collision_time_tests.out force_tests.out gjk_tests.out \
	inertia_tensor_tests.out integrator_tests.out lcp_solver_tests.out physics_engine_tests.out physics_options_tests.out \
	rigid_body_collider_tests.out simplex_tests.out simulation_environment_tests.out tracking_info_tests.out vertex_group_tests.out physics_object_tests.out

# Compile source code to object files
build/%.o : %.cc
	$(CPP) $(CC_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@

# Link the object files to build the tests
main.out : $(TEST_OBJECTS)
	$(CPP) $(LD_OPT) $(TEST_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $@ 

# Build individual test suites
define test_suite_template
$(1) : $(patsubst %, build/%, $(2)) $(patsubst %.out, %.cc, $(1))
	$(CPP) $(CC_OPT) -DSTAND_ALONE $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $(patsubst %.out, %.cc, $(1)) -o $(patsubst %.out, build/%.o, $(1))
	$(CPP) $(LD_OPT) $(patsubst %, build/%, $(2)) $(patsubst %.out, build/%.o, $(1)) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $(1) 
endef

$(eval $(call test_suite_template, clipping_tests.out, physics_common.o))
$(eval $(call test_suite_template, collision_info_tests.out, simplex.o))
$(eval $(call test_suite_template, contact_graph_tests.out, simplex.o))
$(eval $(call test_suite_template, exact_collision_time_tests.out, physics_common.o common.o)) # misses some if's in cubic root finder
$(eval $(call test_suite_template, force_tests.out, ))
$(eval $(call test_suite_template, gjk_tests.out, gjk.o physics_common.o))
$(eval $(call test_suite_template, inertia_tensor_tests.out, ))
$(eval $(call test_suite_template, integrator_tests.out, ))
$(eval $(call test_suite_template, lcp_solver_tests.out, lcp_solver.o)) # misses no solution during main loop
$(eval $(call test_suite_template, physics_engine_tests.out, physics_engine.o vertex_group.o physics_object.o physics_common.o gjk.o simplex.o))
$(eval $(call test_suite_template, physics_object_tests.out, physics_object.o physics_common.o gjk.o simplex.o)) # rotating collision not well covered
$(eval $(call test_suite_template, physics_options_tests.out, ))
$(eval $(call test_suite_template, rigid_body_collider_tests.out, physics_common.o))
$(eval $(call test_suite_template, simplex_tests.out, simplex.o))
$(eval $(call test_suite_template, simulation_environment_tests.out, simulation_environment.o physics_engine.o vertex_group.o physics_object.o physics_common.o gjk.o simplex.o common.o phong_shader.o sdl_wrapper.o sdl_event_handler_factory.o)) # rendering not covered
$(eval $(call test_suite_template, tracking_info_tests.out, simplex.o))
$(eval $(call test_suite_template, vertex_group_tests.out, physics_common.o))
# find support vertex not covered in physics common

# Clean targets
clean ::
	$(RM) build/*
	$(RM) $(EXES)

spotless ::
	$(RM) -rf test_coverage
	$(RM) build/*
	$(RM) $(EXES)
