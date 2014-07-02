CPP = g++
CC = gcc

# Source
ALL_SOURCE 	= $(TEST_SOURCE) $(UUT_SOURCE)
TEST_SOURCE	= main.cc dropping_tests.cc free_motion_tests.cc stacking_tests.cc
UUT_SOURCE 	= physics_common.cc physics_object.cc physics_engine.cc vertex_group.cc \
	gjk.cc simplex.cc simulation_environment.cc \
	sdl_wrapper.cc sdl_event_handler_factory.cc

# Includes
INCLUDE = . ../ ../colliders ../forces ../integrators ../../sdl_wrappers ../../networking ${BOOST_INCLUDE_PATH} \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/include/tbb/ \
	$(LIBARYS_PATH)/fftw-$(FFTW_VER)/include $(RAPTOR_HOME)/common $(RAYTRACER_HOME) \
	$(RAYTRACER_HOME)/materials $(RAYTRACER_HOME)/parsers \
	$(LIBARYS_PATH)/teamcity
vpath %.cc $(INCLUDE)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release $(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib $(RAYTRACER_HOME) ${BOOST_LIB_PATH}
LIBRARY = raytracer SDL2 SDL2_ttf tbb fftw3f boost_unit_test_framework pthread boost_thread boost_filesystem boost_system boost_log boost_serialization

# Defines
DEFINES = REFLECTIONS_ON REFRACTIONS_ON SIMD_PACKET_TRACING FRUSTRUM_CULLING BOOST_TEST_DYN_LINK BOOST_LOG_DYN_LINK BOOST_LOG_LEVEL=boost::log::trivial::trace

# Compile and linker options 
COMMON_OPT 	= -std=c++1y -fPIC -Wno-unused-local-typedefs -Wall -pipe
DEBUG_OPT 	= $(COMMON_OPT) -g3 -O0 --coverage
RELEASE_OPT = $(COMMON_OPT) -O3 -mfpmath=sse -msse4a -fexpensive-optimizations #-march=native
ifeq ($(BUILD), DEBUG)
	LD_OPT = --coverage
	CC_OPT = $(DEBUG_OPT)
	TEST_SOURCE += teamcity_boost.cc teamcity_messages.cc
else
	LD_OPT =
	CC_OPT = $(RELEASE_OPT)
endif

# Derived files
UUT_OBJECTS = $(UUT_SOURCE:%.cc=build/%.o)
ALL_OBJECTS = $(ALL_SOURCE:%.cc=build/%.o)
EXES = $(TEST_SOURCE:.cc=.out)

# All
all:: main.out

# Compile Source code to object files
build/%.o : %.cc
	$(CPP) $(CC_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@

# Link the object files to build the TEST_SOURCEs
main.out : $(ALL_OBJECTS)
	$(CPP) $(LD_OPT) $(ALL_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $@ 

# Build individual TEST_SOURCE suites
%.out : $(UUT_OBJECTS) %.cc
	$(CPP) $(CC_OPT) -DSTAND_ALONE $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $(patsubst %.out, %.cc, $@) -o $(patsubst %.out, build/%.o, $@)
	$(CPP) $(LD_OPT) $(UUT_OBJECTS) $(patsubst %.out, build/%.o, $@) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $@ 

# Clean targets
clean ::
	$(RM) build/*
	$(RM) $(EXES)

spotless ::
	$(RM) -rf test_coverage
	$(RM) test_data/*/*.act
	$(RM) build/*
	$(RM) $(EXES)
