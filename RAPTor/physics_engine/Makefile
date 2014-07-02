CPP = g++
CC = gcc

# Source
SOURCE 	= main.cc simulation_environment.cc physics_common.cc physics_object.cc physics_engine.cc \
	vertex_group.cc gjk.cc simplex.cc \
	sdl_wrapper.cc sdl_event_handler_factory.cc

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
	${BOOST_INCLUDE_PATH} \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
	$(LIBARYS_PATH)/SDL2_image-$(SDLIMAGE_VER)/include/SDL2/ \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/include/tbb/ \
	$(LIBARYS_PATH)/fftw-$(FFTW_VER)/include 
LOCAL_INCLUDES	= . ./colliders ./integrators ./forces ../sdl_wrappers ../networking \
	$(RAPTOR_HOME)/common \
	$(RAYTRACER_HOME) $(RAYTRACER_HOME)/materials $(RAYTRACER_HOME)/parsers
vpath %.cc $(INCLUDE)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release $(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib $(RAYTRACER_HOME) ${BOOST_LIB_PATH}
SO_LIBS = raytracer SDL2 SDL2_ttf tbb pthread boost_thread boost_filesystem boost_system boost_log boost_serialization
LIBRARY = $(SO_LIBS) fftw3f

# Defines
DEFINES = REFLECTIONS_ON REFRACTIONS_ON SIMD_PACKET_TRACING FRUSTRUM_CULLING BOOST_LOG_DYN_LINK BOOST_LOG_LEVEL=boost::log::trivial::trace

# Compile and linker options 
COMMON_OPT 	= -std=c++1y -fPIC -Wno-unused-local-typedefs -Wall -pipe
DEBUG_OPT 	= $(COMMON_OPT) -g3 -O0 --coverage
RELEASE_OPT = $(COMMON_OPT) -O3 -mfpmath=sse -msse4a -fexpensive-optimizations #-march=native
ifeq ($(BUILD), DEBUG)
	LD_OPT = --coverage
	CC_OPT = $(DEBUG_OPT)
else
	LD_OPT =
	CC_OPT = $(RELEASE_OPT)
endif

# Derived files
OBJECTS = $(SOURCE:%.cc=build/%.o)

# All
all:: physics_engine libphysics_engine.a libphysics_engine.so

# Compile source code to object files
build/%.o : %.cc
	$(CPP) $(CC_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@

# Link the object files to build the raytracer
physics_engine : $(OBJECTS) $(RAYTRACER_HOME)/libraytracer.so $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2-2.0.so.0.2.1 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbb.so.2 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbbmalloc.so.2
	$(CPP) $(LD_OPT) -o $@ $(OBJECTS) $(C_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

libphysics_engine.so : $(OBJECTS)
	$(CPP) -fvisibility=default -shared -Wl,-soname,$@.1.0.1 -o $@.1.0.1 $(OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(SO_LIBS))
	ln -s $@.1.0.1 $@
	ln -s $@.1.0.1 $@.1

libphysics_engine.a : $(OBJECTS)
	ar rcs $@ $(OBJECTS)

# Lint code
cppcheck.xml ::
	cppcheck  $(patsubst %,-I%,$(LOCAL_INCLUDES)) --enable=all --error-exitcode=1 --force --inline-suppr --suppress=missingIncludeSystem --xml-version=2 ./ 2> cppcheck.xml

# Clean targets
clean ::
	$(RM) build/*
	$(RM) physics_engine
	$(RM) libphysics_engine.a
	$(RM) libphysics_engine.so
	$(RM) libphysics_engine.so.1
	$(RM) libphysics_engine.so.1.0.1

spotless ::
	$(RM) build/*
	$(RM) physics_engine
	$(RM) libphysics_engine.a
	$(RM) libphysics_engine.so
	$(RM) libphysics_engine.so.1
	$(RM) libphysics_engine.so.1.0.1
	$(RM) cppcheck.xml
	$(RM) lines_of_code
