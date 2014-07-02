CPP = g++
CC = gcc

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

# Source
SOURCE  = $(MAIN)
MAIN    = video_receiver.cc protocol_stack.cc ../raytracer/common.cc \
	../sdl_wrappers/sdl_wrapper.cc ../sdl_wrappers/sdl_event_handler_factory.cc 

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
	${BOOST_INCLUDE_PATH} \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/include/ \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/
LOCAL_INCLUDES = . $(RAPTOR_HOME)/common/ ../raytracer/ ../raytracer/materials/ ../sdl_wrappers

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib $(BOOST_LIB_PATH) $(TBB_LIB_PATH)
LIBRARY = SDL2 SDL2_ttf boost_system boost_serialization tbb

# Defines
DEFINES = 

# Derived files
OBJECTS = $(SOURCE:.cc=.o)

# All
all:: video_receiver

# Compile source code to object files
.cc.o :
	$(CPP) $(CC_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@


# Link the object files to build the raytracer
video_receiver : $(OBJECTS) $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2-2.0.so.0.2.1
	$(CPP) $(LD_OPT) -o $@ $(OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

# Lint code
cppcheck.xml ::
	cppcheck  $(patsubst %,-I%,$(LOCAL_INCLUDES)) --enable=all --error-exitcode=1 --force --inline-suppr --suppress=missingIncludeSystem --xml-version=2 ./ 2> cppcheck.xml

# Clean targets
clean ::
	$(RM) $(OBJECTS)
	$(RM) video_receiver
