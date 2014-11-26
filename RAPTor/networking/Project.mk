# Source
SOURCE  = $(MAIN)
MAIN    = protocol_stack.cc common.cc \
    sdl_wrapper.cc sdl_event_handler_factory.cc 

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
    ${BOOST_INCLUDE_PATH} \
    $(LIBARYS_PATH)/tbb$(TBB_VER)/include/ \
    $(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
    $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/
LOCAL_INCLUDES = . $(RAPTOR_HOME)/common/ $(RAPTOR_HOME)/raytracer/ $(RAPTOR_HOME)/raytracer/materials/ $(RAPTOR_HOME)/sdl_wrappers
vpath %.cc $(LOCAL_INCLUDES)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib $(BOOST_LIB_PATH) $(TBB_LIB_PATH) $(RAYTRACER_HOME) $(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib
SO_LIBS = raytracer SDL2 SDL2_ttf boost_system boost_serialization tbb
LIBRARY = $(SO_LIBS) fftw3f

# Defines
DEFINES = 
