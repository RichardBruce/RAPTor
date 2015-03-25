# Source
SOURCE  = main.cc \
    perlin_noise_2d.cc perlin_noise_3d.cc simplex_noise_2d.cc simplex_noise_3d.cc

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
    $(RAPTOR_HOME)/raytracer/ \
    $(RAPTOR_HOME)/raytracer/materials/ \
    $(RAPTOR_HOME)/raytracer/parsers/ \
    $(RAPTOR_HOME)/physics_engine \
    $(RAPTOR_HOME)/physics_engine/forces/ \
    $(RAPTOR_HOME)/physics_engine/colliders/ \
    $(RAPTOR_HOME)/physics_engine/integrators/ \
    $(RAPTOR_HOME)/sdl_wrappers/ \
    ${BOOST_INCLUDE_PATH} \
    $(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
    $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
    $(LIBARYS_PATH)/SDL2_image-2.0.0/include/SDL2/ \
    $(LIBARYS_PATH)/tbb$(TBB_VER)/include/ \
    $(LIBARYS_PATH)/libtga-$(LIBTGA_VER)/include \
    $(LIBARYS_PATH)/fftw-$(FFTW_VER)/include 
LOCAL_INCLUDES  = . $(RAPTOR_HOME)/common/
vpath %.cc $(INCLUDE)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib \
    $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib \
    $(LIBARYS_PATH)/SDL2_image-$(SDLIMAGE_VER)/lib \
    $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release \
    $(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib \
    $(RAYTRACER_HOME) \
    $(RAPTOR_HOME)/physics_engine \
    ${BOOST_LIB_PATH}
SO_LIBS = raytracer physics_engine SDL2 SDL2_image SDL2_ttf tbb pthread boost_thread boost_filesystem boost_system boost_log boost_serialization jpeg
LIBRARY = $(SO_LIBS) fftw3f

# Defines
DEFINES = REFLECTIONS_ON REFRACTIONS_ON SIMD_PACKET_TRACING FRUSTRUM_CULLING BOOST_LOG_DYN_LINK BOOST_LOG_LEVEL=boost::log::trivial::trace
