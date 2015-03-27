# Source
SOURCE  = main.cc lcp_solver.cc simulation_environment.cc physics_common.cc physics_object.cc physics_engine.cc \
    vertex_group.cc gjk.cc simplex.cc \
    sdl_wrapper.cc sdl_event_handler_factory.cc

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
    ${BOOST_INCLUDE_PATH} \
    $(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
    $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
    $(LIBARYS_PATH)/SDL2_image-$(SDLIMAGE_VER)/include/SDL2/ \
    $(LIBARYS_PATH)/tbb$(TBB_VER)/include/tbb/ \
    $(LIBARYS_PATH)/libtga-$(LIBTGA_VER)/include \
    $(LIBARYS_PATH)/fftw-$(FFTW_VER)/include 
LOCAL_INCLUDES  = . $(RAPTOR_HOME)/physics_engine/colliders $(RAPTOR_HOME)/physics_engine/integrators $(RAPTOR_HOME)/physics_engine/forces $(RAPTOR_HOME)/sdl_wrappers $(RAPTOR_HOME)/networking \
    $(RAPTOR_HOME)/common \
    $(RAYTRACER_HOME) $(RAYTRACER_HOME)/materials $(RAYTRACER_HOME)/parsers $(RAYTRACER_HOME)/spatial_sub_division
vpath %.cc $(LOCAL_INCLUDES)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib \
    $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib \
    $(LIBARYS_PATH)/SDL2_image-$(SDLIMAGE_VER)/lib \
    $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release \
    $(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib \
    $(RAYTRACER_HOME) \
    ${BOOST_LIB_PATH}
SO_LIBS = raytracer SDL2 SDL2_ttf SDL2_image tbb pthread boost_thread boost_filesystem boost_system boost_log boost_serialization
LIBRARY = $(SO_LIBS) fftw3f

# Defines
DEFINES = REFLECTIONS_ON REFRACTIONS_ON SIMD_PACKET_TRACING FRUSTRUM_CULLING BOOST_LOG_DYN_LINK BOOST_LOG_LEVEL=boost::log::trivial::error
