# Source
SOURCE  = $(MAIN) $(TEXTURE) $(SSD) $(PARSER) $(MGF) #$(SHAPE)
MAIN    = main.cc raytracer.cc common.cc ray.cc triangle.cc scene.cc sort.cc line.cc simd.cc packet_ray.cc camera.cc \
	sdl_wrapper.cc sdl_event_handler_factory.cc
#SHAPE   = shape.cc line.cc sphere.cc cylinder.cc cone.cc triangle.cc torus.cc ring.cc
SSD	 = kd_tree.cc kdt_node.cc kdt_builder.cc voxel.cc bih.cc bih_builder.cc bvh.cc bvh_builder.cc
TEXTURE = picture_functions.cc phong_shader.cc cook_torrance_cxy.cc mandelbrot_shader.cc \
	mapper_shader.cc mapper_falloff.cc gradient_mapper.cc checker_board_mapper.cc perlin_noise_2d_mapper.cc perlin_noise_3d_mapper.cc planar_mapper.cc \
	cylindrical_mapper.cc cubic_mapper.cc coloured_mapper_shader.cc perlin_noise_3d.cc perlin_noise_2d.cc
PARSER  = cfg_parser.cc nff_parser.cc mgf_parser.cc lwo_parser.cc lwo1_parser.cc lwo2_parser.cc lwo_surf.cc lwo_bloks.cc normal_calculator.cc obj_parser.cc off_parser.cc ply_parser.cc vrml_parser.cc
MGF	 = parser.c badarg.c lookup.c context.c vect.c words.c fvect.c object.c xf.c

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/include/tbb/ \
	$(LIBARYS_PATH)/fftw-$(FFTW_VER)/include \
	$(LIBARYS_PATH)/libtga-$(LIBTGA_VER)/include \
	${BOOST_INCLUDE_PATH}
LOCAL_INCLUDES = . $(RAPTOR_HOME)/raytracer/materials/ $(RAPTOR_HOME)/raytracer/spatial_sub_division/ \
	$(RAPTOR_HOME)/raytracer/parsers/ $(RAPTOR_HOME)/raytracer/parsers/mgflib $(RAPTOR_HOME)/sdl_wrappers $(RAPTOR_HOME)/common/
vpath %.c $(LOCAL_INCLUDES)
vpath %.cc $(LOCAL_INCLUDES)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release \
	$(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib \
	$(LIBARYS_PATH)/libtga-$(LIBTGA_VER)/lib \
	${BOOST_LIB_PATH}
SO_LIBS = SDL2 SDL2_ttf tbb tbbmalloc jpeg png tga pthread boost_system boost_filesystem boost_log boost_serialization
LIBRARY = $(SO_LIBS) fftw3f

# Defines
DEFINES = SIMD_PACKET_TRACING FRUSTRUM_CULLING BOOST_LOG_DYN_LINK BOOST_LOG_LEVEL=boost::log::trivial::trace # THREADED_RAY_TRACE LOG_DEPTH SIMD_PACKET_TRACING FRUSTRUM_CULLING SHOW_KD_TREE DIFFUSE_REFLECTIONS=128.0 SOFT_SHADOW=256.0 
LINTER_DEFINES = DIFFUSE_REFLECTIONS=128.0 SOFT_SHADOW=256.0 
