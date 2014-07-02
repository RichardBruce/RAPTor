CPP = g++
CC = gcc

# Source
SOURCE 	= $(MAIN) $(TEXTURE) $(SSD) $(PARSER) #$(SHAPE)
MAIN    = main.cc raytracer.cc common.cc ray.cc triangle.cc scene.cc sort.cc line.cc simd.cc packet_ray.cc camera.cc \
	sdl_wrapper.cc sdl_event_handler_factory.cc
#SHAPE   = shape.cc line.cc sphere.cc cylinder.cc cone.cc triangle.cc torus.cc ring.cc
SSD     = kdt_node.cc kd_tree_builder.cc voxel.cc bih_builder.cc
TEXTURE = phong_shader.cc cook_torrance_cxy.cc mandelbrot_shader.cc \
	mapper_shader.cc perlin_noise_2d_mapper.cc perlin_noise_3d_mapper.cc planar_mapper.cc \
	cylindrical_mapper.cc coloured_mapper_shader.cc perlin_noise_3d.cc perlin_noise_2d.cc
PARSER  = cfg_parser.cc nff_parser.cc mgf_parser.cc lwo_parser.cc obj_parser.cc ply_parser.cc vrml_parser.cc
MGF     = parser.c badarg.c lookup.c context.c vect.c words.c fvect.c object.c xf.c

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/include/tbb/ \
	$(LIBARYS_PATH)/fftw-$(FFTW_VER)/include \
	${BOOST_INCLUDE_PATH}
LOCAL_INCLUDES = . materials/ textures/ spatial_sub_division/ parsers/ parsers/mgflib ../sdl_wrappers \
	$(RAPTOR_HOME)/common/
vpath %.c $(INCLUDE)
vpath %.cc $(INCLUDE)

# Libraries
LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release $(LIBARYS_PATH)/fftw-$(FFTW_VER)/lib
SO_LIBS = SDL2 SDL2_ttf tbb tbbmalloc jpeg png
LIBRARY = $(SO_LIBS) fftw3f

# Defines
DEFINES = lint REFLECTIONS_ON REFRACTIONS_ON SIMD_PACKET_TRACING FRUSTRUM_CULLING SPATIAL_SUBDIVISION_BIH # THREADED_RAY_TRACE SPATIAL_SUBDIVISION_STATISTICS LOG_DEPTH SIMD_PACKET_TRACING FRUSTRUM_CULLING SHOW_KD_TREE DIFFUSE_REFLECTIONS=128.0 SOFT_SHADOW=256.0 

# Compile and linker options 
COMMON_OPT = -O3 -g -fPIC -Wno-unused-local-typedefs -Wall -pipe
DEBUG_OPT 	= $(COMMON_OPT) -g3 -O0 --coverage
RELEASE_OPT = $(COMMON_OPT) -O3 -mfpmath=sse -msse4a -fexpensive-optimizations #-march=native
ifeq ($(BUILD), DEBUG)
	LD_OPT = --coverage
	CC_OPT = $(DEBUG_OPT) --std=c11
	CPP_OPT = -std=c++1y $(DEBUG_OPT)
else
	LD_OPT =
	CC_OPT = $(RELEASE_OPT) --std=c11
	CPP_OPT = -std=c++1y $(RELEASE_OPT)
endif

# Derived files
OBJECTS = $(SOURCE:%.cc=build/%.o)
C_OBJECTS = $(MGF:%.c=build/%.o)

# All
all:: raytracer libraytracer.a libraytracer.so

# Compile source code to object files
build/%.o : %.cc
	$(CPP) $(CPP_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@
        
build/%.o : %.c
	$(CC) $(CC_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@


# Link the object files to build the raytracer
raytracer : $(OBJECTS) $(C_OBJECTS) $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2-2.0.so.0.2.1 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbb.so.2 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbbmalloc.so.2
	$(CPP) $(LD_OPT) -o $@ $(OBJECTS) $(C_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

libraytracer.so : $(OBJECTS) $(C_OBJECTS)
	$(CPP) -fvisibility=default -shared -Wl,-soname,$@.1.0.1 -o $@.1.0.1 $(OBJECTS) $(C_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(SO_LIBS))
	ln -s $@.1.0.1 $@
	ln -s $@.1.0.1 $@.1

libraytracer.a : $(OBJECTS) $(C_OBJECTS)
	ar rcs $@ $(OBJECTS) $(C_OBJECTS)


# SPD for test scenes
$(LIBARYS_PATH)/spd$(SPD_VER) : 
	@(cd $(LIBARYS_PATH) ; make -s spd$(SPD_VER) )

SPD : $(LIBARYS_PATH)/spd$(SPD_VER)
	@mkdir scenes/spd$(SPD_VER)
	@mkdir scenes/spd$(SPD_VER)/balls_scenes
	@mkdir scenes/spd$(SPD_VER)/gears_scenes
	@mkdir scenes/spd$(SPD_VER)/jacks_scenes
	@mkdir scenes/spd$(SPD_VER)/lattice_scenes
	@mkdir scenes/spd$(SPD_VER)/mount_scenes
	@mkdir scenes/spd$(SPD_VER)/nurbtst_scenes
	@mkdir scenes/spd$(SPD_VER)/rings_scenes
	@mkdir scenes/spd$(SPD_VER)/sample_scenes
	@mkdir scenes/spd$(SPD_VER)/shells_scenes
	@mkdir scenes/spd$(SPD_VER)/sombrero_scenes
	@mkdir scenes/spd$(SPD_VER)/teapot_scenes
	@mkdir scenes/spd$(SPD_VER)/tetra_scenes
	@mkdir scenes/spd$(SPD_VER)/tree_scenes
	@echo "Generating spd$(SPD_VER) scenes....."
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/balls    balls_scenes    ; cd balls_scenes    ; ./balls    -r 1 -s  1 -t  9 > balls_1.nff    ; ./balls    -r 1 -s  2 -t  9 > balls_2.nff    ; ./balls    -r 1 -s  3 -t  9 > balls_3.nff    ; ./balls    -r 1 -s   4 -t  9 > balls_4.nff    )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/gears    gears_scenes    ; cd gears_scenes    ; ./gears    -r 1 -s  2       > gears_2.nff    ; ./gears    -r 1 -s  5       > gears_5.nff    ; ./gears    -r 1 -s  12      > gears_12.nff   ; ./gears    -r 1 -s  25       > gears_25.nff   )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/jacks    jacks_scenes    ; cd jacks_scenes    ; ./jacks    -r 1 -s  2 -t  9 > jacks_2.nff    ; ./jacks    -r 1 -s  3 -t  9 > jacks_3.nff    ; ./jacks    -r 1 -s  4 -t  9 > jacks_4.nff    ; ./jacks    -r 1 -s   5 -t  9 > jacks_5.nff    )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/lattice  lattice_scenes  ; cd lattice_scenes  ; ./lattice  -r 1 -s  1 -t  9 > lattice_1.nff  ; ./lattice  -r 1 -s  3 -t  9 > lattice_3.nff  ; ./lattice  -r 1 -s  8 -t  9 > lattice_8.nff  ; ./lattice  -r 1 -s  19 -t  9 > lattice_19.nff )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/mount    mount_scenes    ; cd mount_scenes    ; ./mount    -r 1 -s  4 -t 14 > mount_4.nff    ; ./mount    -r 1 -s  7 -t 14 > mount_7.nff    ; ./mount    -r 1 -s  9 -t 14 > mount_9.nff    ; ./mount    -r 1 -s   9 -t 11 > mount_11.nff   )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/nurbtst  nurbtst_scenes  ; cd nurbtst_scenes  ; ./nurbtst  -r 1 -s  1       > nurbtst_1.nff  )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/rings    rings_scenes    ; cd rings_scenes    ; ./rings    -r 1 -s  1 -t  4 > rings_1.nff    ; ./rings    -r 1 -s  2 -t  7 > rings_2.nff    ; ./rings    -r 1 -s  4 -t  9 > rings_4.nff    ; ./rings    -r 1 -s   9 -t  9 > rings_9.nff    )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/sample   sample_scenes   ; cd sample_scenes   ; ./sample   -r 1 -s  1 -t 25 > sample_1.nff   )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/shells   shells_scenes   ; cd shells_scenes   ; ./shells   -r 1 -s  1 -t    > shells_1.nff   ; ./shells   -r 1 -s  2 -t    > shells_2.nff   ; ./shells   -r 1 -s   5 -t   > shells_5.nff   ; ./shells   -r 1 -s   9 -t    > shells_9.nff   )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/sombrero sombrero_scenes ; cd sombrero_scenes ; ./sombrero -r 1 -s  2       > sombrero_2.nff ; ./sombrero -r 1 -s  3       > sombrero_3.nff ; ./sombrero -r 1 -s   5      > sombrero_5.nff ; ./sombrero -r 1 -s   7       > sombrero_7.nff )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/teapot   teapot_scenes   ; cd teapot_scenes   ; ./teapot   -r 1 -s 12       > teapot_12.nff  ; ./teapot   -r 1 -s 38       > teapot_38.nff  ; ./teapot   -r 1 -s 123      > teapot_123.nff ; ./teapot   -r 1 -s 389       > teapot_389.nff )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/tetra    tetra_scenes    ; cd tetra_scenes    ; ./tetra    -r 1 -s  6       > tetra_6.nff    ; ./tetra    -r 1 -s  8       > tetra_8.nff    ; ./tetra    -r 1 -s   9      > tetra_9.nff    ; ./tetra    -r 1 -s  11       > tetra_11.nff   )
	@( cd scenes/spd$(SPD_VER) ; cp $(LIBARYS_PATH)/spd$(SPD_VER)/tree     tree_scenes     ; cd tree_scenes     ; ./tree     -r 1 -s  4 -t 4  > tree_4.nff     ; ./tree     -r 1 -s  5 -t 9  > tree_7.nff     ; ./tree     -r 1 -s  10 -t 6 > tree_10.nff    ; ./tree     -r 1 -s  14 -t  4 > tree_14.nff    )


# Back up the ray tracer
raytracer_$(RAY_VER).tar.gz :
	tar -cvf raytracer_$(RAY_VER).tar ./ --exclude "scenes"
	gzip raytracer_$(RAY_VER).tar

# Lint code
cppcheck.xml ::
	cppcheck  $(patsubst %,-I%,$(LOCAL_INCLUDES)) -DDIFFUSE_REFLECTIONS=64 -DSOFT_SHADOW=64 --enable=all --error-exitcode=1 --force --inline-suppr --suppress=missingIncludeSystem --xml-version=2 ./ 2> cppcheck.xml


# Clean targets
clean ::
	$(RM) build/*
	$(RM) raytracer
	$(RM) libraytracer.a
	$(RM) libraytracer.so
	$(RM) libraytracer.so.1
	$(RM) libraytracer.so.1.0.1

spotless ::
	$(RM) build/*
	$(RM) raytracer
	$(RM) libraytracer.a
	$(RM) libraytracer.so
	$(RM) libraytracer.so.1
	$(RM) libraytracer.so.1.0.1
	$(RM) *.tga
	$(RM) -rf scenes/spd$(SPD_VER)/
