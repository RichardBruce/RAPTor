# Include project makefile
include $(RAPTOR_TOOLS)/Common.mk
include Project.mk

# All
all:: raytracer libraytracer.a libraytracer.so

# Link the object files to build the raytracer
raytracer : $(CPP_OBJECTS) $(C_OBJECTS) $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2-2.0.so.0.2.1 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbb.so.2 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbbmalloc.so.2
	$(CPP) $(LD_OPT) -o $@ $(CPP_OBJECTS) $(C_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))


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

# Clean targets
clean ::
	$(RM) -r build
	$(RM) raytracer
	$(RM) libraytracer.a
	$(RM) libraytracer.so
	$(RM) libraytracer.so.1
	$(RM) libraytracer.so.1.0.1
	$(MAKE) -C unit_tests clean
	$(MAKE) -C regression_tests clean

spotless ::
	$(RM) -r build
	$(RM) raytracer
	$(RM) libraytracer.a
	$(RM) libraytracer.so
	$(RM) libraytracer.so.1
	$(RM) libraytracer.so.1.0.1
	$(RM) *.tga
	$(RM) -rf scenes/spd$(SPD_VER)/
	$(MAKE) -C unit_tests spotless
	$(MAKE) -C regression_tests spotless
