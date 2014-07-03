# Include project makefile
include $(RAPTOR_TOOLS)/Common.mk
include Project.mk

# All
all:: terrain_generator libterrain_generator.a libterrain_generator.so

# Link the object files to build the raytracer
terrain_generator : $(OBJECTS) $(RAYTRACER_HOME)/libraytracer.so $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2-2.0.so.0.2.1 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbb.so.2 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbbmalloc.so.2
	$(CPP) $(LD_OPT) -o $@ $(OBJECTS) $(C_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

libterrain_generator.so : $(OBJECTS)
	$(CPP) -fvisibility=default -shared -Wl,-soname,$@.1.0.1 -o $@.1.0.1 $(OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(SO_LIBS))
	ln -s $@.1.0.1 $@
	ln -s $@.1.0.1 $@.1

libterrain_generator.a : $(OBJECTS)
	ar rcs $@ $(OBJECTS)

# Clean targets
clean ::
	$(RM) -r build
	$(RM) terrain_generator
	$(RM) libterrain_generator.a
	$(RM) libterrain_generator.so
	$(RM) libterrain_generator.so.1
	$(RM) libterrain_generator.so.1.0.1

spotless ::
	$(RM) -r build
	$(RM) terrain_generator
	$(RM) libterrain_generator.a
	$(RM) libterrain_generator.so
	$(RM) libterrain_generator.so.1
	$(RM) libterrain_generator.so.1.0.1
	$(RM) cppcheck.xml
	$(RM) lines_of_code
