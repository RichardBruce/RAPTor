# Include project makefile
include $(RAPTOR_TOOLS)/Common.mk
include Project.mk

# All
all:: physics_engine libphysics_engine.a libphysics_engine.so

# Link the object files to build the raytracer
physics_engine : $(CPP_OBJECTS) $(RAYTRACER_HOME)/libraytracer.so $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2-2.0.so.0.2.1 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbb.so.2 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbbmalloc.so.2
	$(CPP) $(LD_OPT) -o $@ $(CPP_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

# Clean targets
clean ::
	$(RM) -r build
	$(RM) physics_engine
	$(RM) libphysics_engine.a
	$(RM) libphysics_engine.so
	$(RM) libphysics_engine.so.1
	$(RM) libphysics_engine.so.1.0.1

spotless ::
	$(RM) -r build
	$(RM) physics_engine
	$(RM) libphysics_engine.a
	$(RM) libphysics_engine.so
	$(RM) libphysics_engine.so.1
	$(RM) libphysics_engine.so.1.0.1
	$(RM) cppcheck.xml
	$(RM) lines_of_code
