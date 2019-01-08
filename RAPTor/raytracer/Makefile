# Include project makefile
include $(RAPTOR_TOOLS)/Common.mk
include Project.mk

# All
all:: raytracer libraytracer.a libraytracer.so

# Link the object files to build the raytracer
raytracer : $(CPP_OBJECTS) $(C_OBJECTS) $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2.so $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbb.so.2 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbbmalloc.so.2
	$(CPP) $(LD_OPT) -o $@ $(CPP_OBJECTS) $(C_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

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
	$(RM) todo
	$(RM) cppcheck.xml
	$(RM) lines_of_code
	$(RM) *.tga
	$(MAKE) -C unit_tests spotless
	$(MAKE) -C regression_tests spotless
