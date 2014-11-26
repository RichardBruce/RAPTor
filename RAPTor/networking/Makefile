# Include project makefile
SOURCE += video_receiver.cc 

include $(RAPTOR_TOOLS)/Common.mk
include Project.mk

# All
all:: video_receiver

# Link the object files to build the raytracer
video_receiver : $(CPP_OBJECTS) $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib/libSDL2-2.0.so.0.2.1
	$(CPP) $(LD_OPT) -o $@ $(CPP_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

# Clean targets
clean ::
	$(RM) -r build
	$(RM) video_receiver
	$(MAKE) -C unit_tests clean
	$(MAKE) -C regression_tests clean

spotless ::
	$(RM) -r build
	$(RM) video_receiver
	$(RM) todo
	$(RM) cppcheck.xml
	$(RM) lines_of_code
	$(MAKE) -C unit_tests spotless
	$(MAKE) -C regression_tests spotless
