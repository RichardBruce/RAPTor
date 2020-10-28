# Include project makefile
include $(RAPTOR_TOOLS)/Common.mk
include Project.mk

# All
all:: convex_decomposition libconvex_decomposition.a libconvex_decomposition.so

# Link the object files to build the raytracer
convex_decomposition : $(CPP_OBJECTS) $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbb.so.2 $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release/libtbbmalloc.so.2
	$(CPP) $(LD_OPT) -o $@ $(CPP_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY))

# Clean targets
clean ::
	$(RM) -r build
	$(RM) convex_decomposition
	$(RM) libconvex_decomposition.a
	$(RM) libconvex_decomposition.so
	$(RM) libconvex_decomposition.so.1
	$(RM) libconvex_decomposition.so.1.0.1
	$(MAKE) -C unit_tests clean
	$(MAKE) -C regression_tests clean

spotless ::
	$(RM) -r build
	$(RM) convex_decomposition
	$(RM) libconvex_decomposition.a
	$(RM) libconvex_decomposition.so
	$(RM) libconvex_decomposition.so.1
	$(RM) libconvex_decomposition.so.1.0.1
	$(RM) todo
	$(RM) cppcheck.xml
	$(RM) lines_of_code
	$(MAKE) -C unit_tests spotless
	$(MAKE) -C regression_tests spotless
