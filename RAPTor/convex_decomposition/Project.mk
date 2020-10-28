# Source
SOURCE  = main.cc convex_decomposition.cc dac_convex_hull.cc voxel_set.cc tetrahedron_set.cc volume.cc

# Includes
INCLUDE = $(LOCAL_INCLUDES) \
    ${BOOST_INCLUDE_PATH} \
    $(LIBARYS_PATH)/tbb$(TBB_VER)/include/tbb/ 
LOCAL_INCLUDES  = . $(RAPTOR_HOME)/common $(RAPTOR_HOME)/parsers
vpath %.cc $(LOCAL_INCLUDES)

# Libraries
LIBPATH = $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release \
    ${BOOST_LIB_PATH}
SO_LIBS = tbb pthread boost_system boost_filesystem boost_log boost_serialization boost_program_options
LIBRARY = $(SO_LIBS)

# Defines
DEFINES = BOOST_LOG_DYN_LINK BOOST_LOG_LEVEL=boost::log::trivial::trace
