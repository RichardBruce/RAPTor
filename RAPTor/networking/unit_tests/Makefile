CPP = g++
CC = gcc
OPT = -g3 -O0 --coverage -std=c++11 -fPIC -Wno-unused-local-typedefs -Wall

CWD     = $(shell pwd)

SOURCE  = main.cc vector_stream_tests.cc msg_header_tests.cc msg_data_tests.cc stack_controller_tests.cc \
	fragmenter_tests.cc batcher_tests.cc drop_late_sequencer_tests.cc reorder_sequencer_tests.cc \
	ack_retransmission_tests.cc nack_retransmission_tests.cc rate_limiter_tests.cc message_delivery_tests.cc \
	udp_connection_tests.cc tcp_connection_tests.cc group_tests.cc \
	$(LIBARYS_PATH)/teamcity/teamcity_boost.cc  $(LIBARYS_PATH)/teamcity/teamcity_messages.cc


INCLUDE = . ../ ../../raytracer/ ../raytracer/materials/ ../sdl_wrappers ${BOOST_INCLUDE_PATH} \
	$(LIBARYS_PATH)/tbb$(TBB_VER)/include/ \
	$(LIBARYS_PATH)/SDL2-$(SDL_VER)/include/ \
	$(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/include/SDL2/ \
	$(LIBARYS_PATH)/teamcity

DEFINES = BOOST_TEST_DYN_LINK __DEBUG_CHECKS__

OBJECTS = $(SOURCE:.cc=.o)
TEST_OBJECTS = $(SOURCE:.cc=.o)
EXES = $(SOURCE:.cc=.out)
TEST_OUTPUTS = $(SOURCE:.cc=.txt)

GCDA = $(SOURCE:.cc=.gcda)
GCNO = $(SOURCE:.cc=.gcno)

LIBPATH = $(LIBARYS_PATH)/SDL2-$(SDL_VER)/lib $(LIBARYS_PATH)/SDL2_ttf-$(SDLTTF_VER)/lib $(BOOST_LIB_PATH) $(LIBARYS_PATH)/tbb$(TBB_VER)/build/build_release
LIBRARY = boost_system boost_serialization boost_unit_test_framework tbb

# All
all:: tests.out tests.txt
all_test_outputs:: $(TEST_OUTPUTS)

# Compile source code to object files
.cc.o :
	$(CPP) $(OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@

# Link the object files to build the tests
tests.out : $(TEST_OBJECTS)
	$(CPP) --coverage $(TEST_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $@ 

# Run all tests
tests.txt : tests.out
	./$< > $@ 2>&1

test_output : tests.out
	./$<


# Generate html coverage report
index.html : tests.out
	./run_lcov.sh $<

# Build individual test suites
.cc.out :
	$(CPP) $(OPT) -DSTAND_ALONE $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) \
	$< $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $@ 

# Run the tests
.out.txt :
	./$< &> $@

# Clean targets
clean ::
	$(RM) $(OBJECTS)
	$(RM) $(EXES)
	$(RM) $(TEST_OUTPUTS)
	$(RM) tests.out tests.txt
	$(RM) $(GCDA)
	$(RM) $(GCNO)

spotless ::
	$(RM) $(OBJECTS)
	$(RM) $(EXES)
	$(RM) $(TEST_OUTPUTS)
	$(RM) tests.out tests.txt
	$(RM) $(GCDA)
	$(RM) $(GCNO)