# Include project info makefile
include ../Project.mk

# Uut objects
UUT_OBJECTS = $(filter-out %main.o, $(C_OBJECTS:build/%.o=%.o) $(CPP_OBJECTS:build/%.o=%.o))

# Source
TEST_SOURCE = teamcity_boost.cc teamcity_messages.cc

LIBRARY += boost_filesystem

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out
