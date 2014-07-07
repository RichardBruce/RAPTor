# Include project info makefile
include ../Project.mk

# Uut objects
UUT_OBJECTS = $(filter-out %main.o, $(filter-out %.c, $(filter-out %.cc, $(SOURCE:%.cc=%.o) $(SOURCE:%.c=%.o))))

# Source
SOURCE  += nff_tests.cc \
    teamcity_boost.cc teamcity_messages.cc

include $(RAPTOR_TOOLS)/UnitTest.mk

# All
all:: main.out nff_tests.out

#$(eval $(call test_suite_template, clipping_tests.out, physics_common.o))
$(eval $(call test_suite_template, nff_tests.out, $(UUT_OBJECTS)))
