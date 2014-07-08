# Include common makefile
ifndef BUILD
BUILD=DEBUG
endif
include $(RAPTOR_TOOLS)/Common.mk

# Derived files
TEST_AND_UUT_SOURCE = $(TEST_SOURCE) $(SOURCE)
TEST_OBJECTS = $(filter-out %.c, $(filter-out %.cc, $(TEST_AND_UUT_SOURCE:%.cc=build/%.o) $(TEST_AND_UUT_SOURCE:%.c=build/%.o)))
EXES = $(filter-out %.c, $(TEST_SOURCE:.cc=.out))

# Includes
INCLUDE += $(LIBARYS_PATH)/teamcity ../
vpath %.c $(INCLUDE)
vpath %.cc $(INCLUDE)

# Libraries
LIBRARY += boost_unit_test_framework

# Defines
DEFINES += BOOST_TEST_DYN_LINK

# Link the object files to build the tests
main.out : $(TEST_OBJECTS) | build
	$(CPP) $(LD_OPT) $(TEST_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $@ 

# Build individual test suites
define test_suite_template
$(1) : $(patsubst %, build/%, $(filter-out $(patsubst %.out, %.o, $(1)), $(2))) $(patsubst %.out, %.cc, $(1)) | build
	$(CPP) $(CPP_OPT) -DSTAND_ALONE $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $(patsubst %.out, %.cc, $(1)) -o $(patsubst %.out, build/%.o, $(1))
	$(CPP) $(LD_OPT) $(patsubst %, build/%, $(filter-out $(patsubst %.out, %.o, $(1)), $(2))) $(patsubst %.out, build/%.o, $(1)) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(LIBRARY)) -o $(1) 
endef

# Clean targets
clean ::
	$(RM) -r build
	$(RM) $(EXES)

spotless ::
	$(RM) -r test_coverage
	$(RM) -r build
	$(RM) $(EXES)
