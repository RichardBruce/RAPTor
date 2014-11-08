CPP = g++
CC = gcc

# Compile and linker options 
COMMON_OPT  = -fPIC -Wno-unused-local-typedefs -Wall -pipe
DEBUG_OPT   = $(COMMON_OPT) -g3 -O0 --coverage
RELEASE_OPT = $(COMMON_OPT) -O3 -mfpmath=sse -msse4a -fexpensive-optimizations #-march=native
ifeq ($(BUILD), DEBUG)
	LD_OPT = --coverage
	CC_OPT = $(DEBUG_OPT) --std=c11
	CPP_OPT = $(DEBUG_OPT) --std=c++1y
else
	LD_OPT =
	CC_OPT = $(RELEASE_OPT) --std=c11
	CPP_OPT = $(RELEASE_OPT) --std=c++1y
endif

# Derived files
C_SOURCE = $(filter-out %.cc, $(SOURCE))
CPP_SOURCE = $(filter-out %.c, $(SOURCE))

C_OBJECTS = $(C_SOURCE:%.c=build/%.o)
CPP_OBJECTS = $(CPP_SOURCE:%.cc=build/%.o)

# Compile source code to object files
build :
	@mkdir build

build/%.o : %.cc | build
	$(CPP) $(CPP_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@

build/%.o : %.c | build
	$(CC) $(CC_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@

# Link objects to libraries
%.so : $(CPP_OBJECTS) $(C_OBJECTS)
	$(CPP) -fvisibility=default -shared -Wl,-soname,$@.1.0.1 -o $@.1.0.1 $(CPP_OBJECTS) $(C_OBJECTS) $(patsubst %,-L%,$(LIBPATH)) $(patsubst %,-l%,$(SO_LIBS))
	ln -s $@.1.0.1 $@
	ln -s $@.1.0.1 $@.1

%.a : $(CPP_OBJECTS) $(C_OBJECTS)
	ar rcs $@ $(CPP_OBJECTS) $(C_OBJECTS)

# Lint code
cppcheck.xml : $(SOURCE)
	cppcheck  $(patsubst %,-I%,$(LOCAL_INCLUDES)) $(patsubst %,-D%,$(LINTER_DEFINES)) -D__GNUC__ --enable=all --error-exitcode=1 --force --inline-suppr --suppress=missingIncludeSystem --xml-version=2 ./ 2> cppcheck.xml
