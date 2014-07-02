CPP = g++
CC = gcc

# Compile and linker options 
COMMON_OPT  = -std=c++1y -fPIC -Wno-unused-local-typedefs -Wall -pipe
DEBUG_OPT   = $(COMMON_OPT) -g3 -O0 --coverage
RELEASE_OPT = $(COMMON_OPT) -O3 -mfpmath=sse -msse4a -fexpensive-optimizations #-march=native
ifeq ($(BUILD), DEBUG)
	LD_OPT = --coverage
	CC_OPT = $(DEBUG_OPT)
else
	LD_OPT =
	CC_OPT = $(RELEASE_OPT)
endif

# Derived files
OBJECTS = $(SOURCE:%.cc=build/%.o)

# Compile source code to object files
build/%.o : %.cc
	$(CPP) $(CC_OPT) $(patsubst %,-D%,$(DEFINES)) $(patsubst %,-I%,$(INCLUDE)) -c $< -o $@

# Lint code
cppcheck.xml ::
	cppcheck  $(patsubst %,-I%,$(LOCAL_INCLUDES)) --enable=all --error-exitcode=1 --force --inline-suppr --suppress=missingIncludeSystem --xml-version=2 ./ 2> cppcheck.xml
