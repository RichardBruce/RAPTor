#!/usr/bin/env bash
COVERAGE_FILE="./test_coverage/coverage.info"
RUN_COVERAGE_FILE_0="./test_coverage/run_coverage_0.info"
RUN_COVERAGE_FILE_1="./test_coverage/run_coverage_1.info"
BASE_COVERAGE_FILE="./test_coverage/base_coverage.info"

# Clear the last run
rm -rf ./test_coverage
mkdir ./test_coverage

# Set Counters based on the app coverage files
if [ -z "$2" ] || [ "$2" == "-" ]; then
    lcov -q -b . -d ./build -z -q
else
    lcov -q -b . -d $2 -c -i -o $BASE_COVERAGE_FILE
fi

# Run tests and collect coverage
./$1
lcov -q -b . -d ./build -c -o $RUN_COVERAGE_FILE_0
if [ ! -z "$2" ] && [ "$2" != "-" ]; then
    lcov -q -b . -d $2 -c -o $RUN_COVERAGE_FILE_1
fi

# Merge app and test
if [ -z "$2" ] || [ "$2" == "-" ]; then
    mv $RUN_COVERAGE_FILE_0 $COVERAGE_FILE
else
    lcov -a $BASE_COVERAGE_FILE -a $RUN_COVERAGE_FILE_0 -a $RUN_COVERAGE_FILE_1 -o $COVERAGE_FILE
fi

# Remove output for external libraries and test framework
lcov --remove $COVERAGE_FILE "/usr*" -q -o $COVERAGE_FILE
lcov --remove $COVERAGE_FILE "*boost*" -q -o $COVERAGE_FILE
lcov --remove $COVERAGE_FILE "*tbb*" -q -o $COVERAGE_FILE
lcov --remove $COVERAGE_FILE "*tests*" -q -o $COVERAGE_FILE
lcov --remove $COVERAGE_FILE "*teamcity*" -q -o $COVERAGE_FILE

# Clear and regenerate html
genhtml -q -o ./test_coverage --legend -t "Test Coverage" --num-spaces 4 $COVERAGE_FILE

# Publish code coverage
if [ ! -z "$3" ]; then
    TEST_FOLDER="${PWD##*/}"
    PROJECT_PATH="${PWD%/*}"
    COPY_TO="$3/${PROJECT_PATH##*/}/$TEST_FOLDER"
    if [ ! -d "$DIRECTORY" ]; then
        mkdir -p $COPY_TO
    fi
    cp -rf ./test_coverage/* $COPY_TO
fi
