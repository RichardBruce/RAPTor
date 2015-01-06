#!/usr/bin/env bash
COVERAGE_FILE="./test_coverage/coverage.info"
UNIT_COVERAGE_FILE="./unit_tests/test_coverage/coverage.info"
REGRESSION_COVERAGE_FILE="./regression_tests/test_coverage/coverage.info"


# Clear the last run
rm -rf ./test_coverage
mkdir ./test_coverage

# Merge
lcov -a $UNIT_COVERAGE_FILE -a $REGRESSION_COVERAGE_FILE -o $COVERAGE_FILE

# Clear and regenerate html
genhtml -q -o ./test_coverage --legend -t "Test Coverage" --num-spaces 4 $COVERAGE_FILE

# Publish code coverage
if [ ! -z "$1" ]; then
    PROJECT_FOLDER="${PWD##*/}"
    GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
    COPY_TO="$1/$GIT_BRANCH/$2/$PROJECT_FOLDER/combined"
    if [ ! -d "$DIRECTORY" ]; then
        mkdir -p $COPY_TO
    fi
    cp -rf ./test_coverage/* $COPY_TO
fi