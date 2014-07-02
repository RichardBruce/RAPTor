#!/usr/bin/env bash
echo "Source Code" > lines_of_code
cloc --by-file-by-lang --exclude-dir=./unit_tests,./regression_tests,./test_coverage ./ >> lines_of_code
echo "Test Code" >> lines_of_code
cloc --by-file-by-lang --exclude-dir=test_coverage,test_data ./unit_tests ./regression_tests >> lines_of_code