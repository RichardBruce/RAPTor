cmake_minimum_required(VERSION 2.8.9)
project (raptor)

# Find boost
set(BOOSTROOT ${CMAKE_CURRENT_LIST_DIR}/lib)
find_package(Boost 1.61.0 COMPONENTS thread filesystem system log program_options serialization unit_test_framework)

# Compiler flags
set(CMAKE_C_FLAGS "--std=c11 -fPIC -Wno-deprecated -Wno-unused-variable -Wno-maybe-uninitialized -Wall -pipe")
set(CMAKE_C_FLAGS_DEBUG "-g3 -Og")
set(CMAKE_C_FLAGS_RELEASE "-O3 -g -mfpmath=sse -msse4a -mpopcnt -fexpensive-optimizations")

set(CMAKE_CXX_FLAGS "--std=c++17 -fPIC -Wno-deprecated -Wno-unused-variable -Wno-maybe-uninitialized -Wall -pipe")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Og")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -mfpmath=sse -msse4a -mpopcnt -fexpensive-optimizations")
if(COVERAGE STREQUAL "Yes")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

# Compiler definitions
add_definitions(-DREFLECTIONS_ON)
add_definitions(-DREFRACTIONS_ON)
add_definitions(-DSIMD_PACKET_TRACING)
add_definitions(-DFRUSTRUM_CULLING)
add_definitions(-DEXACT_NORMALISE)
add_definitions(-DBOOST_LOG_LEVEL=boost::log::trivial::trace)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_TEST_DYN_LINK)
# add_definitions(-DTHREADED_RAY_TRACE)
# add_definitions(-DLOG_DEPTH)
# add_definitions(-DSIMD_PACKET_TRACING)
# add_definitions(-DFRUSTRUM_CULLING)
# add_definitions(-DSHOW_KD_TREE)
# add_definitions(-DDIFFUSE_REFLECTIONS=128.0)
# add_definitions(-DSOFT_SHADOW=256.0 )
if(VALGRIND STREQUAL "Yes")
    add_definitions(-DVALGRIND_TESTS)
endif()

SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/deploy)


# Functions
function(add_unit_test tests)
    set(source unit_tests/main.cc)
    foreach(test_name ${tests})
        add_executable(${test_name} unit_tests/${test_name}.cc)
        target_link_libraries(${test_name} ${PROJECT_NAME}::static)
        target_compile_definitions(${test_name} PRIVATE STAND_ALONE=1)

        add_custom_target(run_${test_name} DEPENDS ${test_name} COMMAND ${test_name})

        set(source ${source} unit_tests/${test_name}.cc)
        
        install(TARGETS ${test_name} DESTINATION ${PROJECT_NAME}/bin)
    endforeach(test_name)

    add_executable(${PROJECT_NAME}_unit_tests ${source})
    target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME}::static)
    install(TARGETS ${PROJECT_NAME}_unit_tests DESTINATION ${PROJECT_NAME}/bin)

    add_custom_target(run_${PROJECT_NAME}_unit_tests DEPENDS ${PROJECT_NAME}_unit_tests COMMAND ${PROJECT_NAME}_unit_tests)
endfunction(add_unit_test)

function(add_regression_test tests)
    set(source regression_tests/main.cc)
    foreach(test_name ${tests})
        add_executable(${test_name} regression_tests/${test_name}.cc)
        target_link_libraries(${test_name} ${PROJECT_NAME}::static)
        target_compile_definitions(${test_name} PRIVATE STAND_ALONE=1)
        add_custom_target(run_${test_name} DEPENDS ${test_name} COMMAND ${test_name})

        set(source ${source} regression_tests/${test_name}.cc)
        
        install(TARGETS ${test_name} DESTINATION ${PROJECT_NAME}/bin)
    endforeach(test_name)

    include_directories(${Boost_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME}_regression_tests ${source})
    target_link_libraries(${PROJECT_NAME}_regression_tests ${PROJECT_NAME}::static)
    install(TARGETS ${PROJECT_NAME}_regression_tests DESTINATION ${PROJECT_NAME}/bin)

    add_custom_target(run_${PROJECT_NAME}_regression_tests DEPENDS ${PROJECT_NAME}_regression_tests COMMAND ${PROJECT_NAME}_regression_tests)
endfunction(add_regression_test)

function(add_analysis sources headers)
    # Get the include files to also feed to cppcheck
    foreach(dir ${headers})
        LIST(APPEND cppcheck_includes "-I${dir}")
    endforeach()

    # add a custom _target_analysis target
    add_custom_target(${PROJECT_NAME}_analysis)
    set_target_properties(${PROJECT_NAME}_analysis PROPERTIES EXCLUDE_FROM_ALL TRUE)
    
    # add the cppcheck command to the target
    set(CPPCHECK_ARG -DDIFFUSE_REFLECTIONS=128.0 -DSOFT_SHADOW=256.0 --enable=all --error-exitcode=1 --force --inline-suppr --xml-version=2)
    add_custom_command(TARGET ${PROJECT_NAME}_analysis PRE_BUILD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMAND cppcheck ${CPPCHECK_ARG} ${cppcheck_includes} ${sources} 2> ${PROJECT_NAME}_cppcheck.xml
        DEPENDS ${sources}
        COMMENT "Running cppcheck: ${PROJECT_NAME}"
        VERBATIM)
endfunction()

function(add_libs headers source libs)
    # Headers
    add_library(${PROJECT_NAME}_headers INTERFACE)
    target_include_directories(${PROJECT_NAME}_headers INTERFACE ${headers})
    add_library(${PROJECT_NAME}::headers ALIAS ${PROJECT_NAME}_headers)
    install(DIRECTORY ${headers} DESTINATION ${PROJECT_NAME}/include FILES_MATCHING PATTERN "*.h")

    # Libraries
    add_library(${PROJECT_NAME}_static STATIC ${source})
    target_link_libraries(${PROJECT_NAME}_static ${libs})
    set_target_properties(${PROJECT_NAME}_static PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
    add_library(${PROJECT_NAME}::static ALIAS ${PROJECT_NAME}_static)
    install(TARGETS ${PROJECT_NAME}_static DESTINATION ${PROJECT_NAME}/lib)

    add_library(${PROJECT_NAME}_shared SHARED ${source})
    target_link_libraries(${PROJECT_NAME}_shared ${libs})
    set_target_properties(${PROJECT_NAME}_shared PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
    add_library(${PROJECT_NAME}::shared ALIAS ${PROJECT_NAME}_shared)
    install(TARGETS ${PROJECT_NAME}_shared DESTINATION ${PROJECT_NAME}/lib)
endfunction(add_libs)

function(add_exe_and_libs headers source libs)
    add_libs("${headers}" "${source}" "${libs}")

    # Executable
    add_executable(${PROJECT_NAME} ${source} main.cc)
    target_link_libraries(${PROJECT_NAME} ${libs})
    add_custom_target(run_${PROJECT_NAME} COMMAND ${PROJECT_NAME} DEPENDS ${PROJECT_NAME})
    add_custom_target(time_run_${PROJECT_NAME} COMMAND /usr/bin/time ./${PROJECT_NAME} DEPENDS ${PROJECT_NAME})
    add_custom_target(valgrind_${PROJECT_NAME} COMMAND valgrind ./${PROJECT_NAME} DEPENDS ${PROJECT_NAME})
    add_custom_target(callgrind_${PROJECT_NAME} COMMAND valgrind --tool=callgrind ./${PROJECT_NAME} DEPENDS ${PROJECT_NAME})
    add_custom_target(sgcheck_${PROJECT_NAME} COMMAND valgrind --tool=exp-sgcheck ./${PROJECT_NAME} DEPENDS ${PROJECT_NAME})

    install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_NAME}/bin)
endfunction(add_exe_and_libs)

function(add_shaders shaders)
    foreach(SHADER ${shaders})
        get_filename_component(FILE_NAME ${SHADER} NAME)
        set(SPIRV "${PROJECT_BINARY_DIR}/${FILE_NAME}.spv")
        add_custom_command(
            OUTPUT ${SPIRV}
            COMMAND glslangValidator -V ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER} -o ${SPIRV}
            DEPENDS ${SHADER}
            VERBATIM)
        list(APPEND SPIRV_BINARY_FILES ${SPIRV})
    endforeach(SHADER)

    add_custom_target(shaders_${PROJECT_NAME} DEPENDS ${SPIRV_BINARY_FILES})
    add_dependencies(${PROJECT_NAME} shaders_${PROJECT_NAME})
    install(FILES ${SPIRV_BINARY_FILES} DESTINATION ${PROJECT_NAME}/bin)
endfunction(add_shaders)

# Sub projects
add_subdirectory(RAPTor)
