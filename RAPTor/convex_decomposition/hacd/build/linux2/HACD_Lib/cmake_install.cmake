# Install script for directory: /home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/output")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE STATIC_LIBRARY FILES "/home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/HACD_Lib/libHACD_LIB.a")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdMeshDecimator.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdHACD.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdRaycastMesh.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdSArray.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdVector.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdCircularList.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdManifoldMesh.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdMicroAllocator.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdVersion.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdICHull.h"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdGraph.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdVector.inl"
    "/home/bruce/work/RAPTor/terrain_generator/hacd/src/HACD_Lib/inc/hacdCircularList.inl"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

