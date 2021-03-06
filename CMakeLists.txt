## ======================================================================== ##
## Copyright 2009-2018 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

PROJECT(OSPRay)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
CMAKE_POLICY(SET CMP0003 NEW) # since 2.6
IF(POLICY CMP0015)
  CMAKE_POLICY(SET CMP0015 NEW) # since 2.8.1
ENDIF()
IF(POLICY CMP0020)
  CMAKE_POLICY(SET CMP0020 NEW) # since 2.8.11
ENDIF()
IF(POLICY CMP0042)
  CMAKE_POLICY(SET CMP0042 NEW) # since 3.0
ENDIF()
IF(POLICY CMP0054)
  CMAKE_POLICY(SET CMP0054 OLD) # since 3.1
ENDIF()

# configure directories for components we are going to include
SET(OSPRAY_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(COMPONENTS_DIR ${OSPRAY_DIR}/components)
SET(OSPCOMMON_DIR ${COMPONENTS_DIR}/ospcommon)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${OSPCOMMON_DIR}/cmake)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

##############################################################
# CMake modules and macro files
##############################################################

INCLUDE(macros)
INCLUDE(ospray_macros)
INCLUDE(ospray_options)
INCLUDE(ispc)

if(MSVC)
  add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS -DWIN32_LEAN_AND_MEAN)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake/common)
add_subdirectory(libzmq)
add_subdirectory(ZeroEQ)
add_subdirectory(ZeroBuf)
find_package(ZeroEQ CONFIG REQUIRED)
find_package(ZeroBuf CONFIG REQUIRED)
find_package(LibJpegTurbo 1.4 REQUIRED)

##############################################################
# the OSPRay 'common' library
##############################################################
ADD_SUBDIRECTORY(${OSPCOMMON_DIR})

##############################################################
# OSPRay components
#
# 'components' work similar to 'packages' in linux distributions: each
# component is its own little piece that builds some applications,
# libraries, etc, and may have dependencies to other components.
##############################################################
INCLUDE(components/configure.cmake)

##############################################################
# the OSPRay library
##############################################################

ADD_SUBDIRECTORY(ospray)

##############################################################
# OSPRay sample apps; at the end: they may need modules
##############################################################
IF (OSPRAY_ENABLE_APPS)
  ADD_SUBDIRECTORY(apps)
ENDIF()

##############################################################
# OSPRay modules
# modules are OPTTIONAL and EXTERNAL packages that
# users can link into ospray. they can use any and all pieces of
# ospray but obviously no ospray parts may ever depend on a
# module (which by definition is OPTIONAL) - so those go
# AT THE BACK, only AFTER ALL OTHER PARTS OF OSPRAY ARE BUILT
##############################################################
ADD_SUBDIRECTORY(modules)

##############################################################
# Google Test
##############################################################
IF (OSPRAY_ENABLE_TESTING)
  ADD_SUBDIRECTORY(tests)
ENDIF()

##############################################################
# add target for running clang-format
##############################################################
OPTION(OSPRAY_ENABLE_TARGET_CLANGFORMAT
       "Enable 'format' target, requires clang-format too" OFF)
INCLUDE(clang-format)

# create a configure file that both ospray and ispc can read the cmake config
# from needs to be at the end, after all cache variables have been set
CONFIGURE_FILE(ospray/common/OSPConfig.h.in OSPConfig.h)
INSTALL(FILES ${CMAKE_BINARY_DIR}/OSPConfig.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ospray
  COMPONENT devel
)

# has to be last
INCLUDE(CPack)
