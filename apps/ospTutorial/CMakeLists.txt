# Copyright 2018-2020 Intel Corporation SPDX-License-Identifier: Apache-2.0

if(NOT OSPRAY_APPS_EXAMPLES)
  return()
endif()

find_package(Boost 1.42.0 REQUIRED COMPONENTS system thread regex)
include_directories(${Boost_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(IRTS_SOURCES ${PROJECT_SOURCE_DIR}/http-parser/http_parser.c
                 ${PROJECT_SOURCE_DIR}/jsoncpp/jsoncpp.cpp)

# build ospTutorial, for demonstration and API testing
add_executable(ospTutorial ${OSPRAY_RESOURCE} ospTutorial.c)
target_link_libraries(ospTutorial PRIVATE ospray)

# C++ version
add_executable(ospTutorialCpp ${OSPRAY_RESOURCE} ospTutorial.cpp
                              ${IRTS_SOURCES})
target_link_libraries(ospTutorialCpp PRIVATE ospray ${Boost_LIBRARIES})

# async version
add_executable(ospTutorialAsync ${OSPRAY_RESOURCE} ospTutorialAsync.c)
target_link_libraries(ospTutorialAsync PRIVATE ospray)

install(TARGETS ospTutorial
                ospTutorialCpp
                ospTutorialAsync
                DESTINATION
                ${CMAKE_INSTALL_BINDIR}
                COMPONENT
                apps)
