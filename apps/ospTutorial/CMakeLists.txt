# Copyright 2018-2020 Intel Corporation SPDX-License-Identifier: Apache-2.0

if(NOT OSPRAY_APPS_EXAMPLES)
  return()
endif()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Protobuf REQUIRED CONFIG)
find_package(LibJpegTurbo 1.4 REQUIRED)
find_package(Boost 1.42.0 REQUIRED thread)

set(IRTS_SOURCES ${PROJECT_SOURCE_DIR}/http-parser/http_parser.c
                 ${PROJECT_SOURCE_DIR}/jsoncpp/jsoncpp.cpp)

# Generate cpp files from proto file
protobuf_generate(
	LANGUAGE cpp
	OUT_VAR PROTO_SOURCES
	PROTOS proto/FeatureCollection.proto
	PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}
)

if(MSVC)
	# disable auto-linking and use cmake's dependency handling:
	add_definitions( -DBOOST_ALL_NO_LIB )
	# explicitly specify that a dynamic Boost is used:
	add_definitions( -DBOOST_ALL_DYN_LINK )
endif(MSVC)

# C++ version
add_executable(ospTutorialCpp ${OSPRAY_RESOURCE} ospTutorial.cpp
                              ${IRTS_SOURCES}
                              ${PROTO_SOURCES})

target_include_directories(ospTutorialCpp SYSTEM
                           PRIVATE 
                                   ${PROJECT_SOURCE_DIR}
                                   ${LibJpegTurbo_INCLUDE_DIRS}
                                   ${CMAKE_CURRENT_BINARY_DIR})

message(STATUS ${Boost_LIBRARIES})

target_link_libraries(ospTutorialCpp
                      PRIVATE ospray Boost::thread
                              ${LibJpegTurbo_LIBRARIES}
                              protobuf::libprotobuf)

install(TARGETS ospTutorialCpp
                DESTINATION
                ${CMAKE_INSTALL_BINDIR}
                COMPONENT
                apps)
