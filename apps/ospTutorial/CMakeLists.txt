# Copyright 2018-2020 Intel Corporation SPDX-License-Identifier: Apache-2.0

if(NOT OSPRAY_APPS_EXAMPLES)
  return()
endif()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(LibJpegTurbo 1.4 REQUIRED)
find_package(Boost REQUIRED thread)

set(PROTO ${CMAKE_CURRENT_SOURCE_DIR}/proto/FeatureCollection.proto)
set(PROTO_CPP ${CMAKE_CURRENT_BINARY_DIR}/proto/FeatureCollection.pb.cc)
add_custom_command(OUTPUT ${PROTO_CPP}
                   DEPENDS protobuf::protoc ${PROTO}
                   COMMAND protobuf::protoc
                           ${PROTO}
                           --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
                           --cpp_out=${CMAKE_CURRENT_BINARY_DIR})

set(IRTS_SOURCES ${PROJECT_SOURCE_DIR}/http-parser/http_parser.c
                 ${PROJECT_SOURCE_DIR}/jsoncpp/jsoncpp.cpp ${PROTO_CPP})

if(MSVC)
  # disable auto-linking and use cmake's dependency handling:
  add_definitions(-DBOOST_ALL_NO_LIB)
  # explicitly specify that a dynamic Boost is used:
  add_definitions(-DBOOST_ALL_DYN_LINK)
endif(MSVC)

# C++ version
add_executable(ospTutorialCpp
               ${OSPRAY_RESOURCE}
               ospTutorial.cpp
               ${IRTS_SOURCES}
               ${PROTO_SOURCES})

target_include_directories(ospTutorialCpp SYSTEM
                           PRIVATE ${PROJECT_SOURCE_DIR}
                                   ${LibJpegTurbo_INCLUDE_DIRS}
                                   ${CMAKE_CURRENT_BINARY_DIR})

target_sources(ospTutorialCpp PRIVATE
    connection.h
    webscene.h
)

target_link_libraries(ospTutorialCpp
                      PRIVATE ospray
                              Boost::thread
                              ${LibJpegTurbo_LIBRARIES}
                              protobuf::libprotobuf)

install(TARGETS ospTutorialCpp
                DESTINATION
                ${CMAKE_INSTALL_BINDIR}
                COMPONENT
                apps)
