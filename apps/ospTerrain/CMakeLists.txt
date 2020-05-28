## Copyright 2018-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

if (NOT OSPRAY_APPS_EXAMPLES)
  return()
endif()

find_package(LibJpegTurbo 1.4 REQUIRED)
# prefer libGL over libOpenGl for better compatibility with SWR
set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL 2 REQUIRED)
find_package(glfw3 REQUIRED)

set(HEADERS 
  ${PROJECT_SOURCE_DIR}/lerc/include/Lerc_c_api.h 
  ${PROJECT_SOURCE_DIR}/lerc/include/Lerc_types.h
)

include_directories(${PROJECT_SOURCE_DIR}/lerc/include)

add_executable(ospTerrain
  ${OSPRAY_RESOURCE}
  ospTerrainWindow.cpp
  imgui_impl_glfw_gl3.cpp
  ospTerrainMain.cpp
  ospTerrainScene.cpp
  Tile.cpp
  ${HEADERS}
)

target_include_directories(ospTerrain SYSTEM PRIVATE ${LibJpegTurbo_INCLUDE_DIRS})

message(STATUS "LibJpegTurbo_LIBRARIES: ${LibJpegTurbo_LIBRARIES}")
target_link_libraries(ospTerrain
PRIVATE
  arcball_camera
  ospray_testing
  ospray_imgui
  LercLib
  glfw
  ${OPENGL_LIBRARIES}
  ${LibJpegTurbo_LIBRARIES}/turbojpeg.lib
)

install(TARGETS ospTerrain
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  COMPONENT apps
)
