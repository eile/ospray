#pragma once

#include "ArcballCamera.h"
#include "GLFW/glfw3.h"
#include "ospray/ospray_cpp.h"
#include <functional>
#include "ospTerrainScene.h"

using namespace ospcommon::math;
using namespace ospray;

class ospTerrainWindow
{
 public:
  ospTerrainWindow(const vec2i &windowSize, bool denoiser = false);

  ~ospTerrainWindow();

  static ospTerrainWindow* getActiveWindow();

  void mainLoop();

 protected:
  void updateCamera();

  void reshape(const vec2i &newWindowSize);
  void motion(const vec2f &position);
  void display();
  void startNewOSPRayFrame();
  void waitOnOSPRayFrame();
  void updateTitleBar();
  void refreshFrameOperations();

  static ospTerrainWindow *activeWindow;

  vec2i windowSize;
  vec2f previousMouse{-1.f};

  bool denoiserAvailable{false};
  bool updateFrameOpsNextFrame{false};
  bool denoiserEnabled{false};
  bool showAlbedo{false};
  bool renderSunSky{false};
  bool cancelFrameOnInteraction{false};

  // GLFW window instance
  GLFWwindow *glfwWindow = nullptr;

  std::unique_ptr<ArcballCamera> arcballCamera;
  ospTerrainScene scene;

  cpp::Camera camera{"perspective"};
  cpp::Light sunSky{"sunSky"};
  cpp::FrameBuffer framebuffer;
  cpp::Future currentFrame;

  // OpenGL framebuffer texture
  GLuint framebufferTexture = 0;

  // FPS measurement of last frame
  float latestFPS{0.f};
};
