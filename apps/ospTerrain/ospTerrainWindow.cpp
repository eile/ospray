#include "ospTerrainWindow.h"
#include "imgui_impl_glfw_gl3.h"
#include "ospray_testing.h"
#include "imgui.h"
#include <iostream>
#include <stdexcept>

// on Windows often only GL 1.1 headers are present
#ifndef GL_CLAMP_TO_BORDER
#define GL_CLAMP_TO_BORDER 0x812D
#endif
#ifndef GL_SRGB_ALPHA
#define GL_SRGB_ALPHA 0x8C42
#endif
#ifndef GL_FRAMEBUFFER_SRGB
#define GL_FRAMEBUFFER_SRGB 0x8DB9
#endif

static bool g_quitNextFrame = false;


void error_callback(int error, const char *desc)
{
  std::cerr << "error " << error << ": " << desc << std::endl;
}

ospTerrainWindow *ospTerrainWindow::activeWindow = nullptr;

ospTerrainWindow::ospTerrainWindow(const vec2i &windowSize, bool denoiser)
    : denoiserAvailable(denoiser)
{
  if (activeWindow != nullptr) {
    throw std::runtime_error("Cannot create more than one ospTerrainWindow!");
  }

  activeWindow = this;

  glfwSetErrorCallback(error_callback);

  // initialize GLFW
  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize GLFW!");
  }

  glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);

  // create GLFW window
  glfwWindow = glfwCreateWindow(
      windowSize.x, windowSize.y, "OSPRay Tutorial", nullptr, nullptr);

  if (!glfwWindow) {
    glfwTerminate();
    throw std::runtime_error("Failed to create GLFW window!");
  }

  // make the window's context current
  glfwMakeContextCurrent(glfwWindow);

  ImGui_ImplGlfwGL3_Init(glfwWindow, true);

  // set initial OpenGL state
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  // create OpenGL frame buffer texture
  glGenTextures(1, &framebufferTexture);
  glBindTexture(GL_TEXTURE_2D, framebufferTexture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // set GLFW callbacks
  glfwSetFramebufferSizeCallback(
      glfwWindow, [](GLFWwindow *, int newWidth, int newHeight) {
        activeWindow->reshape(vec2i{newWidth, newHeight});
      });

  glfwSetCursorPosCallback(glfwWindow, [](GLFWwindow *, double x, double y) {
    ImGuiIO &io = ImGui::GetIO();
    if (!io.WantCaptureMouse) {
      activeWindow->motion(vec2f{float(x), float(y)});
    }
  });

  glfwSetKeyCallback(
      glfwWindow, [](GLFWwindow *, int key, int, int action, int) {
        if (action == GLFW_PRESS) {
          switch (key) {
          case GLFW_KEY_Q:
            g_quitNextFrame = true;
            break;
          }
        }
      });

  glfwSetMouseButtonCallback(
      glfwWindow, [](GLFWwindow *, int button, int action, int /*mods*/) {
        auto &w = *activeWindow;
        if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
          auto mouse = activeWindow->previousMouse;
          auto windowSize = activeWindow->windowSize;
          const vec2f pos(mouse.x / static_cast<float>(windowSize.x),
              1.f - mouse.y / static_cast<float>(windowSize.y));
           
          auto res =
              w.framebuffer.pick(w.scene.renderer, w.camera, w.scene.world, pos);

          if (res.hasHit) {
            std::cout << "Picked geometry [inst: " << res.instance
                      << ", model: " << res.model << ", prim: " << res.primID
                      << "]" << std::endl;
          }
        }
      });
  
  // OSPRay setup //
  this->scene.refreshScene();
  //refreshScene(true);

  if (true) {
    // create the arcball camera model
    arcballCamera.reset(new ArcballCamera(this->scene.world.getBounds(), windowSize));

    // init camera
    updateCamera();
    camera.commit();
  }

  // trigger window reshape events with current window size
  glfwGetFramebufferSize(glfwWindow, &this->windowSize.x, &this->windowSize.y);
  reshape(this->windowSize);

  bool res = this->scene.commitOutstandingHandles();
  if (res) {
    framebuffer.resetAccumulation();
  }
}

ospTerrainWindow::~ospTerrainWindow()
{
  ImGui_ImplGlfwGL3_Shutdown();
  // cleanly terminate GLFW
  glfwTerminate();
}

ospTerrainWindow *ospTerrainWindow::getActiveWindow()
{
  return activeWindow;
}

void ospTerrainWindow::mainLoop()
{
  // continue until the user closes the window
  startNewOSPRayFrame();

  while (!glfwWindowShouldClose(glfwWindow) && !g_quitNextFrame) {
    ImGui_ImplGlfwGL3_NewFrame();

    display();

    // poll and process events
    glfwPollEvents();
  }

  waitOnOSPRayFrame();
}

void ospTerrainWindow::reshape(const vec2i &newWindowSize)
{
  windowSize = newWindowSize;

  // create new frame buffer
  auto buffers = OSP_FB_COLOR | OSP_FB_DEPTH | OSP_FB_ACCUM | OSP_FB_ALBEDO
      | OSP_FB_NORMAL;
  framebuffer = cpp::FrameBuffer(windowSize, OSP_FB_RGBA32F, buffers);

  refreshFrameOperations();

  // reset OpenGL viewport and orthographic projection
  glViewport(0, 0, windowSize.x, windowSize.y);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0, windowSize.x, 0.0, windowSize.y, -1.0, 1.0);

  // update camera
  arcballCamera->updateWindowSize(windowSize);

  camera.setParam("aspect", windowSize.x / float(windowSize.y));
  camera.commit();
}

void ospTerrainWindow::updateCamera()
{
  camera.setParam("aspect", windowSize.x / float(windowSize.y));
  camera.setParam("position", arcballCamera->eyePos());
  camera.setParam("direction", arcballCamera->lookDir());
  camera.setParam("up", arcballCamera->upDir());
}

void ospTerrainWindow::motion(const vec2f &position)
{
  const vec2f mouse(position.x, position.y);
  if (previousMouse != vec2f(-1)) {
    const bool leftDown =
        glfwGetMouseButton(glfwWindow, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool rightDown =
        glfwGetMouseButton(glfwWindow, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    const bool middleDown =
        glfwGetMouseButton(glfwWindow, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    const vec2f prev = previousMouse;

    bool cameraChanged = leftDown || rightDown || middleDown;

    if (leftDown) {
      const vec2f mouseFrom(clamp(prev.x * 2.f / windowSize.x - 1.f, -1.f, 1.f),
          clamp(prev.y * 2.f / windowSize.y - 1.f, -1.f, 1.f));
      const vec2f mouseTo(clamp(mouse.x * 2.f / windowSize.x - 1.f, -1.f, 1.f),
          clamp(mouse.y * 2.f / windowSize.y - 1.f, -1.f, 1.f));
      arcballCamera->rotate(mouseFrom, mouseTo);
    } else if (rightDown) {
      arcballCamera->zoom(mouse.y - prev.y);
    } else if (middleDown) {
      arcballCamera->pan(vec2f(mouse.x - prev.x, prev.y - mouse.y));
    }

    if (cameraChanged) {
      if (cancelFrameOnInteraction) {
        currentFrame.cancel();
        waitOnOSPRayFrame();
      }
      updateCamera();
      this->scene.addObjectToCommit(camera.handle());
    }
  }

  previousMouse = mouse;
}

void ospTerrainWindow::display()
{
  updateTitleBar();

  // Turn on SRGB conversion for OSPRay frame
  glEnable(GL_FRAMEBUFFER_SRGB);

  static bool firstFrame = true;
  if (firstFrame || currentFrame.isReady()) {
    waitOnOSPRayFrame();

    latestFPS = 1.f / currentFrame.duration();

    auto *fb = framebuffer.map(showAlbedo ? OSP_FB_ALBEDO : OSP_FB_COLOR);

    const GLint glFormat = showAlbedo ? GL_RGB : GL_RGBA;
    glBindTexture(GL_TEXTURE_2D, framebufferTexture);
    glTexImage2D(GL_TEXTURE_2D,
        0,
        glFormat,
        windowSize.x,
        windowSize.y,
        0,
        glFormat,
        GL_FLOAT,
        fb);

    framebuffer.unmap(fb);

    bool res = this->scene.commitOutstandingHandles();
    if (res) {
      framebuffer.resetAccumulation();
    }

    startNewOSPRayFrame();
    firstFrame = false;
  }

  // clear current OpenGL color buffer
  glClear(GL_COLOR_BUFFER_BIT);

  // render textured quad with OSPRay frame buffer contents
  glBegin(GL_QUADS);

  glTexCoord2f(0.f, 0.f);
  glVertex2f(0.f, 0.f);

  glTexCoord2f(0.f, 1.f);
  glVertex2f(0.f, windowSize.y);

  glTexCoord2f(1.f, 1.f);
  glVertex2f(windowSize.x, windowSize.y);

  glTexCoord2f(1.f, 0.f);
  glVertex2f(windowSize.x, 0.f);

  glEnd();

  // Disable SRGB conversion for UI
  glDisable(GL_FRAMEBUFFER_SRGB);

  // swap buffers
  glfwSwapBuffers(glfwWindow);
}

void ospTerrainWindow::startNewOSPRayFrame()
{
  if (updateFrameOpsNextFrame) {
    refreshFrameOperations();
    updateFrameOpsNextFrame = false;
  }
  currentFrame =
      framebuffer.renderFrame(this->scene.renderer, camera, this->scene.world);
}

void ospTerrainWindow::waitOnOSPRayFrame()
{
  currentFrame.wait();
}

void ospTerrainWindow::updateTitleBar()
{
  std::stringstream windowTitle;
  windowTitle << "OSPRay: " << std::setprecision(3) << latestFPS << " fps";
  if (latestFPS < 2.f) {
    float progress = currentFrame.progress();
    windowTitle << " | ";
    int barWidth = 20;
    std::string progBar;
    progBar.resize(barWidth + 2);
    auto start = progBar.begin() + 1;
    auto end = start + progress * barWidth;
    std::fill(start, end, '=');
    std::fill(end, progBar.end(), '_');
    *end = '>';
    progBar.front() = '[';
    progBar.back() = ']';
    windowTitle << progBar;
  }

  glfwSetWindowTitle(glfwWindow, windowTitle.str().c_str());
}

void ospTerrainWindow::refreshFrameOperations()
{
  if (denoiserEnabled) {
    cpp::ImageOperation d("denoiser");
    framebuffer.setParam("imageOperation", cpp::Data(d));
  } else {
    framebuffer.removeParam("imageOperation");
  }

  framebuffer.commit();
}
