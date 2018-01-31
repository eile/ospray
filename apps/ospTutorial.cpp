// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

/* This is a small example tutorial how to use OSPRay in an application.
 *
 * On Linux build it in the build_directory with
 *   g++ ../apps/ospTutorial.cpp -I ../ospray/include -I .. ./libospray.so
 * -Wl,-rpath,. -o ospTutorial On Windows build it in the
 * build_directory\$Configuration with cl ..\..\apps\ospTutorial.cpp /EHsc -I
 * ..\..\ospray\include -I ..\.. -I ..\..\components ospray.lib
 */

#include "ospray/ospray_cpp.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#ifdef _WIN32
#define NOMINMAX
#include <malloc.h>
#else
#include <alloca.h>
#endif

#ifdef JPEG
#include <turbojpeg.h>
#endif

#include <zeroeq/URI.h>
#include <zeroeq/http/Server.h>
#include <zeroeq/http/helpers.h>
#include "data.h"

namespace http = zeroeq::http;

// helper function to write the rendered image as PPM file
void writePPM(const std::string &fileName,
              const ospcommon::vec2i &size,
              const uint32_t *pixel)
{
  FILE *file = fopen(fileName.c_str(), "wb");
  if (file == nullptr) {
    fprintf(stderr, "fopen('%s', 'wb') failed: %d", fileName.c_str(), errno);
    return;
  }
  fprintf(file, "P6\n%i %i\n255\n", size.x, size.y);
  unsigned char *out = (unsigned char *)alloca(3 * size.x);
  for (int y = 0; y < size.y; y++) {
    const unsigned char *in =
        (const unsigned char *)&pixel[(size.y - 1 - y) * size.x];
    for (int x = 0; x < size.x; x++) {
      out[3 * x + 0] = in[4 * x + 0];
      out[3 * x + 1] = in[4 * x + 1];
      out[3 * x + 2] = in[4 * x + 2];
    }
    fwrite(out, 3 * size.x, sizeof(char), file);
  }
  fprintf(file, "\n");
  fclose(file);
}

class Server
{
 public:
  Server() : _server(zeroeq::URI(":4242"))
  {
    _setupOSP();
    _setupZeroEQ();
  }

  void run()
  {
    while (true) {
      if (_server.receive(0)) {
        while (_server.receive(0))
          /*nop, drain*/;
        _render();
      } else
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

 private:
  void _setupOSP()
  {
    // create and setup camera
    const ospcommon::vec2i imgSize{1024, 768};
    _camera.set("aspect", imgSize.x / (float)imgSize.y);
    _camera.set("pos", ospcommon::vec3f{0.f, 0.f, 0.f});
    _camera.set("dir", ospcommon::vec3f{0.1f, 0.f, 1.f});
    _camera.set("up", ospcommon::vec3f{0.f, 1.f, 0.f});
    _camera.commit();  // commit each object to indicate modifications are done

    // create and setup light for Ambient Occlusion
    ospray::cpp::Light light = _renderer.newLight("ambient");
    light.commit();
    auto lightHandle = light.handle();
    ospray::cpp::Data lights(1, OSP_LIGHT, &lightHandle);
    lights.commit();

    // complete setup of renderer
    _renderer.set("aoSamples", 1);
    _renderer.set("bgColor", 1.0f);  // white, transparent
    _renderer.set("model", _world);
    _renderer.set("camera", _camera);
    _renderer.set("lights", lights);
    _renderer.commit();
  }

  void _setupZeroEQ()
  {
    _server.handle(
        http::Method::POST, "points", [this](const http::Request &request) {
          return _handlePoints(request);
        });
    _server.handle(
        http::Method::POST, "camera", [this](const http::Request &request) {
          return _handleCamera(request);
        });
    std::cerr << "Bound to " << _server.getURI() << std::endl;
  }

  void _render()
  {
    // create and setup framebuffer
    _framebuffer.clear(OSP_FB_COLOR | OSP_FB_ACCUM);

    // render 10 frames, which are accumulated to result in a better
    // converged image
    for (int frames = 0; frames < 10; frames++) {
      _renderer.renderFrame(_framebuffer, OSP_FB_COLOR | OSP_FB_ACCUM);
      std::cout << 'r' << std::flush;
    }

    uint32_t *fb = (uint32_t *)_framebuffer.map(OSP_FB_COLOR);
    const ospcommon::vec2i imgSize{1024, 768};
    static size_t num = 0;
    writePPM(
        std::string("accumulatedFrameCpp") + std::to_string(num++) + ".ppm",
        imgSize,
        fb);
    std::cout << 'p' << std::flush;

#ifdef JPEG
    _jpeg.resize(TJBUFSIZE(imgSize[0], imgSize[1]));
    unsigned long size = 0;
    tjCompress(_encoder,
               reinterpret_cast<unsigned char *>(fb),
               imgSize[0],
               imgSize[0] * 3,
               imgSize[1],
               3,
               _jpeg.getData(),
               &size,
               TJ_444,
               0.9,
               TJ_RGB);
    _jpeg.resize(size);
#endif
    _framebuffer.unmap(fb);
  }

  std::future<http::Response> _handlePoints(const http::Request &request)
  {
    ospray::data::Points points;
    points.fromJSON(request.body);

    const auto &origin      = points.getOrigin();
    static const double g[] = {origin.get0(), origin.get1(), origin.get2()};
    _global[0]              = g[0];
    _global[1]              = g[1];
    _global[2]              = g[2];

    const auto name         = points.getIdString();
    const auto &bytes       = points.getPositions();
    const size_t nPositions = bytes.size() / sizeof(float);
    if (nPositions == 0) {
      std::cerr << "0@ " << name << std::flush;
      //  world.remove(name + "_instance");
      return http::make_ready_response(http::Code::OK);
    }

    const float *inPositions = (float *)(bytes.data());
    const auto &rgb          = points.getColors();

    std::vector<float> positions;
    std::vector<float> colors;
    positions.resize(nPositions);
    colors.resize(nPositions);

    for (size_t i = 0; i < nPositions; i += 3) {
      positions[i + 0] = inPositions[i + 0] + origin.get0() - _global[0];
      positions[i + 1] = inPositions[i + 1] + origin.get1() - _global[1];
      positions[i + 2] = inPositions[i + 2] + origin.get2() - _global[2];

      colors[i + 0] = float(rgb[i + 0]) / 255.f;
      colors[i + 1] = float(rgb[i + 1]) / 255.f;
      colors[i + 2] = float(rgb[i + 2]) / 255.f;
    }

    // create and setup spheres
    ospray::cpp::Geometry spheres("spheres");
    ospray::cpp::Data data =
        ospray::cpp::Data(nPositions / 3, OSP_FLOAT3, positions.data());
    data.commit();
    spheres.set("radius", 1.f);
    spheres.set("bytes_per_sphere", 12);
    spheres.set("spheres", data);
    spheres.commit();

    // data = ospray::cpp::Data(nPositions / 3, OSP_FLOAT3, colors.data());
    // data.commit();
    // spheres.set("color", data);
    // spheres.commit();

    _world.addGeometry(spheres);
    _world.commit();

    std::cout << nPositions / 3 << "@" << name << std::flush;
    return http::make_ready_response(http::Code::OK);
  }

  std::future<http::Response> _handleCamera(const http::Request &request)
  {
    ospray::data::Camera camera;
    camera.fromJSON(request.body);

    const auto pos    = camera.getPosition();
    const auto lookat = camera.getLookat();
    const auto up     = camera.getUp();

    ospcommon::vec3f cam_pos{0.f, 0.f, 0.f};
    _camera.set(
        "pos",
        ospcommon::vec3f(
            pos[0] - _global[0], pos[1] - _global[1], pos[2] - _global[2]));
    _camera.set(
        "dir",
        ospcommon::vec3f(
            lookat[0] - pos[0], lookat[1] - pos[1], lookat[2] - pos[2]));
    _camera.set("up", ospcommon::vec3f(up[0], up[1], up[2]));
    _camera.commit();
    _world.commit();

    return http::make_ready_response(http::Code::OK);
  }

  http::Server _server;

  ospray::cpp::Renderer _renderer{"scivis"};
  ospray::cpp::Model _world;
  ospray::cpp::Camera _camera{"perspective"};
  ospray::cpp::FrameBuffer _framebuffer{
      ospcommon::vec2i{1024, 768},
      OSP_FB_SRGBA,
      OSP_FB_COLOR | /*OSP_FB_DEPTH |*/ OSP_FB_ACCUM};
#ifdef JPEG
  void *_encoder{tjInitDecompress()};
  std::vector<char> _jpeg;
#endif

  double _global[3] = {0};
};

int main(int argc, const char **argv)
{
  // initialize OSPRay; OSPRay parses (and removes) its commandline
  // parameters, e.g. "--osp:debug"
  OSPError init_error = ospInit(&argc, argv);
  if (init_error != OSP_NO_ERROR)
    return init_error;

  Server server;
  server.run();

  // final cleanups
  renderer.release();
  camera.release();
  lights.release();
  light.release();
  framebuffer.release();
  world.release();

  ospShutdown();

  return 0;
}
