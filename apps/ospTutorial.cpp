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
#include <deque>
#include <map>
#ifdef _WIN32
#define NOMINMAX
#include <malloc.h>
#else
#include <alloca.h>
#endif

#include <turbojpeg.h>
#include <zeroeq/URI.h>
#include <zeroeq/http/Server.h>
#include <zeroeq/http/helpers.h>
#include <set>
#include "data.h"

namespace http = zeroeq::http;

namespace {
  template <typename T>
  int _sign(T val)
  {
    return (T(0) < val) - (val < T(0));
  }
}  // namespace

//#define NORMALS

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
      _collectTasks();
      while (_server.receive(10))
        /*nop, drain*/;

      std::chrono::duration<float> secondsIdle =
          std::chrono::high_resolution_clock::now() - _lastUpdate;
      if (_passes < 256 && secondsIdle.count() > 1.f)
        _render();
      else
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

 private:
  void _setupOSP()
  {
    // create and setup camera
    _camera.set("aspect", _size.x / (float)_size.y);
    _camera.set("pos", ospcommon::vec3f{0.f, 0.f, 0.f});
    _camera.set("dir", ospcommon::vec3f{0.1f, 0.f, 1.f});
    _camera.set("up", ospcommon::vec3f{0.f, 1.f, 0.f});
    _camera.set("apertureRadius", .2f);
    _camera.commit();  // commit each object to indicate modifications are done

    // create and setup light for Ambient Occlusion
    ospray::cpp::Light lights[] = {_headlight,
                                   _renderer.newLight("distant"),
                                   _renderer.newLight("ambient")};
    lights[0].set("intensity", 1.f);
    lights[0].set("angularDiameter", 5);
    lights[0].set("color", ospcommon::vec3f{1.f, 1.f, 1.f});
    lights[0].set("direction", ospcommon::vec3f{.7f, .7f, 0.014f});
    lights[0].commit();

    lights[1].set("intensity", 1.f);
    lights[1].set("angularDiameter", 0.53);
    lights[1].set("color", ospcommon::vec3f{2.f, 1.9f, 1.9f});
    lights[1].set("direction", ospcommon::vec3f{-.7f, .7f, 0.014f});
    lights[1].commit();

    lights[2].set("intensity", .5f);
    lights[2].set("color", ospcommon::vec3f{.9f, .9f, 1.f});
    lights[2].commit();

    OSPLight lightHandles[] = {
        lights[0].handle(), lights[1].handle(), lights[2].handle()};
    _lights = ospray::cpp::Data(3, OSP_LIGHT, lightHandles);
    _lights.commit();

    // create and setup material
    _material.set("Kd", ospcommon::vec3f{1.f, 1.f, 1.f});
    _material.commit();

    // one dummy sphere to not crash when rendering before data arrives
    std::vector<float> positions = {0, 0, 0};
    ospray::cpp::Geometry spheres("spheres");
    ospray::cpp::Data data = ospray::cpp::Data(1, OSP_FLOAT3, positions.data());
    data.commit();
    spheres.set("radius", 1.f);
    spheres.set("bytes_per_sphere", 12);
    spheres.set("spheres", data);
    spheres.commit();
    _world.addGeometry(spheres);
    _world.commit();

    // complete setup of renderer
    _renderer.set("aoSamples", 1);
    _renderer.set("shadowsEnabled", true);
    _renderer.set("bgColor", 1.0f);  // white, transparent
    _renderer.set("model", _world);
    _renderer.set("camera", _camera);
    _renderer.set("lights", _lights);
    _renderer.commit();
  }

  void _setupZeroEQ()
  {
    _server.handle(
        http::Method::POST, "points", [this](const http::Request &request) {
          return _handlePoints(request);
        });
    _server.handle(
        http::Method::POST, "mesh", [this](const http::Request &request) {
          return _handleMesh(request);
        });
    _server.handle(
        http::Method::POST, "camera", [this](const http::Request &request) {
          return _handleCamera(request);
        });
    _server.handle(
        http::Method::GET, "frame", [this](const http::Request &request) {
          return _frame(request);
        });
    std::cerr << "Bound to " << _server.getURI() << std::endl;
  }

  std::future<http::Response> _frame(const http::Request &request)
  {
    if (_passes == 0 || _lastPass == _passes) {
      _render();
      _lastPass = _passes;
    }

    uint32_t *fb = (uint32_t *)_framebuffer.map(OSP_FB_COLOR);

    // static size_t num = 0;
    // writePPM(
    //     std::string("accumulatedFrameCpp") + std::to_string(num++) + ".ppm",
    //     _size,
    //     fb);
    // std::cout << 'p' << std::flush;
    http::Response response;
    response.headers[http::Header::CONTENT_TYPE] = "image/jpeg";

    response.body.resize(TJBUFSIZE(_size[0], _size[1]));
    unsigned long size = 0;
    if (tjCompress(_encoder,
                   reinterpret_cast<unsigned char *>(fb),
                   _size[0],
                   _size[0] * 4,
                   _size[1],
                   4,
                   (unsigned char *)response.body.data(),
                   &size,
                   TJ_444,
                   90,
                   TJ_BOTTOMUP) != 0) {
      std::cerr << tjGetErrorStr() << std::endl;
      return http::make_ready_response(http::Code::INTERNAL_SERVER_ERROR);
    }

    response.body.resize(size);
    std::cout << 'f' << std::flush;
    _framebuffer.unmap(fb);

    std::promise<http::Response> promise;
    promise.set_value(response);
    return promise.get_future();
  }

  void _render()
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_passes == 0) {
      _world.commit();
      _framebuffer.clear(OSP_FB_COLOR | OSP_FB_ACCUM);
    }

    _renderer.renderFrame(_framebuffer, OSP_FB_COLOR | OSP_FB_ACCUM);
    ++_passes;
    std::cout << 'r' << _passes << std::flush;
  }

  std::future<http::Response> _handlePoints(const http::Request &request)
  {
    auto func = [this, request] { _addPoints(request.body); };
    _tasks.push_back(std::async(std::launch::async, func));
    _collectTasks();
    return http::make_ready_response(http::Code::OK);
  }

  void _addPoints(const std::string &json)
  {
    ospray::data::Points points;
    points.fromJSON(json);
    const auto name   = points.getIdString();
    const auto &bytes = points.getPositions();

    if (bytes.empty()) {
      std::lock_guard<std::mutex> lock(_mutex);
      if (_geometries.count(name) > 0) {
        std::cerr << "d" << std::flush;
        _world.removeGeometry(_geometries[name]);
        _geometries.erase(name);
        _passes     = 0;
        _lastUpdate = std::chrono::high_resolution_clock::now();
      } else
        _deleted.insert(name);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(_mutex);
      if (_deleted.erase(name) > 0 || _geometries.count(name) > 0)
        return;
    }

    const auto &origin = points.getOrigin();
    _updateGlobal(origin.get0(), origin.get1(), origin.get2());

    const size_t nPositions  = bytes.size() / sizeof(float);
    const float *inPositions = (float *)(bytes.data());
    const auto &rgb          = points.getColors();

    std::vector<float> positions(nPositions);
    std::vector<float> colors(nPositions);

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

    ospray::cpp::Data data{nPositions / 3, OSP_FLOAT3, positions.data()};
    data.commit();
    spheres.set("radius", points.getRadius());
    spheres.set("bytes_per_sphere", 12);
    spheres.set("spheres", data);
    spheres.commit();

    auto matHandle = _material.handle();
    data           = ospray::cpp::Data(1, OSP_MATERIAL, &matHandle);
    data.commit();
    spheres.set("materialList", data);
    spheres.commit();

    data = ospray::cpp::Data(nPositions / 3, OSP_FLOAT3, colors.data());
    data.commit();
    spheres.set("color", data);
    spheres.commit();

    std::lock_guard<std::mutex> lock(_mutex);
    _geometries[points.getIdString()] = spheres.handle();
    _world.addGeometry(spheres);

    std::cout << nPositions / 3 << std::flush;
    _passes     = 0;
    _lastUpdate = std::chrono::high_resolution_clock::now();
  }

  std::future<http::Response> _handleMesh(const http::Request &request)
  {
    auto func = [this, request] { _addMesh(request.body); };
    _tasks.push_back(std::async(std::launch::async, func));
    _collectTasks();
    return http::make_ready_response(http::Code::OK);
  }

  void _addMesh(const std::string &json)
  {
    ospray::data::Mesh mesh;
    mesh.fromJSON(json);
    const auto name   = mesh.getIdString();
    const auto &bytes = mesh.getData();

    if (bytes.empty()) {
      std::lock_guard<std::mutex> lock(_mutex);
      if (_geometries.count(name) > 0) {
        std::cerr << "d" << std::flush;
        _world.removeGeometry(_geometries[name]);
        _geometries.erase(name);
        _passes     = 0;
        _lastUpdate = std::chrono::high_resolution_clock::now();
      } else
        _deleted.insert(name);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(_mutex);
      if (_deleted.erase(name) > 0 || _geometries.count(name) > 0)
        return;
    }

    const auto trafo = mesh.getTrafo();
    _updateGlobal(trafo[12], trafo[13], trafo[14]);

    const auto base         = bytes.data();
    const size_t nPositions = bytes.size() / mesh.getStride();
    const auto pos          = base + mesh.getPositionOffset();

    const bool hasColors = mesh.getColorOffset() > 0;
    const bool hasUV     = mesh.getUvOffset() > 0;
    const auto col       = base + mesh.getColorOffset();
    const auto uv        = base + mesh.getUvOffset();

    std::vector<float> positions(nPositions * 4);
    std::vector<float> colors(hasColors ? nPositions * 4 : 0);
    std::vector<float> texcoords(hasUV ? nPositions * 2 : 0);

#ifdef NORMALS
    const bool hasNormals = mesh.getNormalOffset() > 0;
    std::vector<float> normals(nPositions * 4);
    const auto nor = base + mesh.getNormalOffset();
#endif

    for (size_t i = 0; i < nPositions; ++i) {
      const size_t index  = i * mesh.getStride();
      const size_t outdex = 4 * i;

      const float *position = (const float *)(pos + index);
      const double in[]     = {position[0], position[1], position[2], 1.};
      double out[4]         = {0};

      for (size_t r = 0; r < 4; ++r)
        for (size_t c = 0; c < 4; ++c)
          out[r] += trafo[r + 4 * c] * in[c];

      positions[outdex + 0] = out[0] / out[3] - _global[0];
      positions[outdex + 1] = out[1] / out[3] - _global[1];
      positions[outdex + 2] = out[2] / out[3] - _global[2];
      positions[outdex + 3] = 0.f;

      if (hasColors) {
        const uint8_t *color = (const uint8_t *)(col + index);
        colors[outdex + 0]   = float(color[0]) / 255.f;
        colors[outdex + 1]   = float(color[1]) / 255.f;
        colors[outdex + 2]   = float(color[2]) / 255.f;
        colors[outdex + 3]   = 1.f;
      }
      if (hasUV) {
        const float *texcoord = (const float *)(uv + index);
        texcoords[2 * i + 0]  = texcoord[0];
        texcoords[2 * i + 1]  = texcoord[1];
      }
#ifdef NORMALS
      if (hasNormals) {
        const int16_t *normal = (const int16_t *)(nor + index);
        float x               = float(normal[0]) / 32767.f;
        float y               = float(normal[1]) / 32767.f;
        const float z         = 1.f - std::abs(x) - std::abs(y);
        x += _sign(x) * std::min(z, 0.f);
        y += _sign(y) * std::min(z, 0.f);
        const float len = std::sqrt(x * x + y * y + z * z);

        normals[3 * i + 0] = x / len;
        normals[3 * i + 1] = y / len;
        normals[3 * i + 2] = z / len;
        normals[3 * i + 3] = 0.f;
      }
#endif
    }

    const auto &inIndices  = mesh.getIndices();
    const size_t nIndices  = inIndices.size() / 2;
    const uint16_t *shorts = (const uint16_t *)inIndices.data();

    std::vector<int32_t> indices;
    indices.resize(nIndices ? nIndices : nPositions);

    if (nIndices == 0)
      for (size_t i = 0; i < nPositions; ++i)
        indices[i] = i;
    else
      for (size_t i = 0; i < nIndices; ++i)
        indices[i] = shorts[i];

    // create and setup mesh
    ospray::cpp::Geometry triangles("triangles");
    ospray::cpp::Data data =
        ospray::cpp::Data(nPositions, OSP_FLOAT3A, positions.data());
    data.commit();
    triangles.set("vertex", data);
    triangles.commit();

    data = ospray::cpp::Data(indices.size() / 3, OSP_INT3, indices.data());
    data.commit();
    triangles.set("index", data);
    triangles.commit();

    if (hasColors) {
      data = ospray::cpp::Data(nPositions, OSP_FLOAT4, colors.data());
      data.commit();
      triangles.set("vertex.color", data);
      triangles.commit();
    }

#ifdef NORMALS
    if (hasNormals) {
      data = ospray::cpp::Data(nPositions, OSP_FLOAT3A, normals.data());
      data.commit();
      triangles.set("vertex.normal", data);
      triangles.commit();
    }
#endif

    auto matHandle = _material.handle();
    data           = ospray::cpp::Data(1, OSP_MATERIAL, &matHandle);
    data.commit();
    triangles.set("materialList", data);
    triangles.commit();

    std::lock_guard<std::mutex> lock(_mutex);
    _geometries[name] = triangles.handle();
    _world.addGeometry(triangles);
    _world.commit();
    std::cout << nPositions << std::flush;
    _passes     = 0;
    _lastUpdate = std::chrono::high_resolution_clock::now();
  }

  void _collectTasks()
  {
    while (_tasks.size() > 32) {
      _tasks.front().get();
      _tasks.pop_front();
    }
    while (!_tasks.empty()) {
      if (_tasks.front().wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready)
        return;
      _tasks.front().get();
      _tasks.pop_front();
    }
  }

  std::future<http::Response> _handleCamera(const http::Request &request)
  {
    ospray::data::Camera camera;
    camera.fromJSON(request.body);

    _size = ospcommon::vec2i(camera.getWidth() / 2, camera.getHeight() / 2);

    const auto pos    = camera.getPosition();
    const auto lookat = camera.getLookat();
    const auto up     = camera.getUp();
    const ospcommon::vec3f dir(
        lookat[0] - pos[0], lookat[1] - pos[1], lookat[2] - pos[2]);

    _camera.set(
        "pos",
        ospcommon::vec3f(
            pos[0] - _global[0], pos[1] - _global[1], pos[2] - _global[2]));
    _camera.set("dir", dir);
    _camera.set("up", ospcommon::vec3f(up[0], up[1], up[2]));

    _camera.set("fovx", camera.getFovX() * 57.295779513);
    _camera.set("fovy", camera.getFovY() * 57.295779513);
    _camera.set("aspect", _size.x / (float)_size.y);
    _camera.set("focusDistance", length(dir));
    _camera.commit();

    _headlight.set("direction", dir);
    _headlight.commit();
    _lights.commit();

    _framebuffer = ospray::cpp::FrameBuffer{
        _size, OSP_FB_RGBA8, OSP_FB_COLOR | OSP_FB_ACCUM};
    _passes = 0;
    std::cout << "c" << std::flush;
    return http::make_ready_response(http::Code::OK);
  }

  void _updateGlobal(const double x, const double y, const double z)
  {
    if (_global[0] != 0 || _global[1] != 0 || _global[2] != 0)
      return;

    _global[0] = x;
    _global[1] = y;
    _global[2] = z;
  }

  http::Server _server;

  ospcommon::vec2i _size{1024, 576};
  ospray::cpp::Renderer _renderer{"scivis"};
  ospray::cpp::Model _world;
  ospray::cpp::Camera _camera{"perspective"};
  ospray::cpp::Light _headlight{_renderer.newLight("distant")};
  ospray::cpp::Data _lights{3, OSP_LIGHT};
  ospray::cpp::Material _material{_renderer.newMaterial("OBJMaterial")};
  ospray::cpp::FrameBuffer _framebuffer{
      _size, OSP_FB_RGBA8, OSP_FB_COLOR | OSP_FB_ACCUM};

  void *_encoder{tjInitCompress()};

  std::chrono::time_point<std::chrono::high_resolution_clock> _lastUpdate =
      std::chrono::high_resolution_clock::now();
  size_t _passes    = 0;
  size_t _lastPass  = 0;
  double _global[3] = {0};
  std::map<std::string, OSPGeometry> _geometries;

  std::mutex _mutex;
  std::deque<std::future<void>> _tasks;
  std::set<std::string> _deleted;
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
