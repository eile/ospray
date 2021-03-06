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
#include <zeroeq/http/helpers.h>
#include <zeroeq/http/server.h>
#include <zeroeq/uri.h>
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
// #define PATHTRACER
#define VISIBILITY

#ifdef VISIBILITY
#ifndef PATHTRACER
#define PATHTRACER
#endif
#endif

#ifdef PATHTRACER
const static std::string rendererType = "pathtracer";
#else
const static std::string rendererType = "scivis";
#endif

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
  std::cout << "Saved " << fileName << std::endl;
}

class Server
{
 public:
  Server()
  {
    _setupOSP();
    _setupZeroEQ();
  }

  void run()
  {
    std::cout << "Commands on " << _server.getURI() << std::endl;
    _dataThread = std::thread{&Server::_runDataServer, this};
    while (true) {
      while (_server.receive(10))
        /*nop, drain*/;

      if (_passes < 65536 && _isIdle())
        _render();
      else
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

 private:
  void _runDataServer()
  {
    std::cout << "Data on " << _data.getURI() << std::endl;
    while (true) {
      _collectTasks();
      if (!_data.receive(0))
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void _setupOSP()
  {
    // create and setup camera
    _camera.set("aspect", _size.x / (float)_size.y);
    _camera.set("pos", ospcommon::vec3f{0.f, 0.f, 0.f});
    _camera.set("dir", ospcommon::vec3f{0.1f, 0.f, 1.f});
    _camera.set("up", ospcommon::vec3f{0.f, 1.f, 0.f});

#ifndef VISIBILITY
    // create and setup light for Ambient Occlusion
    ospray::cpp::Light lights[] = {{"ambient"}, _headlight, {"distant"}};
    _camera.set("apertureRadius", .15f);

    lights[0].set("intensity", .8f);
    lights[0].set("color", ospcommon::vec3f{1.f, 1.f, 1.f});
    lights[0].commit();

    lights[1].set("intensity", 1.f);
    lights[1].set("color", ospcommon::vec3f{1.f, 1.f, 1.f});
    lights[1].set("angularDiameter", 5);
    lights[1].set("direction", ospcommon::vec3f{.7f, -.7f, 0.014f});
    lights[1].set("position", ospcommon::vec3f{0, 0, 0});
    lights[1].commit();

    lights[2].set("intensity", 1.f);
    lights[2].set("angularDiameter", 0.53);
#ifdef PATHTRACER
    lights[2].set("color", ospcommon::vec3f{3.f, 3.f, 3.f});
#else
    lights[2].set("color", ospcommon::vec3f{2.f, 2.f, 2.f});
#endif
    lights[2].set("direction", ospcommon::vec3f{-1.f, 0.f, 0.f});
    lights[2].commit();

    _lights = {lights[0].handle(), lights[1].handle(), lights[2].handle()};
    ospray::cpp::Data lightData(3, OSP_LIGHT, _lights.data());

    lightData.commit();
    _renderer.set("lights", lightData);
#endif

// create and setup material
#ifdef PATHTRACER
    _material.set("Kd", ospcommon::vec3f{0.f, 0.f, 0.f});
    _material.set("Ks", ospcommon::vec3f{1.f, 1.f, 1.f});
    _material.set("Ns", 100.f);
    _renderer.set("maxDepth", 1);
#else
    _material.set("Kd", ospcommon::vec3f{1.f, 1.f, 1.f});
#endif
    _material.commit();
    _camera.commit();  // commit each object to indicate modifications are done

    _emissive.set("color", ospcommon::vec3f{1000.f, 10.f, 0.f});
    _emissive.set("intensity", 1.f);
    _emissive.commit();

    // complete setup of renderer
    _renderer.set("aoSamples", 1);
    _renderer.set("aoTransparencyEnabled", true);
    _renderer.set("shadowsEnabled", true);
    _renderer.set("bgColor", 1.0f);  // white, transparent
    _renderer.set("model", _world);
    _renderer.set("camera", _camera);
    _renderer.set("epsilon", 3 * std::numeric_limits<float>::epsilon());
    _renderer.commit();
  }

  void _setupZeroEQ()
  {
    _data.handle(
        http::Method::POST, "points", [this](const http::Request &request) {
          return _handlePoints(request);
        });
    _data.handle(
        http::Method::POST, "mesh", [this](const http::Request &request) {
          return _handleMesh(request);
        });

    _server.handle(
        http::Method::POST, "reset", [this](const http::Request &request) {
          return _handleReset(request);
        });
    _server.handle(
        http::Method::POST, "camera", [this](const http::Request &request) {
          return _handleCamera(request);
        });
    _server.handle(
        http::Method::POST, "highlight", [this](const http::Request &request) {
          return _handleHighlight(request);
        });
    _server.handle(
        http::Method::GET, "frame", [this](const http::Request &request) {
          return _frame(request);
        });
  }

  bool _isIdle() const
  {
    std::chrono::duration<float> secondsIdle =
        std::chrono::high_resolution_clock::now() - _lastUpdate;
    return secondsIdle.count() > 1.f;
  }

  std::future<http::Response> _frame(const http::Request &request)
  {
    const bool needsRedraw =
        !_geometries.empty() && (_passes == 0 || _lastPass == _passes);
    if (needsRedraw)
      _render();
    while ((std::chrono::high_resolution_clock::now() - _lastFrame).count() <
           2.f)
      _render();

    _lastPass  = _passes;
    _lastFrame = std::chrono::high_resolution_clock::now();

    const auto frameBuffer = _framebuffer;  // copy fb to encode in parallel
    return std::async(std::launch::async,
                      [this, frameBuffer] { return _encode(frameBuffer); });
  }

  http::Response _encode(const ospray::cpp::FrameBuffer &framebuffer)
  {
    uint32_t *fb = (uint32_t *)framebuffer.map(OSP_FB_COLOR);

    http::Response response;
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
                   100,
                   TJ_BOTTOMUP) != 0) {
      std::cerr << tjGetErrorStr() << std::endl;
      response.code = http::Code::INTERNAL_SERVER_ERROR;
      response.headers[http::Header::CONTENT_TYPE] = "text/plain";
      response.body                                = tjGetErrorStr();
    } else {
      response.headers[http::Header::CONTENT_TYPE] = "image/jpeg";
      response.body.resize(size);
    }

    framebuffer.unmap(fb);
    std::cout << " ^ " << std::flush;
    return response;
  }

  void _render()
  {
    _commit();
    {
      std::lock_guard<std::mutex> lock(_mutex);
      if (_geometries.empty())
        return;
    }

    if (_dirty) {
      _framebuffer.clear(OSP_FB_COLOR | OSP_FB_ACCUM);
      _passes = 0;
      _dirty  = false;
    }

    _renderer.renderFrame(_framebuffer, OSP_FB_COLOR | OSP_FB_ACCUM);
    const std::chrono::duration<float> seconds =
        std::chrono::high_resolution_clock::now() - _lastRender;
    ++_passes;
    _lastRender = std::chrono::high_resolution_clock::now();

    std::cout << "\r" << _geometries.size() << " nodes, pass " << _passes
              << ", " << int(seconds.count() * 1000.f) << "ms\t\t"
              << std::flush;
  }

  void _commit()
  {
    {
      std::lock_guard<std::mutex> lock(_mutex);
      while (!_operations.empty()) {
        auto task = _operations.front();
        _operations.pop_front();

        switch (task.first) {
        case OpType::remove:
          _world.removeGeometry(task.second);
          break;

        case OpType::add:
          _world.addGeometry(task.second);
          break;
        }
        _dirty = true;
      }
    }

    if (_dirty) {
#ifndef VISIBILITY
      _lights[1] = _headlight.handle();
      ospray::cpp::Data lights(3, OSP_LIGHT, _lights.data());
      lights.commit();
      _renderer.set("lights", lights);
#endif
      _renderer.commit();
      _world.commit();
    }
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
      _removeGeometry(name);
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
    const bool hasColors     = rgb.size() == nPositions;

    std::vector<float> positions(nPositions);
    std::vector<float> colors(nPositions);

    for (size_t i = 0; i < nPositions; i += 3) {
      positions[i + 0] = inPositions[i + 0] + origin.get0() - _global[0];
      positions[i + 1] = inPositions[i + 1] + origin.get1() - _global[1];
      positions[i + 2] = inPositions[i + 2] + origin.get2() - _global[2];

      if (hasColors) {
        colors[i + 0] = float(rgb[i + 0]) / 255.f;
        colors[i + 1] = float(rgb[i + 1]) / 255.f;
        colors[i + 2] = float(rgb[i + 2]) / 255.f;
      } else {
        colors[i + 0] = 0.5f;
        colors[i + 1] = 0.5f;
        colors[i + 2] = 0.5f;
      }
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

    _addGeometry(name, spheres, false);
  }

  std::future<http::Response> _handleMesh(const http::Request &request)
  {
    if (request.body.empty()) {
      std::cerr << "e" << std::flush;
      return http::make_ready_response(http::Code::BAD_REQUEST);
    }

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
      _removeGeometry(name);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(_mutex);
      if (_deleted.erase(name) > 0)
        return;

      if (_geometries.count(name) > 0 && !mesh.getUpdate())
        return;
    }

    const auto trafo = mesh.getTrafo();
    _updateGlobal(trafo[12], trafo[13], trafo[14]);

    const auto base         = bytes.data();
    const size_t nPositions = bytes.size() / mesh.getStride();
    const auto pos          = base + mesh.getPositionOffset();

    const bool hasColors     = mesh.getColorOffset() > 0;
    const bool hasAlpha      = mesh.getColorSize() > 3;
    const bool isTransparent = mesh.getOpacity() < 1.f;
    const bool hasUV         = mesh.getUvOffset() > 0;
    const auto col           = base + mesh.getColorOffset();
    const auto uv            = base + mesh.getUvOffset();
    bool nonWhite            = false;

    std::vector<float> positions(nPositions * 4);
    std::vector<float> colors(hasColors || isTransparent ? nPositions * 4 : 0,
                              1.f);
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
        colors[outdex + 3] =
            (hasAlpha ? float(color[3]) / 255.f : 1.f) * mesh.getOpacity();
        if (color[0] < 255.f || color[0] < 255.f || color[0] < 255.f)
          nonWhite = true;
      } else if (isTransparent)
        colors[outdex + 3] = mesh.getOpacity();

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
    const int indexSize    = mesh.getIndexSize();
    const size_t nIndices  = inIndices.size() / indexSize;
    const uint16_t *shorts = (const uint16_t *)inIndices.data();
    const uint32_t *ints   = (const uint32_t *)inIndices.data();

    std::vector<int32_t> indices;
    indices.resize(nIndices ? nIndices : nPositions);

    if (nIndices == 0)
      for (size_t i = 0; i < nPositions; ++i)
        indices[i] = i;
    else
      for (size_t i = 0; i < nIndices; ++i)
        indices[i] = indexSize == 2 ? shorts[i] : ints[i];

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

    if (!colors.empty()) {
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

    // auto matHandle = nonWhite ? _emissive.handle() : _material.handle();
    auto matHandle = _material.handle();
    data           = ospray::cpp::Data(1, OSP_MATERIAL, &matHandle);
    data.commit();
    triangles.set("materialList", data);
    triangles.commit();

    _addGeometry(name, triangles, mesh.getUpdate());
  }

  void _addGeometry(const std::string &name,
                    ospray::cpp::Geometry &geometry,
                    const bool update)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_deleted.erase(name) > 0 || (_geometries.count(name) > 0 && !update))
      return;

    if (_geometries.count(name) > 0 /* && update*/) {
      _operations.push_back({OpType::remove, _geometries[name]});
      std::cout << "*" << std::flush;
    } else
      std::cout << "+" << std::flush;

    _operations.push_back({OpType::add, geometry});
    _geometries[name] = geometry.handle();

    _lastUpdate = std::chrono::high_resolution_clock::now();
  }

  void _removeGeometry(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_geometries.count(name) > 0) {
      std::cout << "-" << std::flush;
      _operations.push_back({OpType::remove, _geometries[name]});
      _geometries.erase(name);
      _lastUpdate = std::chrono::high_resolution_clock::now();
    } else
      _deleted.insert(name);
  }

  void _collectTasks()
  {
    while (_tasks.size() > 64) {
      _tasks.front().get();
      _tasks.pop_front();
    }

    while (!_tasks.empty()) {
      if (_tasks.front().wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
        return;
      }
      _tasks.front().get();
      _tasks.pop_front();
    }
  }

  std::future<http::Response> _handleHighlight(const http::Request &request)
  {
    ospray::data::Highlight highlight;
    highlight.fromJSON(request.body);
    const auto name = highlight.getIdString();

    const auto i = _geometries.find(name);
    if (i == _geometries.end())
      return http::make_ready_response(http::Code::NOT_FOUND);

    const ospray::cpp::Geometry geometry(i->second);
    const bool on = highlight.getEnabled();

    auto matHandle = on ? _emissive.handle() : _material.handle();
    const ospray::cpp::Data data(1, OSP_MATERIAL, &matHandle);
    data.commit();
    geometry.set("materialList", data);
    geometry.commit();
    _dirty = true;

    std::cout << (on ? "H" : "h") << std::flush;
    return http::make_ready_response(http::Code::OK);
  }

  std::future<http::Response> _handleReset(const http::Request &)
  {
    while (!_tasks.empty()) {
      _tasks.front().get();
      _tasks.pop_front();
    }

    std::lock_guard<std::mutex> lock(_mutex);
    _operations.clear();

    for (const auto geo : _geometries)
      _world.removeGeometry(geo.second);

    _geometries.clear();
    _dirty = true;
    return http::make_ready_response(http::Code::OK);
  }

  std::future<http::Response> _handleCamera(const http::Request &request)
  {
    ospray::data::Camera camera;
    camera.fromJSON(request.body);

    const ospcommon::vec2i size(camera.getWidth() * 2, camera.getHeight() * 2);
    if (size != _size) {
      _size        = size;
      _framebuffer = ospray::cpp::FrameBuffer{
          _size, OSP_FB_RGBA8, OSP_FB_COLOR | OSP_FB_ACCUM};
      _passes = 0;
    }

    const auto pos    = camera.getPosition();
    const auto lookat = camera.getLookat();
    const auto up     = camera.getUp();
    const ospcommon::vec3f dir(
        lookat[0] - pos[0], lookat[1] - pos[1], lookat[2] - pos[2]);
    const ospcommon::vec3f position(
        pos[0] - _global[0], pos[1] - _global[1], pos[2] - _global[2]);

    _camera.set("pos", position);
    _camera.set("dir", dir);
    _camera.set("up", ospcommon::vec3f(up[0], up[1], up[2]));

    const float fovX = camera.getFovX() * 57.295779513;
    const float fovY = camera.getFovY() * 57.295779513;
    _camera.set("fovx", fovX);
    _camera.set("fovy", fovY);
    _camera.set("aspect", _size.x / (float)_size.y);
#ifndef VISIBILITY
    const float distance = length(dir);
    _camera.set("focusDistance", distance);
    _camera.set("apertureRadius", std::min(1.f, distance * 0.0001f));

    _headlight.set("direction", dir);
    _headlight.set("position", position);
    _headlight.commit();
#endif
    _camera.commit();

    _framebuffer.clear(OSP_FB_COLOR | OSP_FB_ACCUM);
    _passes = 0;

    std::cout << "o" << std::flush;
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

  http::Server _server{zeroeq::URI(":4242")};
  http::Server _data{zeroeq::URI(":4243")};
  std::thread _dataThread;

  ospcommon::vec2i _size{1024, 576};
  ospray::cpp::Renderer _renderer{rendererType};
  ospray::cpp::Model _world;
  ospray::cpp::Camera _camera{"perspective"};
  ospray::cpp::Light _headlight{"sphere"};
  std::vector<OSPLight> _lights{3};
  ospray::cpp::Material _material{rendererType, "OBJMaterial"};
  ospray::cpp::Material _emissive{rendererType, "Luminous"};
  ospray::cpp::FrameBuffer _framebuffer{
      _size, OSP_FB_RGBA8, OSP_FB_COLOR | OSP_FB_ACCUM};

  void *_encoder{tjInitCompress()};

  std::chrono::time_point<std::chrono::high_resolution_clock> _lastUpdate =
      std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> _lastRender =
      std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> _lastFrame =
      std::chrono::high_resolution_clock::now();
  bool _dirty       = true;
  size_t _passes    = 0;
  size_t _lastPass  = 0;
  double _global[3] = {0};
  std::map<std::string, OSPGeometry> _geometries;

  std::mutex _mutex;
  std::deque<std::future<void>> _tasks;

  enum class OpType
  {
    add,
    remove
  };
  using Operation = std::pair<OpType, ospray::cpp::Geometry>;
  std::deque<Operation> _operations;

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

  ospShutdown();

  return 0;
}
