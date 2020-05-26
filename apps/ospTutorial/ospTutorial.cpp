// Copyright 2009-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

/* This is a small example tutorial how to use OSPRay in an application.
 *
 * On Linux build it in the build_directory with
 *   g++ ../apps/ospTutorial/ospTutorial.cpp -I ../ospray/include \
 *       -I ../../ospcommon -L . -lospray -Wl,-rpath,. -o ospTutorial
 * On Windows build it in the build_directory\$Configuration with
 *   cl ..\..\apps\ospTutorial\ospTutorial.cpp /EHsc -I ..\..\ospray\include ^
 *      -I ..\.. -I ..\..\..\ospcommon ospray.lib
 */

#include "connection.h"

#include "ospray/ospray_cpp.h"

#include <boost/thread/thread.hpp>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <deque>
#include <vector>

#ifdef _WIN32
#define NOMINMAX
#include <malloc.h>
#else
#include <alloca.h>
#endif

// #define NORMALS
// #define PATHTRACER
// #define VISIBILITY

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

using namespace ospcommon::math;

typedef std::shared_ptr<boost::asio::io_service> io_service_ptr;

typedef std::deque<io_service_ptr> io_services;

// helper function to write the rendered image as PPM file
void writePPM(const char *fileName, const vec2i &size, const uint32_t *pixel)
{
  FILE *file = fopen(fileName, "wb");
  if (file == nullptr) {
    fprintf(stderr, "fopen('%s', 'wb') failed: %d", fileName, errno);
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

namespace {
using HandleFunc = std::function<void(const Connection &connection)>;
using HandlerMap = std::unordered_map<std::string, HandleFunc>;

class MyConnection : public Connection
{
 public:
  MyConnection(boost::asio::io_service &ioService, const HandlerMap &handlers)
      : Connection(ioService), _handlers(handlers)
  {}

 protected:
  void onData() final
  {
    const auto &i = _handlers.find(url());
    if (i == _handlers.end())
      std::cout << "Missing handler for " << url() << std::endl;
    else
      i->second(*this);
  }

  const HandlerMap &_handlers;
};

class Server
{
 public:
  Server(const io_services &ioServices,
      const int port,
      const std::string &interface)
      : _ioServices(ioServices),
        _endpoint(interface.empty()
                ? (boost::asio::ip::tcp::endpoint(
                    boost::asio::ip::tcp::v4(), port))
                : boost::asio::ip::tcp::endpoint(
                    boost::asio::ip::address().from_string(interface), port)),
        _acceptor(*_ioServices.front(), _endpoint)
  {
    _setupOSP();
    _startAccept();
  }

  void registerHandler(const std::string &url, const HandleFunc &handler)
  {
    _httpHandlers[url] = handler;
  }

 private:
  void _startAccept()
  {
    _ioServices.push_back(_ioServices.front());
    _ioServices.pop_front();
    const auto connection =
        new MyConnection(*_ioServices.front(), _httpHandlers);

    _acceptor.async_accept(connection->socket(),
        [this, connection](const boost::system::error_code &error) {
          _accept(connection, error);
        });
  }

  void _accept(Connection *connection, const boost::system::error_code &error)
  {
    {
      if (!error) {
        connection->read();
        _startAccept();
      } else
        delete connection;
    }
  }

  void _setupOSP()
  {
    // create and setup camera
    _camera.setParam("aspect", _size.x / (float)_size.y);
    _camera.setParam("position", vec3f{0.f, 0.f, 0.f});
    _camera.setParam("direction", vec3f{0.1f, 0.f, 1.f});
    _camera.setParam("up", vec3f{0.f, 1.f, 0.f});

#ifndef VISIBILITY
    // create and setup light for Ambient Occlusion
    ospray::cpp::Light lights[] = {{"ambient"}, _headlight, {"distant"}};
    _camera.setParam("apertureRadius", .15f);

    lights[0].setParam("intensity", .8f);
    lights[0].setParam("color", vec3f{1.f, 1.f, 1.f});
    lights[0].commit();

    lights[1].setParam("intensity", 1.f);
    lights[1].setParam("color", vec3f{1.f, 1.f, 1.f});
    lights[1].setParam("angularDiameter", 5);
    lights[1].setParam("direction", vec3f{.7f, -.7f, 0.014f});
    lights[1].setParam("position", vec3f{0, 0, 0});
    lights[1].commit();

    lights[2].setParam("intensity", 1.f);
    lights[2].setParam("angularDiameter", 0.53);
#ifdef PATHTRACER
    lights[2].setParam("color", vec3f{3.f, 3.f, 3.f});
#else
    lights[2].setParam("color", vec3f{2.f, 2.f, 2.f});
#endif
    lights[2].setParam("direction", vec3f{-1.f, 0.f, 0.f});
    lights[2].commit();

    _lights = {lights[0].handle(), lights[1].handle(), lights[2].handle()};
    ospray::cpp::Data lightData(1, OSP_LIGHT, _lights.data());
    lightData.commit();
    _world.setParam("light", lightData);
#endif

// create and setup material
#ifdef PATHTRACER
    _material.setParam("Kd", vec3f{0.f, 0.f, 0.f});
    _material.setParam("Ks", vec3f{1.f, 1.f, 1.f});
    _material.setParam("Ns", 100.f);
    _renderer.setParam("maxDepth", 1);
#else
    _material.setParam("Kd", vec3f{1.f, 1.f, 1.f});
#endif
    _material.commit();
    _camera.commit(); // commit each object to indicate modifications are done

    _emissive.setParam("color", vec3f{1000.f, 10.f, 0.f});
    _emissive.setParam("intensity", 1.f);
    _emissive.commit();

    { // remove block
      std::vector<vec3f> vertex = {vec3f(-1.0f, -1.0f, 3.0f),
          vec3f(-1.0f, 1.0f, 3.0f),
          vec3f(1.0f, -1.0f, 3.0f),
          vec3f(0.1f, 0.1f, 0.3f)};

      std::vector<vec4f> color = {vec4f(0.9f, 0.5f, 0.5f, 1.0f),
          vec4f(0.8f, 0.8f, 0.8f, 1.0f),
          vec4f(0.8f, 0.8f, 0.8f, 1.0f),
          vec4f(0.5f, 0.9f, 0.5f, 1.0f)};

      std::vector<vec3ui> index = {vec3ui(0, 1, 2), vec3ui(1, 2, 3)};

      ospray::cpp::Geometry mesh("mesh");
      mesh.setParam("vertex.position", ospray::cpp::Data(vertex));
      mesh.setParam("vertex.color", ospray::cpp::Data(color));
      mesh.setParam("index", ospray::cpp::Data(index));
      mesh.commit();

      // put the mesh into a model
      ospray::cpp::GeometricModel model(mesh);
      model.commit();

      // put the model into a group (collection of models)
      _group.setParam("geometry", ospray::cpp::Data(model));
    }

    _group.commit();
    _instance.commit();
    _world.setParam("instance", ospray::cpp::Data(_instance));
    _world.commit();

    // complete setup of renderer
    _renderer.setParam("aoSamples", 1);
    _renderer.setParam("aoTransparencyEnabled", true);
    _renderer.setParam("shadowsEnabled", true);
    _renderer.setParam("backgroundColor", 1.0f); // white, transparent
    _renderer.setParam("epsilon", 3 * std::numeric_limits<float>::epsilon());
    _renderer.commit();

    _framebuffer.clear();
  }

  io_services _ioServices;
  const boost::asio::ip::tcp::endpoint _endpoint;
  boost::asio::ip::tcp::acceptor _acceptor;

  vec2i _size{1024, 576};
  ospray::cpp::Camera _camera{"perspective"};
  ospray::cpp::Light _headlight{"sphere"};
  std::vector<OSPLight> _lights{3};
  ospray::cpp::Material _material{rendererType, "OBJMaterial"};
  ospray::cpp::Material _emissive{rendererType, "Luminous"};

  ospray::cpp::Group _group;
  ospray::cpp::Instance _instance{_group};
  ospray::cpp::World _world;

  ospray::cpp::Renderer _renderer{rendererType};

  ospray::cpp::FrameBuffer _framebuffer{
      _size, OSP_FB_SRGBA, OSP_FB_COLOR | OSP_FB_ACCUM};

  HandlerMap _httpHandlers;
};
} // namespace

int main(int argc, const char **argv)
{
  // initialize OSPRay; OSPRay parses (and removes) its commandline parameters,
  // e.g. "--osp:debug"
  OSPError init_error = ospInit(&argc, argv);
  if (init_error != OSP_NO_ERROR)
    return init_error;

  try {
    {
      boost::asio::io_service ioService;
      auto connection = new MyConnection(ioService, HandlerMap());
      connection->url() = "/~eile/";
      connection->write("localhost", 80);
      ioService.run();
    }

    const size_t numThreads = 8;
    const int port = 4242;
    const std::string interface = "127.0.0.1";
    io_services ioServices;
    std::deque<boost::asio::io_service::work> work;

    boost::thread_group threads;
    for (size_t i = 0; i < numThreads; ++i) {
      io_service_ptr ioService(new boost::asio::io_service);
      ioServices.push_back(ioService);
      work.push_back(boost::asio::io_service::work(*ioService));
      threads.create_thread([ioService]() { ioService->run(); });
    }

    Server server(ioServices, port, interface);
    threads.join_all();
  } catch (std::exception &e) {
    std::cerr << "Error running irts server: " << e.what() << std::endl;
  }

  ospShutdown();
  return 0;
}
