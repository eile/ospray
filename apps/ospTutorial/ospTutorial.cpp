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
class MyConnection : public Connection
{
 public:
  MyConnection(boost::asio::io_service &ioService) : Connection(ioService) {}

 protected:
  void onRequest() final
  {
    std::cout << url() << ": " << body() << std::endl;
    // request handling goes here
  }
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
    _startAccept();
  }

 private:
  void _startAccept()
  {
    _ioServices.push_back(_ioServices.front());
    _ioServices.pop_front();
    const auto connection = new MyConnection(*_ioServices.front());

    _acceptor.async_accept(connection->socket(),
        [this, connection](const boost::system::error_code &error) {
          _accept(connection, error);
        });
  }

  void _accept(Connection *connection, const boost::system::error_code &error)
  {
    {
      if (!error) {
        connection->readRequest();
        _startAccept();
      } else
        delete connection;
    }
  }

  io_services _ioServices;
  const boost::asio::ip::tcp::endpoint _endpoint;
  boost::asio::ip::tcp::acceptor _acceptor;
};
} // namespace

int main(int argc, const char **argv)
{
  // image size
  vec2i imgSize;
  imgSize.x = 1024; // width
  imgSize.y = 768; // height

  // camera
  vec3f cam_pos{0.f, 0.f, 0.f};
  vec3f cam_up{0.f, 1.f, 0.f};
  vec3f cam_view{0.1f, 0.f, 1.f};

  // triangle mesh data
  std::vector<vec3f> vertex = {vec3f(-1.0f, -1.0f, 3.0f),
      vec3f(-1.0f, 1.0f, 3.0f),
      vec3f(1.0f, -1.0f, 3.0f),
      vec3f(0.1f, 0.1f, 0.3f)};

  std::vector<vec4f> color = {vec4f(0.9f, 0.5f, 0.5f, 1.0f),
      vec4f(0.8f, 0.8f, 0.8f, 1.0f),
      vec4f(0.8f, 0.8f, 0.8f, 1.0f),
      vec4f(0.5f, 0.9f, 0.5f, 1.0f)};

  std::vector<vec3ui> index = {vec3ui(0, 1, 2), vec3ui(1, 2, 3)};

  // initialize OSPRay; OSPRay parses (and removes) its commandline parameters,
  // e.g. "--osp:debug"
  OSPError init_error = ospInit(&argc, argv);
  if (init_error != OSP_NO_ERROR)
    return init_error;

  // use scoped lifetimes of wrappers to release everything before ospShutdown()
  {
    // create and setup camera
    ospray::cpp::Camera camera("perspective");
    camera.setParam("aspect", imgSize.x / (float)imgSize.y);
    camera.setParam("position", cam_pos);
    camera.setParam("direction", cam_view);
    camera.setParam("up", cam_up);
    camera.commit(); // commit each object to indicate modifications are done

    // create and setup model and mesh
    ospray::cpp::Geometry mesh("mesh");
    mesh.setParam("vertex.position", ospray::cpp::Data(vertex));
    mesh.setParam("vertex.color", ospray::cpp::Data(color));
    mesh.setParam("index", ospray::cpp::Data(index));
    mesh.commit();

    // put the mesh into a model
    ospray::cpp::GeometricModel model(mesh);
    model.commit();

    // put the model into a group (collection of models)
    ospray::cpp::Group group;
    group.setParam("geometry", ospray::cpp::Data(model));
    group.commit();

    // put the group into an instance (give the group a world transform)
    ospray::cpp::Instance instance(group);
    instance.commit();

    // put the instance in the world
    ospray::cpp::World world;
    world.setParam("instance", ospray::cpp::Data(instance));

    // create and setup light for Ambient Occlusion
    ospray::cpp::Light light("ambient");
    light.commit();

    world.setParam("light", ospray::cpp::Data(light));
    world.commit();

    // create renderer, choose Scientific Visualization renderer
    ospray::cpp::Renderer renderer("scivis");

    // complete setup of renderer
    renderer.setParam("aoSamples", 1);
    renderer.setParam("backgroundColor", 1.0f); // white, transparent
    renderer.commit();

    // create and setup framebuffer
    ospray::cpp::FrameBuffer framebuffer(
        imgSize, OSP_FB_SRGBA, OSP_FB_COLOR | OSP_FB_ACCUM);
    framebuffer.clear();

    // render one frame
    framebuffer.renderFrame(renderer, camera, world);

    // access framebuffer and write its content as PPM file
    uint32_t *fb = (uint32_t *)framebuffer.map(OSP_FB_COLOR);
    writePPM("firstFrameCpp.ppm", imgSize, fb);
    framebuffer.unmap(fb);

    try {
      const size_t numThreads = 2;
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

    for (int frames = 0; frames < 10; frames++)
      framebuffer.renderFrame(renderer, camera, world);

    fb = (uint32_t *)framebuffer.map(OSP_FB_COLOR);
    writePPM("accumulatedFrameCpp.ppm", imgSize, fb);
    framebuffer.unmap(fb);

    ospray::cpp::PickResult res =
        framebuffer.pick(renderer, camera, world, vec2f(0.5f));

    if (res.hasHit) {
      std::cout << "Picked geometry [inst: " << res.instance.handle()
                << ", model: " << res.model.handle() << ", prim: " << res.primID
                << "]" << std::endl;
    }
  }

  ospShutdown();

  return 0;
}
