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
#include "proto/FeatureCollection.pb.h"
#include "webscene.h"

#include <turbojpeg.h>
#include "jsoncpp/json/json.h"
#include "ospray/ospray_cpp.h"

#include <boost/thread/thread.hpp>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <deque>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
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

using io_service_ptr = std::shared_ptr<boost::asio::io_service>;
using io_services = std::deque<io_service_ptr>;
using high_resolution_clock = std::chrono::high_resolution_clock;

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
namespace convert {
// radian conversion for float types only:
template <class T>
inline T radians(T v) noexcept;
template <>
inline double radians(double v) noexcept
{
  return v * 0.01745329251994329576923690768489;
}
template <>
inline float radians(float v) noexcept
{
  return v * 0.01745329251994329576923690768489f;
}

template <class T>
inline T degrees(T v) noexcept;
template <>
inline double degrees(double v) noexcept
{
  return v * 57.295779513082320876798154814105;
}
template <>
inline float degrees(float v) noexcept
{
  return v * 57.295779513082320876798154814105f;
}

const double rad2deg1 = degrees(1.0);
const double deg2rad1 = radians(1.0);
constexpr double earthRadius = 6378137.0;
constexpr double halfEarthRadius = earthRadius * 0.5;
constexpr double PI = 3.14159265358979323846;
constexpr double halfPI = 0.5 * PI;
constexpr double almostHalfPI = 0.4999999 * PI;

void webMercatorToWGS84(vec3d &point)
{
  // code adapted from esri/geometry/webMercatorUtils
  point.x = rad2deg1 * (point.x / earthRadius);
  point.y =
      rad2deg1 * (halfPI - 2 * std::atan(std::exp((-point.y) / earthRadius)));
}

void wgs84ToSphericalECEF(vec3d &point)
{
  const double radius = earthRadius + point.z;
  const double lat = deg2rad1 * point.y;
  const double lon = deg2rad1 * point.x;
  const double cosLat = std::cos(lat);

  point.x = std::cos(lon) * cosLat * radius;
  point.y = std::sin(lon) * cosLat * radius;
  point.z = std::sin(lat) * radius;
}

void webMercatorToSphericalECEF(vec3d &point)
{
  webMercatorToWGS84(point);
  wgs84ToSphericalECEF(point);
}

} // namespace convert
/** @return http status code */
using HandleFunc = std::function<int(Connection &connection)>;
using HandlerMap = std::unordered_map<std::string, HandleFunc>;
const std::string spinner{".oOo. "};

class MyConnection : public Connection
{
 public:
  MyConnection(boost::asio::io_service &ioService, const HandlerMap &handlers)
      : Connection(ioService), _handlers(handlers)
  {}

 protected:
  int onRequest() final
  {
    auto baseURL = url();
    auto index = baseURL.find('?');
    if (index != std::string::npos)
      baseURL = baseURL.substr(0, index);

    const auto &i = _handlers.find(baseURL);
    if (i == _handlers.end()) {
      std::cout << "Missing handler for " << baseURL << " with body: " << body()
                << std::endl;
      return 404;
    }

    try {
      return i->second(*this);
    } catch (std::runtime_error &e) {
      std::cerr << e.what() << std::endl;
      return 500;
    } catch (...) {
      return 500;
    }
  }

  const HandlerMap &_handlers;
};

class Server;
int _handleCamera(Connection &connection, Server &server);
int _handleWebscene(Connection &connection, Server &server);
int _handleFrame(Connection &connection, Server &server);
void _loadPBF(Server &server, const std::string &url);

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
    _setupHandlers();
    _startAccept();
    const std::string defaultDataUrl =
        "http://servicesqa.arcgis.com/SdQnSRS214Ul5Jv5/arcgis/rest/services/FP__4326__US_NewYork__RecyclingBinsPublic/FeatureServer/0/query?f=pbf&cacheHint=true&maxRecordCountFactor=5&resultOffset=0&resultRecordCount=10000&where=1%3D1&outFields=Address%2CBorough%2CLatitude%2CLongitude%2COBJECTID%2CPark_Site_Name%2CSite_type&outSR=102100&spatialRel=esriSpatialRelIntersects";
    _loadPBF(*this, defaultDataUrl);
  }

  /**
   * @return a new connection.
   * Will be created round-robin on the next ioService/thread.
   */
  Connection *newConnection()
  {
    _ioServices.push_back(_ioServices.front());
    _ioServices.pop_front();
    return new MyConnection(*_ioServices.front(), _httpHandlers);
  }

  vec2i size() const
  {
    std::lock_guard<std::mutex> lock(_ospMutex);
    return _size;
  }

  void setSize(const vec2i &size)
  {
    std::lock_guard<std::mutex> lock(_ospMutex);
    if (size == _size)
      return;

    _size = size;
    _framebuffer = ospray::cpp::FrameBuffer{
        _size, OSP_FB_RGBA8, OSP_FB_COLOR | OSP_FB_ACCUM};
    _passes = 0;
  }

  ospray::cpp::Camera camera()
  {
    std::lock_guard<std::mutex> lock(_ospMutex);
    return _camera;
  }

  void setCamera(const ospray::cpp::Camera &camera)
  {
    std::lock_guard<std::mutex> lock(_ospMutex);
    _camera = camera;
    _camera.commit();
    _clear();
    std::cout << "<" << std::flush;
    _spinnerPos = -1;
  }

  void setWebscene(const Webscene &&webscene)
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    _webscene = webscene;
    for (const auto &layer : _webscene.getFeatureLayers()) {
      std::cout << layer << std::endl;
      _loadLayer(layer);
    }
  }

  ospray::cpp::FrameBuffer frame()
  {
    std::lock_guard<std::mutex> lock(_ospMutex);
    _render();
    _spinnerPos = -1;
    return _framebuffer;
  }

  void render()
  {
    std::lock_guard<std::mutex> lock(_ospMutex);
    _render();
  }

  size_t passesRendered()
  {
    return _passes;
  }

  bool idle()
  {
    return (high_resolution_clock::now() - _lastData).count() > 1.f;
  }

  vec3d updateOrigin(const double x, const double y, const double z)
  {
    return updateOrigin({x, y, z});
  }
  vec3d updateOrigin(const vec3d &origin)
  {
    std::lock_guard<std::mutex> lock(_ospMutex);
    if (_origin.x == 0 && _origin.y == 0 && _origin.z == 0)
      _origin = origin;
    return _origin;
  }

  /**
   * Add a geometry for rendering.
   *
   * The geometry coordinates are expected to have the origin substracted. The
   * origin should be obtained by updateOrigin() called with any coordinate of
   * the geometry -- see _handleCamera() for an example.
   */
  void addGeometry(
      const std::string &name, ospray::cpp::GeometricModel &&geometry)
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    _activeGeometries.emplace(name, geometry);
    _dirty = true;
    _lastData = high_resolution_clock::now();
    std::cout << "+" << std::flush;
    _spinnerPos = -1;
  }

  void removeGeometry(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    if (_activeGeometries.erase(name) > 0) {
      _dirty = true;
      _lastData = high_resolution_clock::now();
      std::cout << "-" << std::flush;
      _spinnerPos = -1;
    }
  }

 private:
  void _setupHandlers()
  {
    _httpHandlers["/reset"] = [this](Connection &) {
      _reset();
      return 200;
    };
    _httpHandlers["/frame"] = [this](Connection &connection) {
      return _handleFrame(connection, *this);
    };
    _httpHandlers["/camera"] = [this](Connection &connection) {
      return _handleCamera(connection, *this);
    };
    _httpHandlers["/webscene"] = [this](Connection &connection) {
      return _handleWebscene(connection, *this);
    };
    _httpHandlers["/favicon.ico"] = [this](Connection &) { return 404; };
  }

  void _loadLayer(const std::string &serviceUrl)
  {
    // format: pbf, where: 1=1 (i.e. fetch all)
    const std::string dataUrl = serviceUrl + "/query?f=pbf&where=1%3D1";
    _loadPBF(*this, dataUrl);
  }

  void _startAccept()
  {
    const auto connection = newConnection();
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
    _camera.setParam("apertureRadius", .015f);

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

    _group.commit();
    _instance.commit();
    _world.setParam("instance", ospray::cpp::Data(_instance));
    _world.commit();

    // complete setup of renderer
    _renderer.setParam("aoSamples", 1);
    _renderer.setParam("aoTransparencyEnabled", true);
    _renderer.setParam("shadowsEnabled", true);
    _renderer.setParam("backgroundColor", 0.0f); // white, transparent
    _renderer.setParam("epsilon", 3 * std::numeric_limits<float>::epsilon());
    _renderer.commit();

    _clear();
  }

  void _clear()
  {
    _framebuffer.clear();
    _passes = 0;
    _lastFrame = high_resolution_clock::now();
  }

  void _reset()
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    _activeGeometries.clear();
    _origin = {0};
    _dirty = true;
    _lastData = high_resolution_clock::now();
    _spinnerPos = -1;
  }

  void _render()
  {
    _commit();
    if (_renderGeometries.empty())
      return;

    _framebuffer.renderFrame(_renderer, _camera, _world);
    ++_passes;
    if (_spinnerPos >= 0)
      std::cout << '\b';
    std::cout << spinner[(++_spinnerPos) % spinner.length()] << std::flush;

    while ((high_resolution_clock::now() - _lastFrame).count() < 2.f) {
      _framebuffer.renderFrame(_renderer, _camera, _world);
      ++_passes;
      std::cout << '\b' << spinner[(++_spinnerPos) % spinner.length()]
                << std::flush;
    }

    _lastFrame = high_resolution_clock::now();
  }

  void _commit()
  {
    {
      std::lock_guard<std::mutex> lock(_dataMutex);
      if (!_dirty)
        return;

      _lastFrame = high_resolution_clock::now();
      _renderGeometries.clear();
      _renderGeometries.reserve(_activeGeometries.size());
      for (const auto &i : _activeGeometries)
        _renderGeometries.push_back(i.second);
      if (_renderGeometries.empty())
        return;
    }

    _group.setParam("geometry", ospray::cpp::Data(_renderGeometries));
    _group.commit();

#ifndef VISIBILITY
    _lights[1] = _headlight.handle();
    ospray::cpp::Data lights(1, OSP_LIGHT, _lights.data());
    lights.commit();
    _world.setParam("light", lights);
#endif

    _renderer.commit();
    _world.commit();
    _clear();
    _dirty = false;
  }

  io_services _ioServices;
  const boost::asio::ip::tcp::endpoint _endpoint;
  boost::asio::ip::tcp::acceptor _acceptor;

  Webscene _webscene;

  mutable std::mutex _ospMutex;
  vec2i _size{1024, 576};
  ospray::cpp::Camera _camera{"perspective"};
  ospray::cpp::Light _headlight{"sphere"};
  std::vector<OSPLight> _lights{3};
  ospray::cpp::Material _material{rendererType, "OBJMaterial"};
  ospray::cpp::Material _emissive{rendererType, "Luminous"};

  vec3d _origin{0};
  ospray::cpp::Group _group;
  ospray::cpp::Instance _instance{_group};
  ospray::cpp::World _world;

  mutable std::mutex _dataMutex;
  std::map<std::string, ospray::cpp::GeometricModel> _activeGeometries;
  std::vector<ospray::cpp::GeometricModel> _renderGeometries;

  bool _dirty = false;
  std::chrono::time_point<high_resolution_clock> _lastFrame =
      high_resolution_clock::now();
  std::chrono::time_point<high_resolution_clock> _lastData =
      high_resolution_clock::now();
  size_t _passes{0};
  ospray::cpp::Renderer _renderer{rendererType};
  ssize_t _spinnerPos{-1};

  ospray::cpp::FrameBuffer _framebuffer{
      _size, OSP_FB_SRGBA, OSP_FB_COLOR | OSP_FB_ACCUM};

  HandlerMap _httpHandlers;
};

int _handleCamera(Connection &connection, Server &server)
{
  Json::Value json;
  Json::Reader reader;
  if (!reader.parse(connection.body(), json)) {
    std::cerr << "Error parsing JSON: " << reader.getFormattedErrorMessages()
              << std::endl
              << "  while parsing: " << connection.body() << std::endl;
    return 400;
  }

  auto size = server.size();
  if (json.isMember("width"))
    size.x = json["width"].asUInt();
  if (json.isMember("height"))
    size.y = json["height"].asUInt();
  server.setSize(size);

  auto camera = server.camera();
  if (json.isMember("position")) {
    const auto position = vec3d{json["position"][0].asDouble(),
        json["position"][1].asDouble(),
        json["position"][2].asDouble()};
    const auto origin = server.updateOrigin(position);
    camera.setParam("position", vec3f{position - origin});

    if (json.isMember("lookat")) {
      const auto lookat = vec3d{json["lookat"][0].asDouble(),
          json["lookat"][1].asDouble(),
          json["lookat"][2].asDouble()};
      camera.setParam("direction", vec3f{lookat - position});
    }
  }
  if (json.isMember("up")) {
    const auto up = json["up"];
    camera.setParam(
        "up", vec3f{up[0].asFloat(), up[1].asFloat(), up[2].asFloat()});
  }
  if (json.isMember("fovX")) {
    const float fovX = json["fovX"].asFloat() * 57.295779513f;
    camera.setParam("fovx", fovX);
  }
  if (json.isMember("fovY")) {
    const float fovY = json["fovY"].asFloat() * 57.295779513f;
    camera.setParam("fovy", fovY);
  }

  camera.setParam("aspect", size.x / (float)size.y);
  // #ifndef VISIBILITY
  //   const float distance = length(dir);
  //   _camera.set("focusDistance", distance);
  //   _camera.set("apertureRadius", std::min(1.f, distance * 0.0001f));

  //   _headlight.set("direction", dir);
  //   _headlight.set("position", position);
  //   _headlight.commit();
  // #endif
  server.setCamera(camera);

  if (false /*how to get data:*/) {
    auto connection = server.newConnection();
    connection->url() = "/~eile/";
    connection->write("localhost", 80, []() {});
  }

  connection.body().clear();
  connection.headers().clear();
  return 200;
}

int _handleWebscene(Connection &connection, Server &server)
{
  server.setWebscene({connection.body()});
  connection.body().clear();
  connection.headers().clear();
  return 200;
}

int _handleFrame(Connection &connection, Server &server)
{
  const auto frameBuffer = server.frame();
  auto fb = (unsigned char *)frameBuffer.map(OSP_FB_COLOR);
  const auto &size = server.size();

  connection.body().resize(TJBUFSIZE(size[0], size[1]));
  unsigned long compressedSize = 0;
  void *encoder{tjInitCompress()};
  int status = 200;
  if (tjCompress(encoder,
          fb,
          size[0],
          size[0] * 4,
          size[1],
          4,
          (unsigned char *)connection.body().data(),
          &compressedSize,
          TJ_444,
          100,
          TJ_BOTTOMUP)
      != 0) {
    std::cerr << tjGetErrorStr() << std::endl;
    connection.headers() = {{"Content-Type", "text/plain"}};
    connection.body() = tjGetErrorStr();
    status = 500;
  } else {
    connection.headers() = {{"Content-Type", "image/jpeg"}};
    connection.body().resize(compressedSize);
    std::cout << server.passesRendered() << std::flush;
  }

  tjDestroy(encoder);
  frameBuffer.unmap(fb);
  return status;
}

void _loadPBF(Server &server, const std::string &url)
{
  struct http_parser_url u;

  int a = http_parser_parse_url(url.c_str(), url.length(), 0, &u);
  if (a != 0) {
    std::cerr << "Unable to parse given url: " << url << std::endl;
    return;
  }

  const bool hasPort = u.field_set & (1 << UF_PORT) != 0;
  const bool hasHost = u.field_set & (1 << UF_HOST) != 0;
  const bool hasPath = u.field_set & (1 << UF_PATH) != 0;
  const bool hasQuery = u.field_set & (1 << UF_QUERY) != 0;

  int port = 80;
  if (!hasPort) {
    // port is not set -> derive from schema
    if (u.field_set & (1 << UF_SCHEMA) != 0) {
      const std::string schema{url.c_str() + u.field_data[UF_SCHEMA].off,
          u.field_data[UF_SCHEMA].len};
      if (schema == "https") {
        port = 443;
      } else {
        if (schema != "http") {
          std::cout << "Unsupported schema, using default port 80" << std::endl;
        }
      }
    }
  }

  if (!hasHost) {
    std::cerr << "Host is undefined - can not load" << std::endl;
    return;
  }

  const std::string host{
      url.c_str() + u.field_data[UF_HOST].off, u.field_data[UF_HOST].len};

  std::string path = "/";
  std::string query = "";
  if (hasPath) {
    path = std::string{
        url.c_str() + u.field_data[UF_PATH].off, u.field_data[UF_PATH].len};
  }
  if (hasQuery) {
    query = std::string{
        url.c_str() + u.field_data[UF_QUERY].off, u.field_data[UF_QUERY].len};
  }

  auto connection = server.newConnection();
  connection->url() = path + "?" + query;
  connection->write(host, port, [connection, &server]() {
    if (connection->headers()["Content-Type"] != "application/x-protobuf") {
      std::cerr << "Expected proto buffer - ignoring";
      return;
    }
    std::stringstream input(connection->body(), std::ios::binary);
    esriPBuffer::FeatureCollectionPBuffer buffer;
    if (!buffer.ParseFromString(connection->body())) {
      std::cerr << "Failed to parse FeatureCollectionPBuffer." << std::endl;
      return;
    }

    const auto &queryresult = buffer.queryresult();
    if (!queryresult.has_featureresult()) {
      std::cerr << "Empty FeatureCollectionPBuffer." << std::endl;
      return;
    }

    const auto &featureresult = queryresult.featureresult();
    if (!featureresult.has_spatialreference()) {
      std::cerr
          << "No spatial reference found - we don't know how to reproject. Ignoring"
          << std::endl;
      return;
    }

    const auto sr = featureresult.spatialreference();
    if (sr.wkid() != 102100 && sr.lastestwkid() != 3857) {
      std::cerr << "Can only handle WebMercator - aborting now." << std::endl;
      return;
    }

    std::cout << "Got PBF from link :)" << std::endl;

    std::vector<vec3f> centers;

    const auto scale = featureresult.transform().scale();
    const auto translate = featureresult.transform().translate();
    const bool hasZ = featureresult.hasz();

    for (const auto &feature : featureresult.features()) {
      if (!feature.has_geometry()
          || feature.geometry().geometrytype()
              != esriPBuffer::
                  FeatureCollectionPBuffer_GeometryType_esriGeometryTypePoint)
        continue;

      const auto &coords = feature.geometry().coords();
      const double x =
          std::round(coords[0] * scale.xscale() + translate.xtranslate());
      const double y =
          std::round(coords[1] * scale.yscale() + translate.ytranslate());
      const double z = hasZ
          ? std::round(coords[2] * scale.zscale() + translate.ztranslate())
          : 1;
      auto position = vec3d(x, y, z);
      convert::webMercatorToSphericalECEF(position);
      const auto origin = server.updateOrigin(position);
      centers.emplace_back(position.x - origin.x,
          position.y - origin.y,
          (hasZ ? position.z - origin.z : 1));
    }

    if (centers.empty()) {
      std::cerr << "No points in FeatureCollectionPBuffer." << std::endl;
      return;
    }

    // create the sphere geometry, and assign attributes
    ospray::cpp::Geometry spheres("sphere");
    spheres.setParam("sphere.position", ospray::cpp::Data(centers));

    std::vector<float> radii(centers.size(), 0.05f);
    spheres.setParam("sphere.radius", ospray::cpp::Data(radii));
    spheres.commit();

    ospray::cpp::GeometricModel model(spheres);

    std::vector<vec4f> colors(centers.size(), vec4f(0.3f, 0.7f, 0.92f, 1.0f));
    model.setParam("color", ospray::cpp::Data(colors));

    if (rendererType == "pathtracer") {
      // create glass material and assign to geometry
      ospray::cpp::Material glassMaterial(rendererType, "thinGlass");
      glassMaterial.setParam("attenuationDistance", 0.2f);
      glassMaterial.commit();
      model.setParam("material", glassMaterial);
    } else if (rendererType == "scivis") {
      ospray::cpp::Material glassMaterial(rendererType, "obj");
      glassMaterial.commit();
      model.setParam("material", glassMaterial);
    }

    model.commit();
    server.addGeometry("my-spheres", std::move(model));

    // Optional:  Delete all global objects allocated by libprotobuf.
    google::protobuf::ShutdownProtobufLibrary();
  });
}
} // namespace

int main(int argc, const char **argv)
{
  // initialize OSPRay; OSPRay parses (and removes) its commandline
  // parameters, e.g. "--osp:debug"
  OSPError init_error = ospInit(&argc, argv);
  if (init_error != OSP_NO_ERROR)
    return init_error;

  try {
    const size_t numThreads = 8;
    const int port = 4242;
    const std::string interface = "127.0.0.1";
    io_services ioServices;
    std::deque<boost::asio::io_service::work> work;

    boost::thread_group threads;

    for (size_t i = 0; i < numThreads; ++i) {
      io_service_ptr ioService{new boost::asio::io_service};
      ioServices.push_back(ioService);
      work.push_back(boost::asio::io_service::work(*ioService));
      threads.create_thread([ioService]() { ioService->run(); });
    }

    Server server(ioServices, port, interface);

    // render thread
    threads.create_thread([&server]() {
      while (true) {
        if (server.idle() && server.passesRendered() < 256) {
          server.render();
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });

    threads.join_all();
  } catch (std::exception &e) {
    std::cerr << "Error running irts server: " << e.what() << std::endl;
  }

  ospShutdown();

  return 0;
}
