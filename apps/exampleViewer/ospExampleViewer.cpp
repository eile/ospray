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

#include "common/sg/SceneGraph.h"
#include "sg/geometry/TriangleMesh.h"

#include "ospapp/OSPApp.h"
#include "widgets/imguiViewer.h"

#include "data.h"

#include <zeroeq/URI.h>
#include <zeroeq/http/Server.h>
#include <zeroeq/http/helpers.h>

namespace http = zeroeq::http;

namespace ospray {
  namespace app {

    struct Sphere
    {
      Sphere(const vec3f &position = vec3f(0.f),
             float radius          = 1.f,
             uint32_t typeID       = 0)
          : position(position), radius(radius), typeID(typeID)
      {
      }

      box3f bounds() const
      {
        return {position - vec3f(radius), position + vec3f(radius)};
      }

      vec3f position;
      float radius;
      uint32_t typeID;
    };

    class ImGuiViewer : public ospray::ImGuiViewer
    {
     public:
      ImGuiViewer(const std::shared_ptr<sg::Node> scenegraph)
          : ospray::ImGuiViewer(scenegraph),
            _server(zeroeq::URI(":4242")),
            _scene(scenegraph)
      {
        _server.handle(
            http::Method::POST, "points", [this](const http::Request &request) {
              return _handlePoints(request);
            });
        std::cerr << "Bound to " << _server.getURI() << std::endl;
      }

     private:
      void display() override
      {
        auto &world = (*_scene)["World"];

        if (_server.receive(10)) {
          while (_server.receive(0))
            /*nop, drain*/;
          world.markAsModified();
        } else
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ospray::ImGuiViewer::display();
      }

      std::future<http::Response> _handlePoints(const http::Request &request)
      {
        ospray::data::Points points;
        points.fromJSON(request.body);

        const auto &origin           = points.getOrigin();
        static const float _global[] = {
            origin.get0(), origin.get1(), origin.get2()};

        auto &world             = (*_scene)["World"];
        const auto name         = points.getIdString();
        const auto &bytes       = points.getPositions();
        const size_t nPositions = bytes.size() / sizeof(float);
        if (nPositions == 0) {
          std::cerr << "Remove " << name << std::endl;
          world.remove(name + "_instance");
          return http::make_ready_response(
              http::Code::OK);  // todo: remove node
        }

        const float *positions = (float *)(bytes.data());
        const auto &rgb        = points.getColors();

        auto bounds     = world.bounds();
        auto sphereData = std::make_shared<sg::DataVectorT<Sphere, OSP_RAW>>();
        auto colorData =
            std::make_shared<sg::DataVectorT<sg::vec4f, OSP_FLOAT4>>();
        auto &colors = colorData->v;

        for (size_t i = 0; i < nPositions; i += 3) {
          Sphere s;
          s.position.x =
              (positions[i + 0] + origin.get0() - _global[0]) / 100.f;
          s.position.y =
              (positions[i + 1] + origin.get1() - _global[1]) / 100.f;
          s.position.z =
              (positions[i + 2] + origin.get2() - _global[2]) / 100.f;
          s.radius = .03;

          const sg::vec4f color(float(rgb[i + 0]) / 255.f,
                                float(rgb[i + 1]) / 255.f,
                                float(rgb[i + 2]) / 255.f,
                                0.5f);
          const auto j = std::find(colors.begin(), colors.end(), color);
          size_t index = 0;
          if (j == colors.end()) {
            index = colors.size();
            colors.push_back(color);
          } else
            index = std::distance(colors.begin(), j);

          s.typeID = index;

          sphereData->v.push_back(s);
          bounds.extend(s.position - s.radius);
          bounds.extend(s.position + s.radius);
        }

        auto spGeom = sg::createNode(name + "_geometry", "Spheres")
                          ->nodeAs<sg::Spheres>();

        spGeom->createChild("bytes_per_sphere", "int", int(sizeof(Sphere)));
        spGeom->createChild("offset_center", "int", int(0));
        spGeom->createChild("offset_radius", "int", int(3 * sizeof(float)));
        spGeom->createChild("offset_colorID", "int", int(4 * sizeof(float)));
        spGeom->createChild("color_offset", "int", int(0 * sizeof(float)));
        spGeom->createChild("color_stride", "int", int(4 * sizeof(float)));

        sphereData->setName("spheres");
        spGeom->add(sphereData);

        colorData->setName("colorData");
        spGeom->add(colorData);

        auto model = sg::createNode(name + "_model", "Model");
        model->add(spGeom);

        auto instance = sg::createNode(name + "_instance", "Instance");
        instance->setChild("model", model);
        model->setParent(instance);

        sg::RenderContext ctx;
        static_cast<sg::Renderer &>(*_scene).preRender(ctx);
        instance->traverse(ctx, "verify");
        instance->traverse(ctx, "commit");

        world.add(instance);
        //_scene->traverse("verify");
        //_scene->traverse("commit");
        //  _server.remove("points");
        std::cerr << "Add " << nPositions / 3 << " spheres for " << name
                  << std::endl;

        return http::make_ready_response(http::Code::OK);
      }

      http::Server _server;
      std::shared_ptr<sg::Node> _scene;
    };

    class OSPExampleViewer : public OSPApp
    {
      void render(const std::shared_ptr<sg::Frame> &) override;
      int parseCommandLine(int &ac, const char **&av) override;

      bool fullscreen   = false;
      float motionSpeed = -1.f;
      std::string initialTextForNodeSearch;
    };

    void OSPExampleViewer::render(const std::shared_ptr<sg::Frame> &root)
    {
      ospray::ImGuiViewer window(root);

      window.create(
          "OSPRay Example Viewer App", fullscreen, vec2i(width, height));

      if (motionSpeed > 0.f)
        window.setMotionSpeed(motionSpeed);

      if (!initialTextForNodeSearch.empty())
        window.setInitialSearchBoxText(initialTextForNodeSearch);

      window.setColorMap(defaultTransferFunction);

      // emulate former negative spp behavior
      auto &renderer = root->child("renderer");
      int spp        = renderer["spp"].valueAs<int>();
      if (spp < 1) {
        window.navRenderResolutionScale = ::powf(2, spp);
        renderer["spp"]                 = 1;
      }

      imgui3D::run();
    }

    int OSPExampleViewer::parseCommandLine(int &ac, const char **&av)
    {
      for (int i = 1; i < ac; i++) {
        const std::string arg = av[i];
        if (arg == "--fullscreen") {
          fullscreen = true;
          removeArgs(ac, av, i, 1);
          --i;
        } else if (arg == "--motionSpeed") {
          motionSpeed = atof(av[i + 1]);
          removeArgs(ac, av, i, 2);
          --i;
        } else if (arg == "--searchText") {
          initialTextForNodeSearch = av[i + 1];
          removeArgs(ac, av, i, 2);
          --i;
        }
      }
      return 0;
    }

  }  // namespace app
}  // namespace ospray

int main(int ac, const char **av)
{
  ospray::app::OSPExampleViewer ospApp;
  return ospApp.main(ac, av);
}
