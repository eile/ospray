#pragma once

#include "ospray_testing.h"

#include "ArcballCamera.h"
#include "Tile.h"
#include "ospcommon/containers/TransactionalBuffer.h"
#include "ospcommon/utility/multidim_index_sequence.h"
#include "ospray/ospray_cpp.h"

using namespace ospcommon::math;
using namespace ospray;

class ospTerrainScene
{
 public:
  ospTerrainScene();
  ~ospTerrainScene();

  void addObjectToCommit(OSPObject obj);
  void refreshScene(bool resetCamera = false);
  bool commitOutstandingHandles();

 private:
  cpp::World createWorld();
  cpp::Group createBoxes();
  cpp::Group createTerrainMesh();

 public:
  cpp::World world;
  cpp::Renderer renderer;
  vec3f bgColor{0.5f};
  cpp::Texture backplateTex{"texture2d"};
  ospcommon::containers::TransactionalBuffer<OSPObject> objectsToCommit;
  vec3i dimensions{4};

  Tile elevationTile;
  Tile imageTile;
};
