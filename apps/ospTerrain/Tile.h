#pragma once

#include <turbojpeg.h>

#include <string>
#include "ospcommon/math/vec.h"

using namespace ospcommon::math;

class Tile
{
 public:
  enum TileType
  {
    IMAGE,
    ELEVATION
  };

	Tile();
	~Tile();

	void readURL(std::string url, TileType type);
	double* getElevation();
	unsigned char* getImage();

public:
	double *elevation;
	unsigned char *imagedata;
	
	vec2ui pixelSize;
	vec2d dataRange;

};
