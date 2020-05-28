#pragma once

#include "jsoncpp/json/json.h"

#include <stdexcept>
#include <string>

namespace {
class Webscene
{
 public:
  Webscene() {}
  Webscene(const std::string &data)
  {
    Json::Reader reader;
    if (!reader.parse(data, _json))
      throw new std::runtime_error(std::string("Error parsing JSON: ")
          + reader.getFormattedErrorMessages() + +"\n  while parsing: " + data);
  }

  std::vector<std::string> getFeatureLayers() const
  {
    if (!_json.isMember("operationalLayers"))
      return {};

    const auto layersJSON = _json["operationalLayers"];
    if (!layersJSON.isArray())
      return {};

    std::vector<std::string> layers;
    for (unsigned i = 0; i < layersJSON.size(); ++i) {
      const auto layerJSON = layersJSON[i];
      if (!layerJSON.isMember("layerType") || !layerJSON.isMember("url"))
        continue;

      const auto layerType = layerJSON["layerType"].asString();
      if (layerType != "ArcGISFeatureLayer")
        continue;

      layers.emplace_back(layerJSON["url"].asString());
    }

    return layers;
  }

 private:
  Json::Value _json;
};
} // namespace