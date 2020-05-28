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

 private:
  Json::Value _json;
};
} // namespace