// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/hardware/model.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>  // for system()
#include <fstream>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

#include "core/utils/logger_macros.hpp"

namespace core::hardware {

namespace {
constexpr std::string_view kModelDir = "/proc/device-tree/model";

const std::unordered_map<std::string_view, ModelType> ModelTypeString{
  {"Beagle", ModelType::BEAGLEBONE},
  {"Raspberry Pi", ModelType::RPI},
  {"PC", ModelType::PC}};

const std::vector<HardwareType> BBHardwareTypes{
  HardwareType::BB_BLACK_RC, HardwareType::BB_BLACK_W, HardwareType::BB_BLACK_W,
  HardwareType::BB_BLACK,    HardwareType::BB_GREEN_W, HardwareType::BB_GREEN,
  HardwareType::BB_BLUE,     HardwareType::BB_BLACK_W};

const std::vector<HardwareType> RPIHardwareTypes{
  HardwareType::RPI_B_PLUS,  HardwareType::RPI_B,   HardwareType::RPI2_B,
  HardwareType::RPI3_B_PLUS, HardwareType::RPI3_B,  HardwareType::RPI0_W,
  HardwareType::RPI0,        HardwareType::RPI_CM3, HardwareType::RPI_CM};

const std::map<std::string_view, HardwareType> HardwareTypeString{
  {"TI AM335x BeagleBone Black RoboticsCape", HardwareType::BB_BLACK_RC},
  {"TI AM335x BeagleBone Black Wireless RoboticsCape",
   HardwareType::BB_BLACK_W},
  {"TI AM335x BeagleBone Black Wireless", HardwareType::BB_BLACK_W},
  {"TI AM335x BeagleBone Black", HardwareType::BB_BLACK},
  {"TI AM335x BeagleBone Green Wireless", HardwareType::BB_GREEN_W},
  {"TI AM335x BeagleBone Green", HardwareType::BB_GREEN},
  {"TI AM335x BeagleBone Blue", HardwareType::BB_BLUE},
  {"TI AM335x PocketBeagle", HardwareType::BB_BLACK_W},
  {"Raspberry Pi Model B+", HardwareType::RPI_B_PLUS},
  {"Raspberry Pi Model B", HardwareType::RPI_B},
  {"Raspberry Pi 2 Model B", HardwareType::RPI2_B},
  {"Raspberry Pi 3 Model B+", HardwareType::RPI3_B_PLUS},
  {"Raspberry Pi 3 Model", HardwareType::RPI3_B},
  {"Raspberry Pi Zero W", HardwareType::RPI0_W},
  {"Raspberry Pi Zero", HardwareType::RPI0},
  {"Raspberry Pi Computer Module 3", HardwareType::RPI_CM3},
  {"Raspberry Pi Compute Module", HardwareType::RPI_CM},
  {"PC", HardwareType::PC}};

std::vector<std::string> ReadAllLinesFromFile(std::string_view filename) {
  std::string line;
  std::vector<std::string> lines;
  std::ifstream file;
  file.open(filename.data(), std::ios::in);

  if (file.is_open()) {
    while (std::getline(file, line)) {
      lines.push_back(line);
    }
    file.close();
  } else {
    SYS_LOG_ERROR("ERROR reading " + filename.data());
  }
  return lines;
}

// check for x86/x86_64 personal computer
int CheckForPersonalComputer() {
  const auto ret = system("uname --machine | grep x86 --quiet");
  return ret == 0;
}

template <template <typename...> class Map, typename ENUM, typename... Ts>
std::optional<ENUM> Find(std::string txt,
                         Map<std::string_view, ENUM, Ts...> map) {
  const auto itr = std::find_if(map.begin(), map.end(), [txt](auto pair) {
    return txt.find(pair.first) != std::string::npos;
  });
  if (itr == map.end()) {
    return {};
  }
  return itr->second;
}

HardwareType ExtractHardwareType(std::string str) {
  const auto result = Find(str, HardwareTypeString);
  return result.value_or(HardwareType::UNKNOWN);
}

ModelType ExtractModelType(std::string str) {
  const auto result = Find(str, ModelTypeString);
  return result.value_or(ModelType::UNKNOWN);
}
}  // namespace

ModelType GetModeType(const HardwareType type) {
  auto itr1 = std::find(BBHardwareTypes.begin(), BBHardwareTypes.end(), type);
  if (itr1 != BBHardwareTypes.end()) {
    return ModelType::BEAGLEBONE;
  }
  auto itr2 = std::find(RPIHardwareTypes.begin(), RPIHardwareTypes.end(), type);
  if (itr2 != RPIHardwareTypes.end()) {
    return ModelType::RPI;
  }
  if (type == HardwareType::PC) {
    return ModelType::PC;
  }
  return ModelType::UNKNOWN;
}

HardwareModel::HardwareModel(const HardwareType type) {
  model_ = GetModeType(type);
  hardware_ = type;
}

HardwareModel::HardwareModel(std::string_view str) {
  model_ = ExtractModelType(str.data());
  hardware_ = ExtractHardwareType(str.data());
}

HardwareModel ExtractHardwareModel() {
  if (CheckForPersonalComputer()) {
    return HardwareModel(HardwareType::PC);
  } else {
    return ExtractHardwareModelFromFile(kModelDir);
  }
}

HardwareModel ExtractHardwareModelFromFile(std::string_view filename) {
  // read model fom device tree
  auto lines = ReadAllLinesFromFile(filename);
  if (lines.size() != 1) {
    SYS_LOG_ERROR("ERROR unexpected data size");
  }
  return HardwareModel(lines[0]);
}

std::string HardwareTypeToString(const HardwareType type) {
  auto itr =
    std::find_if(HardwareTypeString.begin(), HardwareTypeString.end(),
                 [type](const auto& pair) { return pair.second == type; });
  if (itr == HardwareTypeString.end()) {
    return "";
  }
  return std::string{itr->first};
}

std::string ModelTypeToString(const ModelType type) {
  auto itr =
    std::find_if(ModelTypeString.begin(), ModelTypeString.end(),
                 [type](const auto& pair) { return pair.second == type; });
  if (itr == ModelTypeString.end()) {
    return "";
  }
  return std::string{itr->first};
}
}  // namespace core::hardware