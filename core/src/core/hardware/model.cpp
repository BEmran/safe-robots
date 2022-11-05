// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/hardware/model.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

#include "core/utils/logger_macros.hpp"
#include "core/utils/system.hpp"

namespace core::hardware {

namespace {
/// @brief name and path of device tree file
constexpr std::string_view kDeviceTreeFile = "/proc/device-tree/model";

/// @brief system command to run uname and check if it is of x86 architecture
constexpr std::string_view kUnameCommand = "uname --machine | grep x86 --quiet";

/// @brief list of all BeagleBone board type
constexpr std::array BBHardwareTypes{
  HardwareType::BB_BLACK,   HardwareType::BB_BLACK_RC,
  HardwareType::BB_BLACK_W, HardwareType::BB_BLACK_W_RC,
  HardwareType::BB_BLUE,    HardwareType::BB_GREEN,
  HardwareType::BB_GREEN_W, HardwareType::BB_POCKET};

/// @brief list of all Raspberry PI board type
constexpr std::array RPIHardwareTypes{
  HardwareType::RPI0,        HardwareType::RPI0_W, HardwareType::RPI_B,
  HardwareType::RPI_B_PLUS,  HardwareType::RPI2_B, HardwareType::RPI3_B,
  HardwareType::RPI3_B_PLUS, HardwareType::RPI_CM, HardwareType::RPI_CM3};

/// @brief map of all hardware type and their actual name
const std::map<std::string_view, HardwareType, std::greater<std::string_view>>
  HardwareTypeMap{
    {"BeagleBone Black RoboticsCape", HardwareType::BB_BLACK_RC},
    {"BeagleBone Black Wireless RoboticsCape", HardwareType::BB_BLACK_W_RC},
    {"BeagleBone Black Wireless", HardwareType::BB_BLACK_W},
    {"BeagleBone Black", HardwareType::BB_BLACK},
    {"BeagleBone Blue", HardwareType::BB_BLUE},
    {"BeagleBone Green Wireless", HardwareType::BB_GREEN_W},
    {"BeagleBone Green", HardwareType::BB_GREEN},
    {"PocketBeagle", HardwareType::BB_BLACK_W},
    {"Raspberry Pi Model B Plus", HardwareType::RPI_B_PLUS},
    {"Raspberry Pi Model B", HardwareType::RPI_B},
    {"Raspberry Pi 2 Model B", HardwareType::RPI2_B},
    {"Raspberry Pi 3 Model B Plus", HardwareType::RPI3_B_PLUS},
    {"Raspberry Pi 3 Model", HardwareType::RPI3_B},
    {"Raspberry Pi Zero W", HardwareType::RPI0_W},
    {"Raspberry Pi Zero", HardwareType::RPI0},
    {"Raspberry Pi Computer Module 3", HardwareType::RPI_CM3},
    {"Raspberry Pi Compute Module", HardwareType::RPI_CM},
    {"PC", HardwareType::PC}};

/**
 * @brief Read all lines written in the passed filename
 *
 * @param filename name of the file to read
 * @return std::vector<std::string> list of red lines
 */
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

/**
 * @brief check if current board is personal computer
 * @details by reading the device name and search for x86/x86_64
 *
 * @return true if current board is personal computer
 * @return false otherwise
 */
bool IsPersonalComputer() {
  const auto ret = utils::ExecuteSystemCommand(kUnameCommand.data());
  if (ret.has_value()) {
    return ret.value() == 0;
  } else {
    return false;
  }
}

/**
 * @brief Get the First Line from a File
 *
 * @param filename name of file to read
 * @return std::optional<std::string> first line if exists
 */
std::optional<std::string> GetFirstLineFromFile(std::string_view filename) {
  auto lines = ReadAllLinesFromFile(filename);
  if (lines.size() != 1) {
    SYS_LOG_WARN("unexpected data size");
    return {};
  }
  return lines[0];
}

/**
 * @brief Find a matched key to the passed text
 * @details container assumed to be of type map<string_view, VALUE, ...>
 *
 * @tparam Map templated container
 * @tparam VALUE value type
 * @tparam Ts other templated types
 * @param txt string to find match to
 * @param map map to search its keys
 * @return std::optional<VALUE> value if matched is found
 */
template <template <typename...> class Map, typename VALUE, typename... Ts>
std::optional<VALUE> FindValue(const std::string& txt,
                               Map<std::string_view, VALUE, Ts...> map) {
  const auto itr = std::find_if(map.begin(), map.end(), [txt](auto pair) {
    return txt.find(pair.first) != std::string::npos;
  });
  if (itr == map.end()) {
    return {};
  }
  return itr->second;
}

/**
 * @brief Extract Hardware Type from passed string
 *
 * @param txt string contains hardware information
 * @return HardwareType hardware type
 */
HardwareType ExtractHardwareType(std::string txt) {
  const auto result = FindValue(txt, HardwareTypeMap);
  return result.value_or(HardwareType::UNKNOWN);
}

/**
 * @brief Get Model Type from passed Hardware Type
 *
 * @param type Hardware type
 * @return ModelType model type
 */

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
}  // namespace

HardwareModel::HardwareModel(const HardwareType type) {
  hardware = type;
  model = GetModeType(type);
}

HardwareModel::HardwareModel(std::string_view str)
  : HardwareModel(ExtractHardwareType(str.data())) {
}

std::string HardwareModel::ToString() {
  auto itr =
    std::find_if(HardwareTypeMap.begin(), HardwareTypeMap.end(),
                 [this](const auto& pair) { return pair.second == hardware; });

  if (itr == HardwareTypeMap.end()) {
    SYS_LOG_WARN("undefined HardwareType");
    return "";
  }
  return itr->first.data();
}

HardwareModel ExtractHardwareModel() {
  if (IsPersonalComputer()) {
    return HardwareModel(HardwareType::PC);
  }
  return ExtractHardwareModelFromFile(kDeviceTreeFile);
}

HardwareModel ExtractHardwareModelFromFile(std::string_view filename) {
  const auto line = GetFirstLineFromFile(filename);
  if (line.has_value()) {
    return HardwareModel(line.value());
  } else {
    return HardwareModel();
  }
}
}  // namespace core::hardware