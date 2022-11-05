
// Copyright (C) 2022 Bara Emran - All Rights Reserved
/**
 * @brief Determine the model of board currently being used.
 *
 * The user may sometimes need to determine which board they are running on to
 * decide which pins or modules to use. These functions are also used internally
 * to the library for hardware specific configuration. See the rc_model example
 * for a demonstration.
 */

#ifndef CORE_HARDWARE_MODEL_HPP
#define CORE_HARDWARE_MODEL_HPP

#include <string>

namespace core::hardware {
/**
 * @brief List of models detectable by the HardwareModel() function. This is not
 * a complete list of board this library will run on. Nor is it a list of boards
 * that the library is guaranteed to work on. Currently, MODEL_PC indicates
 * personal computer and will be selected whenever running on an x86 or x86_64
 * architecture machine until more specific boards are added to this list by
 * user request.
 *
 */
enum class HardwareType {
  BB_BLACK,
  BB_BLACK_RC,
  BB_BLACK_W,
  BB_BLACK_W_RC,
  BB_GREEN,
  BB_GREEN_W,
  BB_BLUE,
  BB_POCKET,
  PC,
  RPI_B,
  RPI_B_PLUS,
  RPI2_B,
  RPI3_B,
  RPI3_B_PLUS,
  RPI0,
  RPI0_W,
  RPI_CM,
  RPI_CM3,
  UNKNOWN
};

/**
 * This is a list of general categories of boards.
 */
enum class ModelType { BEAGLEBONE, RPI, PC, UNKNOWN };

struct HardwareModel {
  ModelType model_{ModelType::UNKNOWN};
  HardwareType hardware_{HardwareType::UNKNOWN};
  HardwareModel();
  HardwareModel(const HardwareType type);
  HardwareModel(std::string_view str);
};

HardwareModel ExtractHardwareModel();
HardwareModel ExtractHardwareModelFromFile(std::string_view filename);

/**
 * @brief Get the current board model name
 *
 * @return HardwareModel enum representation of model
 */
HardwareModel GetHardwareModel();

/**
 * @brief prints to the screen the human-readable version of the model name
 * with no trailing newline character.
 */
std::string HardwareTypeToString(const HardwareType type);

/**
 * @brief prints to the screen the human-readable version of the category
 * name with no trailing newline character.
 */
std::string ModelTypeToString(const ModelType type);
}  // namespace core::hardware
#endif  // CORE_HARDWARE_MODEL_HPP