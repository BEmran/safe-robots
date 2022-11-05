
// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_HARDWARE_MODEL_HPP
#define CORE_HARDWARE_MODEL_HPP

#include <string>

namespace core::hardware {
/**
 * @brief List of hardware models
 */
enum class HardwareType {
  BB_BLACK,       // BeagleBone Black
  BB_BLACK_RC,    // BeagleBone Black RoboticsCape
  BB_BLACK_W,     // BeagleBone Black Wireless
  BB_BLACK_W_RC,  // BeagleBone Black Wireless RoboticsCape
  BB_BLUE,        // BeagleBone Blue
  BB_GREEN,       // BeagleBone Green
  BB_GREEN_W,     // BeagleBone Green Wireless
  BB_POCKET,      // PocketBeagle
  PC,             // Personal Computer
  RPI0,           // Raspberry Pi Zero
  RPI0_W,         // Raspberry Pi Zero Wireless
  RPI_B,          // Raspberry Pi Model B
  RPI_B_PLUS,     // Raspberry Pi Model B+
  RPI2_B,         // Raspberry Pi 2 Model B
  RPI3_B,         // Raspberry Pi 3 Model B
  RPI3_B_PLUS,    // Raspberry Pi 3 Model B+
  RPI_CM,         // Raspberry Pi Computer Module
  RPI_CM3,        // Raspberry Pi Compute Module 3
  UNKNOWN         // Unknown board
};

/**
 * @brief List of general model of boards.
 */
enum class ModelType {
  BEAGLEBONE,  // BeagleBone
  RPI,         // Raspberry Pi
  PC,          // Personal Computer
  UNKNOWN      // Unknown Model
};

/**
 * @brief Determine the model and board currently being used.
 *
 */
struct HardwareModel {
  /// @brief Model type
  ModelType model{ModelType::UNKNOWN};

  /// @brief Hardware type
  HardwareType hardware{HardwareType::UNKNOWN};

  /**
   * @brief Construct a default Hardware Model object
   *
   */
  HardwareModel() = default;

  /**
   * @brief Construct a new Hardware Model object using Hardware type
   *
   * @param type Hardware type
   */
  HardwareModel(const HardwareType type);

  /**
   * @brief Construct a new Hardware Model object from a string
   *
   * @param text contains Hardware Model information
   */
  HardwareModel(std::string_view text);

  /**
   * @brief Get the Hardware Model information in string format
   *
   */
  std::string ToString();
};

/**
 * @brief Extract Hardware Model information of the current board
 * @details First it will check if the current hardware is personal computer,
 * otherwise it will extract Hardware Model from device tree file.
 *
 * @return HardwareModel contains hardware model
 */
HardwareModel ExtractHardwareModel();

/**
 * @brief Extract board Hardware Model information from a file
 *
 * @return HardwareModel contains hardware model
 */
HardwareModel ExtractHardwareModelFromFile(std::string_view filename);

}  // namespace core::hardware
#endif  // CORE_HARDWARE_MODEL_HPP