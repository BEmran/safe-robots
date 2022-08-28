#ifndef CORE_UTILS_MODULE_HPP
#define CORE_UTILS_MODULE_HPP

#include <string>

namespace core::utils {
/**
 * @brief defines module types
 *
 */
enum class ModuleType { UNDEFINED, SENSOR };

/**
 * @brief Converts module type to string
 *
 * @param type module type to be converted
 * @return std::string name of module type
 */
std::string ModuleTypeToString(const ModuleType type);

/**
 * @brief Abstracts class used to define main object of beaglebone hardware
 *
 */
class ModuleAbs {
 public:
  /**
   * @brief Construct a new Module object
   *
   * @param type module's type
   * @param name module's name
   * @param debug enable/disable debug
   */
  ModuleAbs(const ModuleType type, const std::string& name, const bool debug);

  /**
   * @brief Destroy the Module Abs object
   *
   */
  virtual ~ModuleAbs() = default;

  /**
   * @brief returns the type of the Module Abs object
   *
   * @return ModuleType module's type
   */
  ModuleType Type() const;

  /**
   * @brief returns the name of the Module Abs object
   *
   * @return std::string module's name
   */
  std::string Name() const;

  /**
   * @brief indicate if debug flag is enabled or not
   *
   * @return true if enabled
   * @return false if disabled
   */
  bool IsDebugEnabled() const;

 private:
  ModuleType type_{ModuleType::UNDEFINED};
  std::string name_{"UNNAMED"};
  bool debug_{false};
};

}  // namespace core::utils
#endif  // CORE_UTILS_MODULE_HPP
