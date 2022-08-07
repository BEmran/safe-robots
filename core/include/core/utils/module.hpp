#ifndef CORE_UTILS_MODULE_HPP
#define CORE_UTILS_MODULE_HPP

#include <string>

namespace core::utils
{
/**
 * @brief defines module types
 *
 */
enum class ModuleType
{
  UNDEFINED,
  SENSOR
};

/**
 * @brief Converts module type to string
 *
 * @param type module type to be converted
 * @return std::string name of module type
 */
std::string ModuleTypeToString(const ModuleType type);

/**
 * @brief Abustract class used to define main object of beaglebone hardware
 *
 */
class ModuleAbs
{
 public:
  /**
   * @brief Construct a new Module object
   *
   * @param type module's type
   */
  explicit ModuleAbs(const ModuleType type);

  /**
   * @brief Construct a new Module object
   *
   * @param type module's type
   * @param name module's name
   */
  ModuleAbs(const ModuleType type, const std::string& name);

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

  // /**
  //  * @brief prtints info to screen
  //  *
  //  */
  // virtual void PrintOnScreen() = 0;
 private:
  ModuleType type_{ModuleType::UNDEFINED};
  std::string name_;
};

}  // namespace core::utils
#endif  // CORE_UTILS_MODULE_HPP