
#ifndef CORE_UTILS_COMMON_HPP
#define CORE_UTILS_COMMON_HPP

#include <memory>

#include "core/utils/terminal.hpp"

namespace core
{
namespace utils
{
/**
 * @brief Defines the level of an event to be deal with
 *
 */
struct EventLevel
{
  typedef enum
  {
    EL_INFO = 0,  // basic level of information
    EL_DEBUG,     // only for information used in debug mode
    EL_WARN,      // when system behaviour is not desirable but not fatal
    EL_ERROR,     // when system is fatal and system should treat it
  } event_level_t;
};

class LabeledModifier
{
 public:
  explicit LabeledModifier(const EventLevel::event_level_t event);

  LabeledModifier(const EventLevel::event_level_t event,
                  const std::string& label);

  LabeledModifier(const EventLevel::event_level_t event,
                  const Modifier& modifier);

  LabeledModifier(const EventLevel::event_level_t event,
                  const std::string& label, const Modifier& modifier);

  EventLevel::event_level_t get_event_level() const;

  /**
   * @brief Returns the Modifier associated to an EventLevel
   *
   * @param event event level
   * @return terminal::Modifier* a pointer to associated the modifier
   */
  Modifier get_modifier() const;

  /**
   * @brief Returns the Modifier associated to an EventLevel
   *
   * @param event event level
   * @return terminal::Modifier* a pointer to associated the modifier
   */
  std::string get_label() const;

 private:
  EventLevel::event_level_t event_;
  std::string label_;
  Modifier modifier_;
};

/**
 * @brief Returns the name of the EventLevel
 *
 * @param event event level
 * @return std::string name of the event
 */
std::string event_level_to_string(const EventLevel::event_level_t event);

/**
 * @brief override the << operator which writes the label using the modifier
 * configuration
 *
 * @param os output stream
 * @param lm labled modifer
 * @return std::ostream& reference to the output stredam
 */
std::ostream& operator<<(std::ostream& os, const LabeledModifier& lm);

LabeledModifier debug_labeled_modifier();

LabeledModifier error_labeled_modifier();

LabeledModifier info_labeled_modifier();

LabeledModifier warn_labeled_modifier();
}  // namespace utils
}  // namespace core

#endif  // CORE_UTILS_COMMON_HPP