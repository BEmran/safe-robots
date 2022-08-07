#ifndef CORE_UTILS_WRITTER_FILE_HPP
#define CORE_UTILS_WRITTER_FILE_HPP

#include <fstream>
#include <memory>
#include <mutex>

#include "core/utils/writter.hpp"

namespace core::utils
{
/**
 * @brief A concreate class of Writter used to log data to a file using oftream
 * object.
 *
 */
class FileWritter : public Writter
{
 public:
  /**
   * @brief Construct a new File Logger object with a specific file name
   *
   * @param filename name of the file to be created
   *
   * TODO: create a robust way to create a file with date tag or without
   */
  explicit FileWritter(const std::string& filename);

  /**
   * @brief Destroy the File Writter object
   *
   */
  ~FileWritter();

  /* Writter Interface */
  void dump(const std::string& str) override;

 private:
  /**
   * @brief the actual function that logs a string to a file
   *
   * @param str string to be logged
   */
  void DumpToFile(const std::string& str);

  std::string filename_;
  std::shared_ptr<std::ofstream> file_;
  std::mutex dump_mutex_;
};

}  // namespace core::utils

#endif  // CORE_UTILS_WRITTER_FILE_HPP