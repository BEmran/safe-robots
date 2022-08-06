
#include <sys/stat.h>

#include <experimental/filesystem>

#include "core/utils/exception.hpp"

namespace core
{
namespace utils
{
namespace fs = std::experimental::filesystem;

void create_directories(const std::string& full_path)
{
  fs::path path{full_path};
  try
  {
    fs::create_directories(path);
  }
  catch (fs::filesystem_error& e)
  {
    throw Exception(e.what());
  }
}

bool is_path_exists(const std::string& path)
{
  struct stat buffer
  {
  };
  return stat(path.c_str(), &buffer) == 0;
}

}  // namespace utils
}  // namespace core
