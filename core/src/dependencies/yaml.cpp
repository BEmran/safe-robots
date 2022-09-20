#include <dependencies/entree.hpp>
#include <dependencies/yaml.hpp>

namespace yaml {
struct Vec3 {
  double x, y, z; /* etc - make sure you have overloaded operator== */
};

template <>
struct convert<Vec3> {
  static Node encode(const Vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, Vec3& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    return true;
  }
};

// Map LoadFile(const std::string& filename) {
//   const YAML::Node node = YAML::LoadFile(filename);
//   return LoadNode(node);
// }

// Map LoadNode(const YAML::Node& node) {
//   Map map;
//   std::cout << "size: " << node.size() << std::endl;
//   for (auto it = node.begin(); it != node.end(); ++it) {
//     std::string key = it->first.as<std::string>();
//     std::string value;
//     std::cout << "\t- ";
//     if (node.Type() == YAML::NodeType::Sequence) {
//       std::cout << " [Sequence] ";
//       std::cout << std::quoted(key) << "\n";
//     }
//     if (node.Type() == YAML::NodeType::Map) {
//       std::cout << " [map] ";
//       try {
//         value = it->second.as<std::string>();
//         std::cout << std::quoted(key) << " is " << value << "\n";
//         map.insert({key, value});
//       } catch (...) {
//         try {
//           LoadNode(it->second.as<YAML::Node>());
//         } catch (...) {
//         }
//       }
//     }
//   }
//   return map;
// }

// Map LoadFile(const std::string& filename) {
//   const YAML::Node node = YAML::LoadFile(filename);
//   std::cout << "size: " << node.size() << std::endl;

//   for (auto it = node.begin(); it != node.end(); ++it) {
//     std::string value;
//     if (!it->second.IsNull()) {
//       std::cout << it->second.size() << "\t"
//                 << (it->second.IsMap() ? "Map" : "Not Map") << "\t"
//                 << (it->second.IsNull() ? "Null" : "Not Null") << "\t"
//                 << (it->second.IsScalar() ? "Scalar" : "Not Scalar") << "\t"
//                 << (it->second.IsDefined() ? "Defined" : "Not Defined")
//                 << std::endl;
//       if (it->second.IsScalar()) {
//         value = it->second.as<std::string>();
//       }
//     }
//     std::cout << it->first.as<std::string>() << ": " << value << std::endl;
//   }
//   Map map;
//   return map;
// }

// bool IsMap(YAML::Node node) {
//   return !node.IsNull() && !node.IsScalar() && node.IsMap();
// }

// bool IsScalar(YAML::Node node) {
//   return !node.IsNull() && node.IsScalar() && !node.IsMap();
// }

// bool IsList(YAML::Node node) {
//   return !node.IsNull() && !node.IsScalar() && !node.IsMap();
// }
template <typename Out>
void split(const std::string& s, char delim, Out result) {
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim)) {
    *result++ = item;
  }
}

std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, std::back_inserter(elems));
  return elems;
}

const char* ValueToString(YAML::NodeType::value value) {
  switch (value) {
    case (YAML::NodeType::value::Map):
      return "Map";
    case (YAML::NodeType::value::Undefined):
      return "Undefined";
    case (YAML::NodeType::value::Scalar):
      return "Scalar";
    case (YAML::NodeType::value::Sequence):
      return "Sequence";
    case (YAML::NodeType::value::Null):
      return "Null";
  }
}

void LoadNode(const YAML::Node& node) {
  std::cout << "size: " << node.size() << std::endl;
  for (auto it = node.begin(); it != node.end(); ++it) {
    auto n1 = it->first;
    if (it->first.IsNull()) {
      return;
    }

    auto n2 = it->second;
    auto t = ValueToString(n2.Type());
    std::string value;
    if (n2.IsScalar()) {
      value = n2.as<std::string>();
    } else if (n2.IsSequence()) {
      value = "VECTOR " + std::to_string(n2.size()) + ": ";
      for (size_t i = 0; i < n2.size(); ++i) {
        auto nn = n2[i];
        if (nn.IsScalar()) {
          // value += nn.Scalar();
          std::vector<std::string> vec = split(nn.Scalar(), ' ');
        } else if (nn.IsMap()) {
          LoadNode(nn);
        } else {
          std::cout << "ERRRRRROR" << std::endl;
        }
      }
    } else if (n2.IsMap()) {
      value = "MAP";
      LoadNode(n2);
    } else {
      value = "ELSE";
    }
    std::cout << n1.as<std::string>() << "[" << t << "]: " << value
              << std::endl;
  }
}
Map LoadFile(const std::string& filename) {
  const YAML::Node node = YAML::LoadFile(filename);
  LoadNode(node);

  Map map;
  return map;
}
// Map LoadNode(const YAML::Node& node) {
//   Map map;
//   for (auto it = node.begin(); it != node.end(); ++it) {
//     std::string key = it->first.as<std::string>();
//     std::string value;

//     // if (node.Type() == YAML::NodeType::Sequence) {
//     //   std::cout << " [Sequence] ";
//     //   std::cout << std::quoted(key) << "\n";
//     // } else
//     if (node.Type() == YAML::NodeType::Map) {
//       // std::cout << " [map] ";
//       try {
//         value = it->second.as<std::string>();
//         std::cout << std::quoted(key) << " is " << value << "\n";
//         map.insert({key, value});
//       } catch (...) {
//         try {
//           LoadNode(it->second.as<YAML::Node>());
//         } catch (...) {
//         }
//       }
//     }
//   }
//   return map;
// }

// Node* ExtractSingleNode(YAML::const_iterator it) {
//   std::string key = it->first.as<std::string>();
//   std::string value;
//   Node* node = nullptr;
//   if (it->second) {
//     value = it->second.as<std::string>();
//     auto a = it->second;
//     Node* node = new Single(key, EntreeType::STRING, value);
//   }
//   return node;
// }

// Node* ExtractListNode(YAML::const_iterator it) {
//   std::string key = it->first.as<std::string>();
//   std::string value;
//   Node* node = nullptr;
//   if (it->second) {
//     value = it->second.as<std::string>();
//     auto a = it->second;
//     Node* node = new Single(key, EntreeType::STRING, value);
//   }
//   return node;
// }

void DumpFile(const std::string& filename, Map map) {
  YAML::Node node;
  for (auto it = map.begin(); it != map.end(); ++it) {
    node[it->first] = it->second;
  }

  std::ofstream out(filename);
  out << node;
}
}  // namespace yaml