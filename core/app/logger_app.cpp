#include "core/utils/node.hpp"

#include <iostream>

using core::utils::create_node;
using core::utils::Node;

int main()
{ 
    std::string name1 = "colorful";
    std::string name2 = "default";

    core::utils::Node node1 = create_node(name1);
    core::utils::Node node2(name2);

    node1.log_debug("this is a debug message");
    node1.log_info("this is a info message");
    node1.log_warn("this is a warn message");
    // node1.log_error("this is a error message");
    LOG_ERROR(node1, "this is a error message");
    node2.log_debug("this is a debug message");
    node2.log_info("this is a info message");
    node2.log_warn("this is a warn message");
    // node2.log_error("this is a error message");

    return 0;
}

