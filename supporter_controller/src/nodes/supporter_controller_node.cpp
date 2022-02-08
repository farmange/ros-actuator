//============================================================================
// Name        : support_controller_node.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "supporter_controller/supporter_controller.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<SupporterController> node = std::make_shared<SupporterController>("supporter_controller");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();

  return 0;
}