#include <cstdio>
#include "../include/ros2-marvelmind/marvelmindROSWrapper.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("marvelmind");

  RCLCPP_INFO(node->get_logger(),"Started Marvelmind node");

  marvelmindWrapper marvelmindWrapper(node);

  if (marvelmindWrapper.connect()){
    rclcpp::spin(node);
  }
  else RCLCPP_ERROR(node->get_logger(),"Connection to Marvelmind failed");

  marvelmindWrapper.disconnect();

  RCLCPP_INFO(node->get_logger(),"Exiting ...");
  rclcpp::shutdown();
//

  return 0;
}
