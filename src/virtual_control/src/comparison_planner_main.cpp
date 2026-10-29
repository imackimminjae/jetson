#include "virtual_control/comparison_planner_ros.hpp"

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(imac_ctrl::comparison::makePlannerNode(COMPARISON_PLANNER_KIND));
  rclcpp::shutdown(); return 0;
}
