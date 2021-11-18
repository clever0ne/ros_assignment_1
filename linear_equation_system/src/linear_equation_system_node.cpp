#include "linear_equation_system/linear_equation_system.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_equation_system");
  auto node_handle = ros::NodeHandle("linear_equation_system");

  auto linear_equation_system = linear_equation_system::LinearEquationSystem(node_handle);
  linear_equation_system.start();

  return 0;
}
