#include "quadratic_equation/quadratic_equation.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadratic_equation");
  auto node_handle = ros::NodeHandle("quadratic_equation");

  auto quadratic_equation = quadratic_equation::QuadraticEquation(node_handle);
  quadratic_equation.start();

  return 0;
}
