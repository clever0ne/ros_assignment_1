#include "matrix_determinant/matrix_determinant.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "matrix_determinant");
  auto node_handle = ros::NodeHandle("matrix_determinant");

  auto martix_determinant = matrix_determinant::MatrixDeterminant(node_handle);
  martix_determinant.start();

  return 0;
}
