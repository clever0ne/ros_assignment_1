#include "matrix_determinant/matrix_determinant_parity_checker.h"

namespace matrix_determinant
{

MatrixDeterminantParityChecker::MatrixDeterminantParityChecker(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Launch Matrix Determinant Parity Checker.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  subscriber_ = node_handle_.subscribe(topic_name_, static_cast<uint32_t>(queue_size_),
                                       &MatrixDeterminantParityChecker::checkParity, this);

  ROS_INFO("Matrix Determinant Parity Checker has been successfully launched.");
  ros::spinOnce();
}

bool MatrixDeterminantParityChecker::readParameters()
{
  if (node_handle_.getParam(PARAMETER_QUEUE_SIZE, queue_size_) != true)
  {
    return false;
  }

  if (node_handle_.getParam(PARAMETER_TOPIC_NAME, topic_name_) != true)
  {
    return false;
  }

  return true;
}

void MatrixDeterminantParityChecker::checkParity(const std_msgs::Float32::ConstPtr &message)
{
  if (static_cast<int32_t>(message->data) % 2 == 0)
  {
    ROS_INFO("even");
  }
  else
  {
    ROS_INFO("odd");
  }
}

} /* namespace matrix_determinant */
