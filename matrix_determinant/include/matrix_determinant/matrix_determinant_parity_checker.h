#pragma once

#ifndef MATRIX_DETERMINANT_PARITY_CHECKER_H
#define MATRIX_DETERMINANT_PARITY_CHECKER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <string>

namespace matrix_determinant
{

class MatrixDeterminantParityChecker
{
public:
  MatrixDeterminantParityChecker() = delete;
  explicit MatrixDeterminantParityChecker(ros::NodeHandle &node_handle);
  virtual ~MatrixDeterminantParityChecker() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE = "queue_size";
  const std::string PARAMETER_TOPIC_NAME = "topic_name";

private:
  bool readParameters();
  void checkParity(const std_msgs::Float32::ConstPtr &message);

  int32_t queue_size_;
  std::string topic_name_;
  ros::NodeHandle &node_handle_;
  ros::Subscriber subscriber_;
};

} /* namespace matrix_determinant */

#endif /* MATRIX_DETERMINANT_PARITY_CHECKER_H */
