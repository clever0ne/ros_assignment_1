#pragma once

#ifndef MATRIX_DETERMINANT_CALCULATOR_H
#define MATRIX_DETERMINANT_CALCULATOR_H

#include <ros/ros.h>
#include <string>

#include <boost/qvm/mat.hpp>
#include <boost/qvm/mat_operations.hpp>

#include "matrix_determinant_msgs/Calculate.h"

namespace matrix_determinant
{

class MatrixDeterminantCalculator
{
public:
  MatrixDeterminantCalculator() = delete;
  explicit MatrixDeterminantCalculator(ros::NodeHandle &node_handle);
  virtual ~MatrixDeterminantCalculator() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE   = "queue_size";
  const std::string PARAMETER_TOPIC_NAME   = "topic_name";
  const std::string PARAMETER_SERVICE_NAME = "service_name";

  virtual void start();

private:
  bool readParameters();
  bool calculate(matrix_determinant_msgs::Calculate::Request  &request,
                 matrix_determinant_msgs::Calculate::Response &response);

  template<typename T, int Rows, int Cols>
  void printMatrix(const boost::qvm::mat<T, Rows, Cols> &matrix) const;

  int32_t queue_size_;
  std::string topic_name_;
  std::string service_name_;
  ros::NodeHandle &node_handle_;
  ros::Publisher publisher_;
  ros::ServiceServer service_server_;
};

} /* namespace matrix_determinant */

#endif /* MATRIX_DETERMINANT_CALCULATOR_H */
