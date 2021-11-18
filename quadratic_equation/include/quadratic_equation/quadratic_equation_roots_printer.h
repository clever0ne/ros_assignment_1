#pragma once

#ifndef QUADRATIC_EQUATION_ROOTS_PRINTER_H
#define QUADRATIC_EQUATION_ROOTS_PRINTER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace quadratic_equation
{

class QuadraticEquationRootsPrinter
{
public:
  QuadraticEquationRootsPrinter() = delete;
  explicit QuadraticEquationRootsPrinter(ros::NodeHandle &node_handle);
  virtual ~QuadraticEquationRootsPrinter() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE = "queue_size";
  const std::string PARAMETER_TOPIC_NAME = "topic_name";

private:
  void print(const std_msgs::Float32MultiArray::ConstPtr &message);
  bool readParameters();

  int32_t queue_size_;
  std::string topic_name_;
  ros::NodeHandle &node_handle_;
  ros::Subscriber subscriber_;
};

} /* namespace quadratic_equation */

#endif /* QUADRATIC_EQUATION_ROOTS_PRINTER_H */
