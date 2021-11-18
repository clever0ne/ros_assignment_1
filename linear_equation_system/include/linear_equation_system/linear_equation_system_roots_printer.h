#pragma once

#ifndef LINEAR_EQUATION_SYSTEM_ROOTS_PRINTER_H
#define LINEAR_EQUATION_SYSTEM_ROOTS_PRINTER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace linear_equation_system
{

class LinearEquationSystemRootsPrinter
{
public:
  LinearEquationSystemRootsPrinter() = delete;
  explicit LinearEquationSystemRootsPrinter(ros::NodeHandle &node_handle);
  virtual ~LinearEquationSystemRootsPrinter() = default;

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

} /* namespace linear_equation_system */

#endif /* LINEAR_EQUATION_SYSTEM_ROOTS_PRINTER_H */
