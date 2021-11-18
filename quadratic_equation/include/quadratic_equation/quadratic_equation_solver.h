#pragma once

#ifndef QUADRATIC_EQUATION_SOLVER_H
#define QUADRATIC_EQUATION_SOLVER_H

#include <ros/ros.h>
#include <string>
#include <complex>

#include "quadratic_equation_msgs/Solve.h"

namespace quadratic_equation
{

class QuadraticEquationSolver
{
public:
  QuadraticEquationSolver() = delete;
  explicit QuadraticEquationSolver(ros::NodeHandle &node_handle);
  virtual ~QuadraticEquationSolver() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE   = "queue_size";
  const std::string PARAMETER_TOPIC_NAME   = "topic_name";
  const std::string PARAMETER_SERVICE_NAME = "service_name";

  virtual void start();

private:
  bool readParameters();
  bool solve(quadratic_equation_msgs::Solve::Request  &request,
             quadratic_equation_msgs::Solve::Response &response);

  // Шаблонные методы, которые всё кастают к double
  template<typename T>
  void printEquation(const T a, const T b, const T c);

  template<typename T>
  void printRoots(const std::pair<std::complex<T>, std::complex<T>> &roots);

  template<typename T>
  std::pair<std::complex<T>, std::complex<T>> findRoots(const T a, const T b, const T c);

  int32_t queue_size_;
  std::string topic_name_;
  std::string service_name_;
  ros::NodeHandle &node_handle_;
  ros::Publisher publisher_;
  ros::ServiceServer service_server_;
};

} /* namespace quadratic_equation */

#endif /* QUADRATIC_EQUATION_SOLVER_H */
