#pragma once

#ifndef LINEAR_EQUATION_SYSTEM_SOLVER_H
#define LINEAR_EQUATION_SYSTEM_SOLVER_H

#include <ros/ros.h>
#include <string>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include "linear_equation_system_msgs/Solve.h"

namespace linear_equation_system
{

class LinearEquationSystemSolver
{
public:
  LinearEquationSystemSolver() = delete;
  explicit LinearEquationSystemSolver(ros::NodeHandle &node_handle);
  virtual ~LinearEquationSystemSolver() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE   = "queue_size";
  const std::string PARAMETER_TOPIC_NAME   = "topic_name";
  const std::string PARAMETER_SERVICE_NAME = "service_name";

  virtual void start();

private:
  bool readParameters();
  bool solve(linear_equation_system_msgs::Solve::Request  &request,
             linear_equation_system_msgs::Solve::Response &response);

  template<typename T>
  void printEquationSystem(const T a, const T b, const T c, const T d, const T e, const T f);

  template<typename T>
  void printRoots(const std::vector<T> &roots);

  template<typename T>
  std::vector<T> findRoots(boost::numeric::ublas::matrix<T> &matrix,
                           boost::numeric::ublas::vector<T> &vector);

  int32_t queue_size_;
  std::string topic_name_;
  std::string service_name_;
  ros::NodeHandle &node_handle_;
  ros::Publisher publisher_;
  ros::ServiceServer service_server_;
};

} /* namespace linear_equation_system */

#endif /* LINEAR_EQUATION_SYSTEM_SOLVER_H */
