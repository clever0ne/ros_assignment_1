#pragma once

#ifndef QUADRATIC_EQUATION_H
#define QUADRATIC_EQUATION_H

#include <ros/ros.h>

namespace quadratic_equation
{

class QuadraticEquationSolver;
class QuadraticEquationRootsPrinter;

class QuadraticEquation
{
public:
  QuadraticEquation() = delete;
  explicit QuadraticEquation(ros::NodeHandle &node_handle);
  virtual ~QuadraticEquation();

  virtual void start();

private:
  ros::NodeHandle &node_handle_;
  QuadraticEquationSolver *solver_ = nullptr;
  QuadraticEquationRootsPrinter *roots_printer_ = nullptr;
};

} /* namespace quadratic_equation */

#endif /* QUADRATIC_EQUATION_H */
