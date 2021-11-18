#pragma once

#ifndef LINEAR_EQUATION_SYSTEM_H
#define LINEAR_EQUATION_SYSTEM_H

#include <ros/ros.h>

namespace linear_equation_system
{

class LinearEquationSystemSolver;
class LinearEquationSystemRootsPrinter;

class LinearEquationSystem
{
public:
  LinearEquationSystem() = delete;
  explicit LinearEquationSystem(ros::NodeHandle &node_handle);
  virtual ~LinearEquationSystem();

  virtual void start();
private:
  ros::NodeHandle &node_handle_;
  LinearEquationSystemSolver *solver_ = nullptr;
  LinearEquationSystemRootsPrinter *roots_printer_ = nullptr;
};

} /* namespace linear_equation_system */

#endif /*LINEAR_EQUATION_SYSTEM_H */
