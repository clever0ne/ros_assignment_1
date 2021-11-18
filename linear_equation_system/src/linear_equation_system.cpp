#include "linear_equation_system/linear_equation_system.h"
#include "linear_equation_system/linear_equation_system_solver.h"
#include "linear_equation_system/linear_equation_system_roots_printer.h"

namespace linear_equation_system
{

LinearEquationSystem::LinearEquationSystem(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
  , solver_(new LinearEquationSystemSolver(node_handle))
  , roots_printer_(new LinearEquationSystemRootsPrinter(node_handle))
{
  /* Здесь могла бы быть ваша реклама */
}

LinearEquationSystem::~LinearEquationSystem()
{
  delete solver_;
  delete roots_printer_;
}

void LinearEquationSystem::start()
{
  solver_->start();
}

} /* namespace linear_equation_system */
