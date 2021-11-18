#include "quadratic_equation/quadratic_equation.h"
#include "quadratic_equation/quadratic_equation_solver.h"
#include "quadratic_equation/quadratic_equation_roots_printer.h"

namespace quadratic_equation
{

QuadraticEquation::QuadraticEquation(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
  , solver_(new QuadraticEquationSolver(node_handle))
  , roots_printer_(new QuadraticEquationRootsPrinter(node_handle))
{
  /* Здесь могла бы быть ваша реклама */
}

QuadraticEquation::~QuadraticEquation()
{
  delete solver_;
  delete roots_printer_;
}

void QuadraticEquation::start()
{
  solver_->start();
}

} /* namespace quadratic_equation */
