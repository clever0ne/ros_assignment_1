#include "matrix_determinant/matrix_determinant.h"
#include "matrix_determinant/matrix_determinant_calculator.h"
#include "matrix_determinant/matrix_determinant_parity_checker.h"

namespace matrix_determinant
{

MatrixDeterminant::MatrixDeterminant(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
  , calculator_(new MatrixDeterminantCalculator(node_handle))
  , parity_checker_(new MatrixDeterminantParityChecker(node_handle))
{
  /* Здесь могла бы быть ваша реклама */
}

MatrixDeterminant::~MatrixDeterminant()
{
  delete calculator_;
  delete parity_checker_;
}

void MatrixDeterminant::start()
{
  calculator_->start();
}

} // namespace matrix_determinant
