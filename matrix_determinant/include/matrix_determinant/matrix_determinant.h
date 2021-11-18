#pragma once

#ifndef MATRIX_DETERMINANT_H
#define MATRIX_DETERMINANT_H

#include <ros/ros.h>

namespace matrix_determinant
{

class MatrixDeterminantCalculator;
class MatrixDeterminantParityChecker;

class MatrixDeterminant
{
public:
  MatrixDeterminant() = delete;
  explicit MatrixDeterminant(ros::NodeHandle &node_handle);
  virtual ~MatrixDeterminant();

  virtual void start();

private:
  ros::NodeHandle &node_handle_;
  MatrixDeterminantCalculator *calculator_ = nullptr;
  MatrixDeterminantParityChecker *parity_checker_ = nullptr;
};

} /* namespace matrix_determinant */

#endif /* MATRIX_DETERMINANT_H */
