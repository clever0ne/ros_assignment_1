#include "matrix_determinant/matrix_determinant_calculator.h"

#include <std_msgs/Float32.h>

#include "matrix_determinant_msgs/Calculate.h"

namespace matrix_determinant
{

MatrixDeterminantCalculator::MatrixDeterminantCalculator(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Launch Matrix Determinant Calculator.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  publisher_ = node_handle_.advertise<std_msgs::Float32>(topic_name_,
                                                         static_cast<uint32_t>(queue_size_));
  service_server_ = node_handle_.advertiseService(service_name_,
                                                  &MatrixDeterminantCalculator::calculate, this);

  ROS_INFO("Matrix Determinant Calculator has been successfully launched.");
}

bool MatrixDeterminantCalculator::readParameters()
{
  if (node_handle_.getParam(PARAMETER_QUEUE_SIZE, queue_size_) != true)
  {
    return false;
  }

  if (node_handle_.getParam(PARAMETER_TOPIC_NAME, topic_name_) != true)
  {
    return false;
  }

  if (node_handle_.getParam(PARAMETER_SERVICE_NAME, service_name_) != true)
  {
    return false;
  }

  return true;
}

void MatrixDeterminantCalculator::start()
{
  ROS_INFO("Ready to calculate matrix determinant.");
  ros::spin();
}

bool MatrixDeterminantCalculator::calculate(matrix_determinant_msgs::Calculate::Request  &request,
                                            matrix_determinant_msgs::Calculate::Response &response)
{
  static constexpr int WIDTH = 10;
  boost::qvm::mat<int32_t, 3, 3> matrix =
  {
    {
      { request.m11, request.m12, request.m13 },
      { request.m21, request.m22, request.m23 },
      { request.m31, request.m32, request.m33 }
    }
  };

  printMatrix(matrix);

  auto determinant = boost::qvm::determinant(matrix);
  response.detm = static_cast<float>(determinant);

  auto message = std_msgs::Float32();
  message.data = static_cast<float>(determinant);

  publisher_.publish(message);
  ROS_INFO("determinant:\n\t\t\t\t\t  % *lli", WIDTH, static_cast<signed long long int>(determinant));

  return true;
}

template<typename T, int Rows, int Cols>
void MatrixDeterminantCalculator::printMatrix(const boost::qvm::mat<T, Rows, Cols> &matrix) const
{
  auto matrix_string = std::string("matrix:\t");
  for (size_t row = 0; row < Rows; row++)
  {
    matrix_string += "\n\t\t\t\t";
    for (size_t col = 0; col < Cols; col++)
    {
      static constexpr int WIDTH = 10;
      char element[WIDTH + 1];
      sprintf(element, "% *.0f", WIDTH, static_cast<double>(matrix.a[row][col]));
      matrix_string += element;
    }
  }
  ROS_INFO("%s", matrix_string.c_str());
}

} // namespace matrix_determinant
