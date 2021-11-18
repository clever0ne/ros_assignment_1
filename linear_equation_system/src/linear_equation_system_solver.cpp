#include "linear_equation_system/linear_equation_system_solver.h"

#include <array>

#include <std_msgs/Float32MultiArray.h>

namespace linear_equation_system
{

LinearEquationSystemSolver::LinearEquationSystemSolver(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Linear Equation System Solver.");

  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>(topic_name_,
                                                                   static_cast<uint32_t>(queue_size_));
  service_server_ = node_handle_.advertiseService(service_name_,
                                                  &LinearEquationSystemSolver::solve, this);

  ROS_INFO("Linear Equation System Solver has been successfully launched.");
}

void LinearEquationSystemSolver::start()
{
  ROS_INFO("Ready to solve linear equation system.");
  ros::spin();
}

bool LinearEquationSystemSolver::readParameters()
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

bool LinearEquationSystemSolver::solve(linear_equation_system_msgs::Solve::Request  &request,
                                       linear_equation_system_msgs::Solve::Response &response)
{
  auto a = request.a;
  auto b = request.b;
  auto c = request.c;
  auto d = request.d;
  auto e = request.e;
  auto f = request.f;
  printEquationSystem(a, b, c, d, e, f);

  boost::numeric::ublas::matrix<float> matrix(2, 2);
  boost::numeric::ublas::vector<float> vector(2);

  matrix <<= a, b, d, e;
  vector <<= c, f;

  auto roots = findRoots(matrix, vector);
  printRoots(roots);

  auto message = std_msgs::Float32MultiArray();
  message.data = roots;
  response.roots = roots;

  publisher_.publish(message);

  return true;
}

template<typename T>
void LinearEquationSystemSolver::printEquationSystem(const T a, const T b, const T c,
                                                     const T d, const T e, const T f)
{
  auto sign_a = static_cast<double>(a) < 0.0 ? '-' : ' ';
  auto sign_b = static_cast<double>(b) < 0.0 ? '-' : '+';
  auto sign_c = static_cast<double>(c) < 0.0 ? '-' : ' ';
  auto sign_d = static_cast<double>(d) < 0.0 ? '-' : ' ';
  auto sign_e = static_cast<double>(e) < 0.0 ? '-' : '+';
  auto sign_f = static_cast<double>(f) < 0.0 ? '-' : ' ';
  auto abs_a = abs(static_cast<double>(a));
  auto abs_b = abs(static_cast<double>(b));
  auto abs_c = abs(static_cast<double>(c));
  auto abs_d = abs(static_cast<double>(d));
  auto abs_e = abs(static_cast<double>(e));
  auto abs_f = abs(static_cast<double>(f));
  ROS_INFO("equation system:\n\t\t\t\t\t\t %c%.4fx %c %.4fy = %c%.4f\n\t\t\t\t\t\t %c%.4fx %c %.4fy = %c%.4f",
           sign_a, abs_a, sign_b, abs_b, sign_c, abs_c, sign_d, abs_d, sign_e, abs_e, sign_f, abs_f);
}

template<typename T>
void LinearEquationSystemSolver::printRoots(const std::vector<T> &roots)
{
  if (roots.empty())
  {
    return;
  }

  ROS_INFO("roots:");
  for (const auto &root : roots)
  {
    auto sign = static_cast<double>(root) < 0.0 ? '-' : ' ';
    printf("\t\t\t\t\t\t %c%.4f\n", sign, abs(static_cast<double>(root)));
  }
}

template<typename T>
std::vector<T> LinearEquationSystemSolver::findRoots(boost::numeric::ublas::matrix<T> &matrix,
                                                     boost::numeric::ublas::vector<T> &vector)
{
  std::vector<T> roots;
  boost::numeric::ublas::permutation_matrix<float> permut_matrix(std::min(matrix.size1(), matrix.size2()));

  auto is_singular = boost::numeric::ublas::lu_factorize(matrix, permut_matrix);

  if (is_singular != 0)
  {
    ROS_ERROR("The system is singular and has no roots.");
    return roots;
  }

  boost::numeric::ublas::lu_substitute(matrix, permut_matrix, vector);
  for (const auto &root : vector)
  {
    roots.emplace_back(static_cast<float>(root));
  }

  return roots;
}

} /* namespace linear_equation_system */
