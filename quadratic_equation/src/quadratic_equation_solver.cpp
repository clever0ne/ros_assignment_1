#include "quadratic_equation/quadratic_equation_solver.h"

#include <std_msgs/Float32MultiArray.h>

namespace quadratic_equation
{

QuadraticEquationSolver::QuadraticEquationSolver(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Launch Quadratic Equation Solver.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>(topic_name_,
                                                                   static_cast<uint32_t>(queue_size_));
  service_server_ = node_handle_.advertiseService(service_name_,
                                                  &QuadraticEquationSolver::solve, this);

  ROS_INFO("Quadratic Equation Solver has been successfully launched.");
}

void QuadraticEquationSolver::start()
{
  ROS_INFO("Ready to solve quadratic equation.");
  ros::spin();
}

bool QuadraticEquationSolver::readParameters()
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

bool QuadraticEquationSolver::solve(quadratic_equation_msgs::Solve::Request  &request,
                                    quadratic_equation_msgs::Solve::Response &response)
{
  auto a = request.a;
  auto b = request.b;
  auto c = request.c;
  printEquation(a, b, c);

  auto roots = findRoots(a, b, c);
  printRoots(roots);

  auto real_roots = std::vector<float>();
  if (roots.first == roots.second)
  {
    real_roots.emplace_back(roots.first.real());
  }
  else if (roots.first.imag() == 0.0f && roots.second.imag() == 0.0f)
  {
    real_roots.emplace_back(roots.first.real());
    real_roots.emplace_back(roots.second.real());
  }

  auto message = std_msgs::Float32MultiArray();
  message.data = real_roots;
  response.roots = real_roots;

  publisher_.publish(message);

  return true;
}

template<typename T>
void QuadraticEquationSolver::printEquation(const T a, const T b, const T c)
{
  auto sign_a = static_cast<double>(a) < 0.0 ? '-' : ' ';
  auto sign_b = static_cast<double>(b) < 0.0 ? '-' : '+';
  auto sign_c = static_cast<double>(c) < 0.0 ? '-' : '+';
  auto abs_a = abs(static_cast<double>(a));
  auto abs_b = abs(static_cast<double>(b));
  auto abs_c = abs(static_cast<double>(c));
  ROS_INFO("equation:\n\t\t\t\t\t %c%.4fx^2 %c %.4fx %c %.4f = 0",
           sign_a, abs_a, sign_b, abs_b, sign_c, abs_c);
}

template<typename T>
void QuadraticEquationSolver::printRoots(const std::pair<std::complex<T>, std::complex<T>> &roots)
{
  // Проверки для более информативного вывода в терминал сервиса
  if (std::isnan(roots.first.real()) == true || std::isnan(roots.first.imag()) == true)
  {
    throw std::runtime_error("The first root is not a number (NaN).");
  }

  if (std::isinf(roots.first.real()) == true || std::isinf(roots.first.imag()) == true)
  {
    throw std::runtime_error("The first root is infinite (inf).");
  }

  if (std::isnan(roots.second.real()) == true || std::isnan(roots.second.imag()) == true)
  {
    throw std::runtime_error("The second root is not a number (NaN).");
  }

  if (std::isinf(roots.second.real()) == true || std::isinf(roots.second.imag()) == true)
  {
    throw std::runtime_error("The second root is infinite (inf).");
  }

  // Дополнительный отладочный вывод комплексных корней в терминал узла
  auto sign_first_root_real  = static_cast<double>(roots.first.real())  < 0 ? '-' : ' ';
  auto sign_first_root_imag  = static_cast<double>(roots.first.imag())  < 0 ? '-' : '+';
  auto sign_second_root_real = static_cast<double>(roots.second.real()) < 0 ? '-' : ' ';
  auto sign_second_root_imag = static_cast<double>(roots.second.imag()) < 0 ? '-' : '+';
  auto abs_first_root_real  = abs(static_cast<double>(roots.first.real()));
  auto abs_first_root_imag  = abs(static_cast<double>(roots.first.imag()));
  auto abs_second_root_real = abs(static_cast<double>(roots.first.real()));
  auto abs_second_root_imag = abs(static_cast<double>(roots.first.imag()));
  ROS_INFO("roots:\n\t\t\t\t\t %c%.4f %c %.4fi\n\t\t\t\t\t %c%.4f %c %.4fi",
           sign_first_root_real,  abs_first_root_real,  sign_first_root_imag,  abs_first_root_imag,
           sign_second_root_real, abs_second_root_real, sign_second_root_imag, abs_second_root_imag);
}

template<typename T>
std::pair<std::complex<T>, std::complex<T>> QuadraticEquationSolver::findRoots(const T a, const T b, const T c)
{
  // Проверка нулевых a и b
  static constexpr auto EPSILON = 1e-9;
  if (abs(a) < static_cast<T>(EPSILON))
  {
    if (abs(b) < static_cast<T>(EPSILON))
    {
      auto root = std::complex<T>(static_cast<T>(NAN));
      return std::make_pair(root, root);
    }

    auto root = std::complex<T>(c / b);
    return std::make_pair(root, root);
  }

  // Общий случай с комплексными корнями
  auto squared_descriminant = std::complex<T>(b * b - 4 * a * c);
  std::complex<T> descriminant = sqrt(squared_descriminant);

  auto first_root  = (-b - descriminant) / (2 * a);
  auto second_root = (-b + descriminant) / (2 * a);

  return std::make_pair(first_root, second_root);
}

} /* namespace quadratic_equation */
