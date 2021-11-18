#include "quadratic_equation/quadratic_equation_roots_printer.h"

namespace quadratic_equation
{

QuadraticEquationRootsPrinter::QuadraticEquationRootsPrinter(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Launch Quadratic Equation Roots Printer.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  subscriber_ = node_handle_.subscribe(topic_name_, static_cast<uint32_t>(queue_size_),
                                       &QuadraticEquationRootsPrinter::print, this);

  ROS_INFO("Quadratic Equation Roots Printer has been successfully launched.");
}

void QuadraticEquationRootsPrinter::print(const std_msgs::Float32MultiArray::ConstPtr &message)
{
  // Случай положительного дискриминанта
  if (message->data.size() == 2)
  {
    // Хардкодим два знака после запятой, чтобы как в примере
    ROS_INFO("x1 = %.2f, x2 = %.2f", static_cast<double>(message->data.at(0)),
                                     static_cast<double>(message->data.at(1)));
  }

  // Случай нулевого дискриминанта или неквадратного уравнения
  if (message->data.size() == 1)
  {
    // Хардкодим два знака после запятой, чтобы как в примере
    ROS_INFO("x = %.2f", static_cast<double>(message->data.at(0)));
  }

  // Здесь мог бы быть метод empty
  if (message->data.size() == 0)
  {
    // Корни-то есть, вот только они комплексные
    ROS_INFO("no real roots");
  }
}

bool QuadraticEquationRootsPrinter::readParameters()
{
  if (node_handle_.getParam(PARAMETER_QUEUE_SIZE, queue_size_) != true)
  {
    return false;
  }

  if (node_handle_.getParam(PARAMETER_TOPIC_NAME, topic_name_) != true)
  {
    return false;
  }

  return true;
}

} /* namespace quadratic_equation */
