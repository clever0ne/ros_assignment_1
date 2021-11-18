#include "linear_equation_system/linear_equation_system_roots_printer.h"

namespace linear_equation_system
{

LinearEquationSystemRootsPrinter::LinearEquationSystemRootsPrinter(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Linear Equation System Roots Printer.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  subscriber_ = node_handle_.subscribe(topic_name_, static_cast<uint32_t>(queue_size_),
                                       &LinearEquationSystemRootsPrinter::print, this);

  ROS_INFO("Linear Equation System Roots Printer has been successfully launched.");
}

void LinearEquationSystemRootsPrinter::print(const std_msgs::Float32MultiArray::ConstPtr &message)
{
  if (message->data.empty() != true)
  {
    ROS_INFO("x = %.2f, y = %.2f", static_cast<double>(message->data.at(0)),
                                   static_cast<double>(message->data.at(1)));
  }
  else
  {
    ROS_INFO("no roots");
  }
}

bool LinearEquationSystemRootsPrinter::readParameters()
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

} /* namespace linear_equation_system */
