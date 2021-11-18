#include "squid_game/squid_game_player.h"

namespace squid_game
{

SquidGamePlayer::SquidGamePlayer(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Launch Squid Game Player.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::shutdown();
  }
  subscriber_ = node_handle_.subscribe(topic_name_, static_cast<uint32_t>(queue_size_),
                                       &SquidGamePlayer::move, this);

  ROS_INFO("Squid Game Player has been successfully launched.");
}

bool SquidGamePlayer::readParameters()
{
  if (node_handle_.getParam(PARAMETER_QUEUE_SIZE, queue_size_) != true)
  {
    return false;
  }

  if (node_handle_.getParam(PARAMETER_BRIDGE_SIZE, bridge_size_) != true)
  {
    return false;
  }

  if (node_handle_.getParam(PARAMETER_TOPIC_NAME, topic_name_) != true)
  {
    return false;
  }

  return true;
}

void SquidGamePlayer::move(const std_msgs::String::ConstPtr &message)
{
  static std::string path;
  path += message->data;

  if (path.size() < static_cast<size_t>(bridge_size_))
  {
    // Предыдущие выборы игрока
    ROS_INFO("%s", path.c_str());
  }
  else
  {
    // Победа игрока
    ROS_INFO("congratulation, you won");
    ros::requestShutdown();
  }
}

} /* namespace squid_game */
