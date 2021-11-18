#pragma once

#ifndef SQUID_GAME_PLAYER_H
#define SQUID_GAME_PLAYER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

namespace squid_game
{

class SquidGamePlayer
{
public:
  SquidGamePlayer() = delete;
  explicit SquidGamePlayer(ros::NodeHandle &node_handle);
  virtual ~SquidGamePlayer() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE  = "queue_size";
  const std::string PARAMETER_BRIDGE_SIZE = "bridge_size";
  const std::string PARAMETER_TOPIC_NAME  = "topic_name";

private:
  bool readParameters();
  void move(const std_msgs::String::ConstPtr &message);

  int32_t queue_size_;
  int32_t bridge_size_;
  std::string topic_name_;
  ros::NodeHandle &node_handle_;
  ros::Subscriber subscriber_;
};

} /* namespace squid_game */

#endif /* SQUID_GAME_PLAYER_H */
