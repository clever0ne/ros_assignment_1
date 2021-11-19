#pragma once

#ifndef SQUID_GAME_GLASS_BRIDGE_H
#define SQUID_GAME_GLASS_BRIDGE_H

#include <ros/ros.h>

#include <string>
#include <random>

#include "squid_game_msgs/Step.h"

namespace squid_game
{

class SquidGameGlassBridge
{
public:
  SquidGameGlassBridge() = delete;
  explicit SquidGameGlassBridge(ros::NodeHandle &node_handle);
  virtual ~SquidGameGlassBridge() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE   = "queue_size";
  const std::string PARAMETER_BRIDGE_SIZE  = "bridge_size";
  const std::string PARAMETER_TOPIC_NAME   = "topic_name";
  const std::string PARAMETER_SERVICE_NAME = "service_name";

  // Стороны стеклянного моста
  const std::string SIDE_LEFT  = "l";
  const std::string SIDE_RIGHT = "r";

  virtual void start();

private:
  bool readParameters();
  void generateRandomGlassBridge(std::string &glass_panels);
  void printPath(const std::string &name, const std::string &path);

  bool step(squid_game_msgs::Step::Request  &request,
            squid_game_msgs::Step::Response &response);

  int32_t queue_size_;
  int32_t bridge_size_;
  std::string topic_name_;
  std::string service_name_;
  std::string tempered_glass_panels_;
  std::mt19937 random_generator_;
  std::random_device random_device_;
  std::uniform_int_distribution<std::mt19937::result_type> distribution_;
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
  ros::ServiceServer service_server_;
};

} /* namespace squid_game */

#endif /* SQUID_GAME_GLASS_BRIDGE_H */
