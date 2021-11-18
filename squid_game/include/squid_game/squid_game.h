#pragma once

#ifndef GLASS_BRIDGE_H
#define GLASS_BRIDGE_H

#include <ros/ros.h>

namespace squid_game
{

class SquidGameGlassBridge;
class SquidGamePlayer;

class SquidGame
{
public:
  SquidGame() = delete;
  explicit SquidGame(ros::NodeHandle &node_handle);
  virtual ~SquidGame();

  virtual void start();

private:
  ros::NodeHandle &node_handle_;
  SquidGameGlassBridge *glass_bridge_;
  SquidGamePlayer *player_;
};

} /* namespace squid_game */

#endif /* SQUID_GAME_H */
