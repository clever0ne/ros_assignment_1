#include "squid_game/squid_game.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squid_game");
  auto node_handle = ros::NodeHandle("squid_game");

  auto squid_game = squid_game::SquidGame(node_handle);
  squid_game.start();

  return 0;
}
