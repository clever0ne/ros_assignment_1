#include "casino_roulette/casino_roulette.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "casino_roulette");
  auto node_handle = ros::NodeHandle("casino_roulette");

  auto casino_roulette = casino_roulette::CasinoRoulette(node_handle);
  casino_roulette.start();

  return 0;
}
