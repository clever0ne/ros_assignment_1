#pragma once

#ifndef CASINO_ROULETTE_H
#define CASINO_ROULETTE_H

#include <ros/ros.h>

namespace casino_roulette
{

class CasinoRouletteWheelSpinner;
class CasinoRouletteSpinCommentator;

class CasinoRoulette
{
public:
  CasinoRoulette() = delete;
  explicit CasinoRoulette(ros::NodeHandle &node_handle);
  virtual ~CasinoRoulette();

  virtual void start();

private:
  ros::NodeHandle &node_handle_;
  CasinoRouletteWheelSpinner *wheel_spinner_ = nullptr;
  CasinoRouletteSpinCommentator *spin_commentator_ = nullptr;
};

} /* namespace casino_roulette */

#endif /* CASINO_ROULETTE_H */
