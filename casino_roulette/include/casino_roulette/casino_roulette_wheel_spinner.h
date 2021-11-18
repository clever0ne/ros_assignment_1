#pragma once

#ifndef CASINO_ROULETTE_WHEEL_SPINNER_H
#define CASINO_ROULETTE_WHEEL_SPINNER_H

#include <ros/ros.h>

#include <string>
#include <random>

#include "casino_roulette_msgs/Spin.h"

namespace casino_roulette
{

class CasinoRouletteWheelSpinner
{
public:
  CasinoRouletteWheelSpinner() = delete;
  explicit CasinoRouletteWheelSpinner(ros::NodeHandle &node_handle);
  virtual ~CasinoRouletteWheelSpinner() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE   = "queue_size";
  const std::string PARAMETER_TOPIC_NAME   = "topic_name";
  const std::string PARAMETER_SERVICE_NAME = "service_name";

  // Цвета ячеек на столе европейской рулетки
  const std::string COLOR_RED   = "red";
  const std::string COLOR_BLACK = "black";
  const std::string COLOR_GREEN = "green";

  // Диапазон номеров ячеек на столе европейской рулетки
  const uint32_t MIN_NUMBER = 0;
  const uint32_t MAX_NUMBER = 36;

  // Ассоциативный массив чисел и цветов ячеек на столе европейской рулетки
  const std::map<uint32_t, std::string> COLOR_MAP =
  {
                         { 0,  COLOR_GREEN },
    { 1,  COLOR_RED   }, { 2,  COLOR_BLACK }, { 3,  COLOR_RED   },
    { 4,  COLOR_BLACK }, { 5,  COLOR_RED   }, { 6,  COLOR_BLACK },
    { 7,  COLOR_RED   }, { 8,  COLOR_BLACK }, { 9,  COLOR_RED   },
    { 10, COLOR_BLACK }, { 11, COLOR_BLACK }, { 12, COLOR_RED   },
    { 13, COLOR_BLACK }, { 14, COLOR_RED   }, { 15, COLOR_BLACK },
    { 16, COLOR_RED   }, { 17, COLOR_BLACK }, { 18, COLOR_RED   },
    { 19, COLOR_RED   }, { 20, COLOR_BLACK }, { 21, COLOR_RED   },
    { 22, COLOR_BLACK }, { 23, COLOR_RED   }, { 24, COLOR_BLACK },
    { 25, COLOR_RED   }, { 26, COLOR_BLACK }, { 27, COLOR_RED   },
    { 28, COLOR_BLACK }, { 29, COLOR_BLACK }, { 30, COLOR_RED   },
    { 31, COLOR_BLACK }, { 32, COLOR_RED   }, { 33, COLOR_BLACK },
    { 34, COLOR_RED   }, { 35, COLOR_BLACK }, { 36, COLOR_RED   },
  };

  virtual void start();

private:
  bool readParameters();
  bool spin(casino_roulette_msgs::Spin::Request  &request,
            casino_roulette_msgs::Spin::Response &response);

  int32_t queue_size_;
  std::string topic_name_;
  std::string service_name_;
  std::random_device random_device_;
  std::mt19937 random_generator_;
  std::uniform_int_distribution<std::mt19937::result_type> distribution_;
  ros::NodeHandle &node_handle_;
  ros::Publisher publisher_;
  ros::ServiceServer service_server_;
};

} /* namespace casino_roulette */

#endif /* CASINO_ROULETTE_WHEEL_SPINNER_H */
