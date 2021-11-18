#pragma once

#ifndef CASINO_ROULETTE_SPIN_COMMENTATOR_H
#define CASINO_ROULETTE_SPIN_COMMENTATOR_H

#include <ros/ros.h>

#include <vector>
#include <string>
#include <random>

#include <std_msgs/String.h>

namespace casino_roulette
{

class CasinoRouletteSpinCommentator
{
public:
  CasinoRouletteSpinCommentator() = delete;
  explicit CasinoRouletteSpinCommentator(ros::NodeHandle &node_handle);
  virtual ~CasinoRouletteSpinCommentator() = default;

  // Имена параметров
  const std::string PARAMETER_QUEUE_SIZE = "queue_size";
  const std::string PARAMETER_TOPIC_NAME = "topic_name";

  // Результаты, приходящие в коллбэк
  const std::string RESULT_WIN = "win";
  const std::string RESULT_LOSE = "lose";

  // Комментарии по случаю выигрыша
  const std::vector<std::string> WIN_COMMENTS =
  {
    "Congratulations!",
    "What a lucky spin!",
    "You catch your luck!"
    "You're lucky one!"
  };

  // Комментарии по случаю проигрыша
  const std::vector<std::string> LOSE_COMMENTS =
  {
    "Don't be upset, next time you will definitely win!",
    "Place another bet, you're just one step away from winning!",
    "Try once more, you always will be able to stop!",
    "There are so many losses, but the next spin will be your win!"
  };

private:
  bool readParameters();
  void comment(const std_msgs::String::ConstPtr &message);

  int32_t queue_size_;
  std::string topic_name_;
  std::mt19937 random_generator_;
  std::uniform_int_distribution<std::mt19937::result_type> win_comments_distribution_;
  std::uniform_int_distribution<std::mt19937::result_type> lose_comments_distribution_;
  ros::NodeHandle &node_handle_;
  ros::Subscriber subscriber_;
  std::random_device random_device_;
};

} /* namespace casino_roulette */

#endif /* CASINO_ROULETTE_SPIN_COMMENTATOR_H */
