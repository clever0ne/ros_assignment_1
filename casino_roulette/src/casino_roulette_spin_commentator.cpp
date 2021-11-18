#include "casino_roulette/casino_roulette_spin_commentator.h"

#include <chrono>

namespace casino_roulette
{

CasinoRouletteSpinCommentator::CasinoRouletteSpinCommentator(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Launch Casino Roulette Wheel Spinner.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  // Настройка генератора псевдослучайных числе (ГПСЧ)
  auto seed = random_device_() ^ (static_cast<std::mt19937::result_type>
                                 (std::chrono::duration_cast<std::chrono::seconds>
                                 (std::chrono::system_clock::now().time_since_epoch()).count()) +
                                 (static_cast<std::mt19937::result_type>
                                 (std::chrono::duration_cast<std::chrono::microseconds>
                                 (std::chrono::high_resolution_clock::now().time_since_epoch()).count())));

  random_generator_ = std::mt19937(seed);
  win_comments_distribution_  = std::uniform_int_distribution<std::mt19937::result_type>(0, WIN_COMMENTS.size() - 1);
  lose_comments_distribution_ = std::uniform_int_distribution<std::mt19937::result_type>(0, LOSE_COMMENTS.size() - 1);

  subscriber_ = node_handle_.subscribe(topic_name_, static_cast<uint32_t>(queue_size_),
                                      &CasinoRouletteSpinCommentator::comment, this);

  ROS_INFO("Casino Roulette Wheel Spinner has been successfully launched.");
}

bool CasinoRouletteSpinCommentator::readParameters()
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

void CasinoRouletteSpinCommentator::comment(const std_msgs::String::ConstPtr &message)
{
  auto result = message->data;
  if (result == RESULT_WIN)
  {
    auto comment_number = static_cast<uint32_t>(win_comments_distribution_(random_generator_));
    ROS_INFO("%s", WIN_COMMENTS.at(comment_number).c_str());
  }
  if (result == RESULT_LOSE)
  {
    auto comment_number = static_cast<uint32_t>(lose_comments_distribution_(random_generator_));
    ROS_INFO("%s", LOSE_COMMENTS.at(comment_number).c_str());
  }
}

} /* namespace casino_roulette */
