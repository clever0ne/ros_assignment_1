#include "squid_game/squid_game_glass_bridge.h"

#include <chrono>
#include <array>

#include <std_msgs/String.h>

namespace squid_game
{

SquidGameGlassBridge::SquidGameGlassBridge(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  ROS_INFO("Launch Squid Game Glass Bridge.");

  // Чтение параметров
  if (readParameters() != true)
  {
    ROS_ERROR("Failed to read the parameters.");
    ros::requestShutdown();
  }

  // Настройка генератора псевдослучайных числе (ГПСЧ)
  // Когда вас попросят как можно короче описать C++, покажите это
  auto seed = random_device_() ^ (static_cast<std::mt19937::result_type>
                                 (std::chrono::duration_cast<std::chrono::seconds>
                                 (std::chrono::system_clock::now().time_since_epoch()).count()) +
                                 (static_cast<std::mt19937::result_type>
                                 (std::chrono::duration_cast<std::chrono::microseconds>
                                 (std::chrono::high_resolution_clock::now().time_since_epoch()).count())));

  random_generator_ = std::mt19937(seed);
  distribution_ = std::uniform_int_distribution<std::mt19937::result_type>(0, 1);

  generateRandomGlassBridge(tempered_glass_panels_);

  publisher_ = node_handle_.advertise<std_msgs::String>(topic_name_,
                                                        static_cast<uint32_t>(queue_size_));
  service_server_ = node_handle_.advertiseService(service_name_,
                                                  &SquidGameGlassBridge::step, this);

  ROS_INFO("Squid Game Glass Bridge has been successfully launched.");
}

void SquidGameGlassBridge::start()
{
  ROS_INFO("I want to play a game with you.");
  printPath("generated path:", tempered_glass_panels_);
  ros::spin();
}

bool SquidGameGlassBridge::readParameters()
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

  if (node_handle_.getParam(PARAMETER_SERVICE_NAME, service_name_) != true)
  {
    return false;
  }

  return true;
}

void SquidGameGlassBridge::generateRandomGlassBridge(std::string &glass_panels)
{
  // Генерация случайной последовательности стеклянных плит
  for (auto panel_number = 0; panel_number < bridge_size_; panel_number++)
  {
    glass_panels += distribution_(random_generator_) ? SIDE_LEFT : SIDE_RIGHT;
  }
}

void SquidGameGlassBridge::printPath(const std::string &name, const std::string &path)
{
  // Отладочный вывод маршрута в терминал узла (и чтобы выиграть таки можно было)
  std::array<std::string, 5> strings { "\n\t\t\t\t\t +",
                                       "\n\t\t\t\t\t |",
                                       "\n\t\t\t\t\t +",
                                       "\n\t\t\t\t\t |",
                                       "\n\t\t\t\t\t +" };
  for (auto panel_number = 0; panel_number < bridge_size_; panel_number++)
  {
    strings.at(0) += "-+";
    strings.at(2) += "-+";
    strings.at(4) += "-+";

    strings.at(1) += path.size() > static_cast<size_t>(panel_number) &&
                     path.substr(static_cast<size_t>(panel_number), 1) == SIDE_LEFT  ? "*|" : " |";
    strings.at(3) += path.size() > static_cast<size_t>(panel_number) &&
                     path.substr(static_cast<size_t>(panel_number), 1) == SIDE_RIGHT ? "*|" : " |";

  }
  ROS_INFO("%s%s%s%s%s%s", name.c_str(),          strings.at(0).c_str(), strings.at(1).c_str(),
                           strings.at(2).c_str(), strings.at(3).c_str(), strings.at(4).c_str());
}

bool SquidGameGlassBridge::step(squid_game_msgs::Step::Request  &request,
                                squid_game_msgs::Step::Response &response)
{
  // Счётчик шагов и маршрут
  static size_t steps_counter = 0;
  static std::string path;

  // Обработка некорректного ввода
  auto side = request.side;
  if (side != SIDE_LEFT && side != SIDE_RIGHT)
  {
    throw std::runtime_error("You have to make a choise.");
  }

  // Вывод эталонного и текущего маршрутов
  path += side;
  printPath("generated path:", tempered_glass_panels_);
  printPath("player's path:", path);

  // Проверка выбранной стороны
  if (side == tempered_glass_panels_.substr(steps_counter, 1))
  {
    response.result = "move_next";
    steps_counter++;

    auto message = std_msgs::String();
    message.data = side;
    publisher_.publish(message);
  }
  else
  {
    // Поздравляю, вы умерли
    ROS_ERROR("You died.");
    response.result = "dead";
    ros::requestShutdown();
  }

  if (steps_counter >= static_cast<size_t>(bridge_size_))
  {
    // Поздравляю, вы выжили
    ROS_INFO("You survived.");
  }

  return true;
}

} /* namespace squid_game */
