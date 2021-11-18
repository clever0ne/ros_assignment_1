#include "casino_roulette/casino_roulette_wheel_spinner.h"

#include <chrono>
#include <std_msgs/String.h>

namespace casino_roulette
{

CasinoRouletteWheelSpinner::CasinoRouletteWheelSpinner(ros::NodeHandle &node_handle)
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
  distribution_ = std::uniform_int_distribution<std::mt19937::result_type>(MIN_NUMBER, MAX_NUMBER);
  /*
   *  По этому случаю анекдот:
   *  Заходят как-то программисты на Python, C# и C++ в бар.
   *
   *  Программист на Python говорит:
   *  - Мне бокал пива.
   *  * Выпил, оплатил, ушёл *
   *
   *  Программист на C#:
   *  - Здравствуйте! Налейте мне, пожалуйста, пенного хмельного пива в стеклянную тару,
   *  в простонародии называемую бокалом.
   *  * Минут 30 сидел, пил, оплатил, со всеми в баре попрощался и ушёл *
   *
   *  Программист на C++ три часа не мог открыть рот.
   */

  publisher_ = node_handle_.advertise<std_msgs::String>(topic_name_,
                                                        static_cast<uint32_t>(queue_size_));
  service_server_ = node_handle_.advertiseService("spin",
                                                  &CasinoRouletteWheelSpinner::spin, this);

  ROS_INFO("Casino Roulette Wheel Spinner has been successfully launched.");
}

void CasinoRouletteWheelSpinner::start()
{
  ROS_INFO("Ready to spin roulette wheel.");
  ros::spin();
}

bool CasinoRouletteWheelSpinner::readParameters()
{
  if (node_handle_.getParam(PARAMETER_QUEUE_SIZE, queue_size_) != true)
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

bool CasinoRouletteWheelSpinner::spin(casino_roulette_msgs::Spin::Request  &request,
                                      casino_roulette_msgs::Spin::Response &response)
{
  auto color = request.color;
  auto number = request.number;
  ROS_INFO("your bet:\n\t\t\t\t\t     %s %u", color.c_str(), number);

  // Недопустимый цвет
  if (color != COLOR_RED && color != COLOR_BLACK && color != COLOR_GREEN)
  {
    throw std::runtime_error("Unacceptable color.");
  }

  // Недопустимое число
  if (number < MIN_NUMBER || number > MAX_NUMBER)
  {
    throw std::runtime_error("Unacceptable number.");
  }

  // Недопустимое сочетание цвета и числа
  if (COLOR_MAP.at(number) != color)
  {
    throw std::runtime_error("Unacceptable combination of color and number.");
  }

  // Случайное число из равномерного распределение
  auto random_number = static_cast<uint32_t>(distribution_(random_generator_));
  ROS_INFO("winning bet:\n\t\t\t\t\t     %s %u", COLOR_MAP.at(random_number).c_str(), random_number);

  // Проверка результата
  std::string result;
  if (random_number == number)
  {
    result = "win";
  }
  else
  {
    result = "lose";
  }

  auto message = std_msgs::String();
  message.data = result;

  response.result = result;
  publisher_.publish(message);

  return true;
}

} /* namespace casino_roulette */
