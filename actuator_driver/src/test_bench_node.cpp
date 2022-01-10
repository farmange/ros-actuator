// #include <memory>

#include "rclcpp/rclcpp.hpp"
#include "actuator_driver/rmd_comm.h"

class TestBenchExp : public rclcpp::Node
{
public:
  TestBenchExp() : Node("minimal_subscriber")
  {
    int8_t temperature = 0;
    uint32_t acquisition_period = 20000;  //[us]
    int8_t temperature_limit = 75;        //[°C]
    int16_t maxspeed = 1080, iq, speed;
    uint16_t encoder;
    int32_t zero_position = 0;
    int32_t desired_position_1 = 100500;
    int32_t desired_position_2 = -100500;
    int32_t current_position = 0;
    int32_t position_error_tolerance = 0;
    uint32_t max_accel = 0;
    uint32_t desired_iteration = 2;
    uint32_t current_iteration = 0;

    // rmd_comm_.init();

    // rmd_comm_.positionControlCommand2_(maxspeed, zero_position, temperature, iq, speed, encoder);
    printf("Wait for keyboard enter press to start experiment.\n");
    getchar();

    rclcpp::Rate loop_rate(10);
    rclcpp::Rate pause_rate(0.5);
    // while(rclcpp::ok())
    // {
    while (current_iteration < desired_iteration && temperature < temperature_limit)
    {
      int32_t desired_position = 0;

      // rclcpp::spin_some();
      rclcpp::spin_some(this->get_node_base_interface());

      RCLCPP_INFO(get_logger(), "Cycle N° %d / %d.\n", current_iteration + 1, desired_iteration);
      for (uint8_t i = 0; i < 2; i++)  // remplacer par un tableau à parcourir si plus de deux positions
      {
        if (i == 0)
        {
          desired_position = desired_position_1;
        }
        else
        {
          desired_position = desired_position_2;
        }

        // rmd_comm_.positionControlCommand2_(maxspeed, desired_position, temperature, iq, speed, encoder);

        while ((current_position > (desired_position + position_error_tolerance)) ||
               (current_position < (desired_position - position_error_tolerance)))
        {
          // Read motor multi turn angle
          int64_t dummy_angle = 0;
          // rmd_comm_.readMultiTurnsAngleCommand(dummy_angle);
          current_position = static_cast<int32_t>(dummy_angle);

          // std::vector <std::string> dataline(12, "");

          // rmd_comm_.readActuatorData(mc, desired_position, dataline, current_iteration);
          // csv.writeLog(dataline);

          // Wait 50ms
          // usleep(acquisition_period);
          // test_time = 100L;
          loop_rate.sleep();
        }

        pause_rate.sleep();

        // while(pause_currenttime < pause_starttime + pause_duration)
        // {
        //     std::vector <std::string> dataline(12, "");

        //     mc.readActuatorData(mc, desired_position, dataline, current_iteration);
        //     csv.writeLog(dataline);

        //     // Wait
        //     usleep(acquisition_period);
        //     pause_currenttime = getCurrentTime();
        // }

        RCLCPP_INFO(get_logger(), "Windings temperature = %d°C\n", temperature);
      }
      current_iteration++;
    }
    RCLCPP_INFO(get_logger(), "End of experiment, motor getting back to 0 position.\n");
    // rmd_comm_.positionControlCommand2_(maxspeed, 0.0, temperature, iq, speed, encoder);
    pause_rate.sleep();
    // rmd_comm_.motorOffCommand();

    RCLCPP_INFO(get_logger(), "in da loop");
    // }
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    // "topic", 10, std::bind(&TestBenchExp::topic_callback, this, _1));
  }

private:
  // void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  // }
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // RMDComm rmd_comm_;
  RMDComm rmd_comm_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::make_shared<TestBenchExp>();
  rclcpp::shutdown();
  return 0;
}
