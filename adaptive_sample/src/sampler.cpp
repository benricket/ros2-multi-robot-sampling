#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sampling_interfaces/msg/sample_return.hpp"
#include "sampling_interfaces/msg/sample_return_array.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;

class Sampler : public rclcpp::Node
{
  public:
    Sampler(int id_in) : Node("orchestrator"), 
    count_(0) {
      id = id_in;
      std::string sample_topic = "/robot" + std::to_string(id_in) + "/data";
      sample_pub = this->create_publisher<SampleReturnArray>(sample_topic, 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&Sampler::timer_callback, this));
    }

  private:
    void timer_callback() {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    void query_waypoint();
    void upload_data(std::vector<SampleReturn> samples);

    SampleReturn take_sample();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int id;

    geometry_msgs::msg::Pose pose;
    std::vector<SampleReturn> sample_vec;
    rclcpp::Publisher<SampleReturnArray>::SharedPtr sample_pub;



};

void Sampler::upload_data(std::vector<SampleReturn> samples) {
    int x = 0;
}