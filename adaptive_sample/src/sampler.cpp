#include "sampler.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;

Sampler::Sampler() : Node("orchestrator"), 
count_(0) {
  id = this->declare_parameter<int>("id",999);
  std::string sample_topic = "/robot" + std::to_string(id) + "/data";
  std::string log_topic = "/robot" + std::to_string(id) + "/log";
  sample_pub = this->create_publisher<SampleReturnArray>(sample_topic, 10);
  publisher_ = this->create_publisher<std_msgs::msg::String>(log_topic, 10);
}

void Sampler::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void Sampler::upload_data(std::vector<SampleReturn> samples) {
    int x = 0;
}