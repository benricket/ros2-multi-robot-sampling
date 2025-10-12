#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <GaussianProcess/GaussianProcess.h>
#include <GaussianProcess/kernel/SquaredExponential.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sampling_interfaces/msg/sample_return.hpp"
#include "sampling_interfaces/msg/sample_return_array.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;

class SamplerOrchestrator : public rclcpp::Node
{
  public:
    SamplerOrchestrator() : Node("orchestrator"), 
    count_(0),
    kernel_func(std::make_unique<gauss::gp::SquaredExponential>(1.0,1.0)),
    gauss_process(std::move(kernel_func),3,1) {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&SamplerOrchestrator::timer_callback, this));
    }

  private:
    void timer_callback() {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }



    void query_waypoint_callback(const std_msgs::msg::String& msg);
    void upload_data_callback(const SampleReturnArray& msg);

    std::vector<rclcpp::Subscription<SampleReturnArray>::SharedPtr> init_data_subs(int count);
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> init_waypoint_pubs(int count);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    // Parameters for sampling setup
    gauss::gp::KernelFunctionPtr kernel_func;
    gauss::gp::GaussianProcess gauss_process;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> waypoint_pubs;
    std::vector<rclcpp::Subscription<SampleReturnArray>::SharedPtr> data_subs;

    int num_robots;
    double map_x_min;
    double map_x_max;
    double map_y_min;
    double map_y_max;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SamplerOrchestrator>());
  rclcpp::shutdown();
  return 0;
}

std::vector<rclcpp::Subscription<SampleReturnArray>::SharedPtr> SamplerOrchestrator::init_data_subs(int count) {
  std::vector<rclcpp::Subscription<SampleReturnArray>::SharedPtr> sub_list(count);
  for (int i = 0; i < count; ++i) {
    std::string topic_name = "/robot" + std::to_string(i) + "/data";
    sub_list[i] = this->create_subscription<SampleReturnArray>(topic_name, 10, std::bind(&SamplerOrchestrator::upload_data_callback, this, std::placeholders::_1));
  }
  return sub_list;
}

void SamplerOrchestrator::upload_data_callback(const SampleReturnArray& msg) {

}