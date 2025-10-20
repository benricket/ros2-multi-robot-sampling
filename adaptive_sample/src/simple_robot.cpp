#include "sampler.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;
using geometry_msgs::msg::PoseStamped;

class SimpleRobot : public Sampler {
    public:
        SimpleRobot() : Sampler() {
            speed = 0.1;
            std::string pos_topic = "/robot" + std::to_string(this->id) + "/pos";
            pos_pub = this->create_publisher<PoseStamped>(pos_topic, 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&SimpleRobot::timer_callback, this));
        }
    private:
        geometry_msgs::msg::Pose move_towards_waypoint(geometry_msgs::msg::Pose waypoint);
        geometry_msgs::msg::Pose pose;
        double speed;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<PoseStamped>::SharedPtr pos_pub;
        void timer_callback();
};

void SimpleRobot::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, I'm  robot " + std::to_string(this->id);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleRobot>());
  rclcpp::shutdown();
  return 0;
}
