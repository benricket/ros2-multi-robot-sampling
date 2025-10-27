#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sampling_interfaces/msg/sample_return.hpp"
#include "sampling_interfaces/msg/sample_return_array.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;

class SamplePub : public rclcpp::Node {
    public:
        SamplePub() : Node("sampler") {
            pub = this->create_publisher<SampleReturn>("/sample_in",10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&SamplePub::timer_callback, this));
        }

        void timer_callback() {
            double x, y, val;
            // Prompt for input for values
            std::cout << "Enter the X position to sample: ";
            std::cin >> x;
            std::cout << "Enter the Y position to sample: ";
            std::cin >> y;
            std::cout << "Enter the value sampled at (" << x << ", " << y << "): ";
            std::cin >> val;
            std::cout << "Publishing value to topic" << std::endl;
            
            // Wrap values in SampleReturn for msg passing
            SampleReturn msg;
            msg.pose_stamped.pose.position.x = x;
            msg.pose_stamped.pose.position.y = y;
            msg.reading = val;
            pub->publish(msg);
        }
    private:
        rclcpp::Publisher<SampleReturn>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SamplePub>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}