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

class LocationPub : public rclcpp::Node {
    public:
        LocationPub() : Node("location_pub") {
            pub = this->create_publisher<SampleReturn>("/loc_in",10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&LocationPub::timer_callback, this));
        }

        void timer_callback() {
            double x, y;
            // Prompt for input for values
            std::cout << "Enter the X position to calculate reward from: ";
            std::cin >> x;
            std::cout << "Enter the Y position to calculate reward from: ";
            std::cin >> y;
            std::cout << "Publishing value to topic" << std::endl;
            
            // Wrap values in SampleReturn for msg passing
            SampleReturn msg;
            msg.pose_stamped.pose.position.x = x;
            msg.pose_stamped.pose.position.y = y;
            pub->publish(msg);
        }
    private:
        rclcpp::Publisher<SampleReturn>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocationPub>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}