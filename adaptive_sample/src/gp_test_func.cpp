#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sampling_interfaces/msg/sample_return.hpp"
#include "sampling_interfaces/msg/sample_return_array.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;

class FunctionPub : public rclcpp::Node {
    public:
        FunctionPub() : Node("sampler"), rd(), rand_gen(rd()), 
        x_dist(0,10), y_dist(0,10), noise_dist(0.0,0.05) {
            pub = this->create_publisher<SampleReturn>("/sample_in",10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&FunctionPub::timer_callback, this));
            map_x_min = 0;
            map_x_max = 10;
            map_y_min = 0;
            map_y_max = 10;
        }

        void timer_callback() {
            // Prompt for input for values
            std::cout << "Enter to sample a new point at random ";
            std::cin.get();

            double x = x_dist(rand_gen);
            double y = y_dist(rand_gen);
            std::cout << "X: " << x << ", Y: " << y << "\n";
            
            // Wrap values in SampleReturn for msg passing
            SampleReturn msg;
            msg.pose_stamped.pose.position.x = x;
            msg.pose_stamped.pose.position.y = y;
            msg.reading = eval_normal(x,y);

            // Add noise to reading
            msg.reading += noise_dist(rand_gen);
            pub->publish(msg);
        }
    private:
        rclcpp::Publisher<SampleReturn>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer_;
        std::random_device rd;
        std::mt19937 rand_gen;
        std::uniform_real_distribution<> x_dist;
        std::uniform_real_distribution<> y_dist;
        std::normal_distribution<> noise_dist;

        double map_x_min;
        double map_x_max;
        double map_y_min;
        double map_y_max;

        double normalize_coord(double x, double min, double max) {
            return 2.0 * ((x - min) / (max - min)) - 1.0;
        }

        double eval_normal(double x_in, double y_in) {
            double x = normalize_coord(x_in,map_x_min,map_x_max);
            double y = normalize_coord(y_in,map_y_min,map_y_max);

            return exp(-1 * (pow(x,2) + pow(y,2)));
        }

        double eval_donut(double x_in, double y_in) {
            double x = normalize_coord(x_in,map_x_min,map_x_max);
            double y = normalize_coord(y_in,map_y_min,map_y_max);

            double x_bias = 0.1;
            double x_scale = 4;
            double y_scale = 4;

            return exp(x_bias*x - pow((1 - (pow(x_scale*x,2) + pow(y_scale*y,2))),2));
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FunctionPub>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}