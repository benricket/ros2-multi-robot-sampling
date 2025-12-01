#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <geometry_msgs/msg/pose.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sampling_interfaces/msg/sample_return.hpp"
#include "sampling_interfaces/msg/sample_return_array.hpp"

using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;

class Sampler : public rclcpp::Node {
    public:
        Sampler();
    protected:
        void timer_callback();
        void query_waypoint();
        void upload_data(SampleReturn);

        SampleReturn take_sample();

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        int id;

        geometry_msgs::msg::Pose pose;
        std::vector<SampleReturn> sample_vec;
        rclcpp::Publisher<SampleReturn>::SharedPtr sample_pub;

};