#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <GaussianProcess/GaussianProcess.h>
#include <GaussianProcess/kernel/SquaredExponential.h>
#include <TrainingTools/iterative/solvers/GradientDescend.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sampling_interfaces/msg/sample_return.hpp"
#include "sampling_interfaces/msg/sample_return_array.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;
using visualization_msgs::msg::MarkerArray;

class GPTest : public rclcpp::Node
{
  public:
    GPTest() : Node("model"), 
    count_(0),
    kernel_func(std::make_unique<gauss::gp::SquaredExponential>(1.0,1.0)),
    gauss_process(std::move(kernel_func),2,1) {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&GPTest::timer_callback, this));
      data_sub = this->create_subscription<SampleReturn>("/sample_in",10,std::bind(&GPTest::upload_data_callback,this,std::placeholders::_1));
      vis_pub = this->create_publisher<MarkerArray>("/model_vis",10);

      // Define map bounds
      map_x_max = 10.0;
      map_x_min = 0.0;
      map_y_max = 10.0;
      map_y_min = 0.0;
    }

  private:
    void timer_callback() {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      //publisher_->publish(message);
    }

    std::pair<double,double> predict(Eigen::Vector2d loc);
    void query_waypoint_callback(const std_msgs::msg::String& msg);
    void upload_data_callback(const SampleReturn& msg);
    void visualize_model();
    void retrain_hyperparams();
    std::tuple<float,float,float> colormap(double in, double max, double min);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    // Parameters for sampling setup
    gauss::gp::KernelFunctionPtr kernel_func;
    gauss::gp::GaussianProcess gauss_process;
    rclcpp::Subscription<SampleReturn>::SharedPtr data_sub;
    rclcpp::Publisher<MarkerArray>::SharedPtr vis_pub;

    double map_x_min;
    double map_x_max;
    double map_y_min;
    double map_y_max;

    int add_count;

    std::vector<Eigen::Vector3d> samples;

};

std::pair<double,double> GPTest::predict(Eigen::Vector2d loc) {
  auto predicted = gauss_process.predict(loc,gauss::gp::SINGLE_PREDICTIVE_DISTRIBUTION_TAG);
  double mean = predicted.getMean()(0);
  double var = predicted.getCovariance()(0,0);
  return std::pair<double,double>(mean,var);
}

void GPTest::upload_data_callback(const SampleReturn& msg) {
  double x = msg.pose_stamped.pose.position.x;
  double y = msg.pose_stamped.pose.position.y;
  double reading = msg.reading;

  Eigen::Vector3d obs(x,y,reading);
  gauss_process.getTrainSet().addSample(obs);
  visualize_model();

  ++add_count;
  if (add_count > 15) {
    add_count = 0;
    retrain_hyperparams();
  }
}

void GPTest::retrain_hyperparams() {
  RCLCPP_INFO(this->get_logger(), "Retraining hyperparameters");
  train::GradientDescendFixed gradient_descender;
  gauss::gp::train(gauss_process,gradient_descender);
}

void GPTest::visualize_model() {
  int res_x = 100;
  int res_y = 100;
  int n = res_x * res_y;

  std::vector<visualization_msgs::msg::Marker> markers(n);
  std::vector<double> mean_arr(n);
  std::vector<double> var_arr(n);
  for (int i = 0; i < res_x; ++i) {
    double x = map_x_min + (double) (i * (map_x_max - map_x_min)) / (double)res_x; 
    for (int j = 0; j < res_y; ++j) {
      double y = map_y_min + (double) (j * (map_y_max - map_y_min)) / (double)res_y; 
      Eigen::Vector2d loc(x,y);
      std::pair<double,double> val = predict(loc);
      double mean = val.first;
      double var = val.second;

      mean_arr[res_x*i + j] = mean;
      var_arr[res_x*i + j] = var;
    }
  }
  double mean_max = *std::max_element(mean_arr.begin(),mean_arr.end());
  double mean_min = *std::min_element(mean_arr.begin(),mean_arr.end());
  for (int i = 0; i < n; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    int i_index = i / res_x;
    int j_index = i % res_x;
    marker.pose.position.x = map_x_min + (double) (i_index * (map_x_max - map_x_min)) / (double)res_x;
    marker.pose.position.y = map_y_min + (double) (j_index * (map_y_max - map_y_min)) / (double)res_y;
    marker.pose.position.z = 0.0;
    std::tuple<float,float,float> color = colormap(mean_arr[i],mean_max,mean_min);
    marker.color.r = std::get<0>(color);
    marker.color.g = std::get<1>(color);
    marker.color.b = std::get<2>(color);
    marker.color.a = 1.0;
    marker.scale.x = 1.0 * (map_x_max - map_x_min) / (double)res_x;
    marker.scale.y = 1.0 * (map_y_max - map_y_min) / (double)res_y;
    marker.scale.z = 1.0;
    marker.type = 1;
    marker.id = i;
    markers[i] = marker;
  }

  // Publish marker array
  MarkerArray arr;
  arr.markers = markers;
  vis_pub->publish(arr);
}

std::tuple<float,float,float> GPTest::colormap(double in, double max, double min) {
  double value_norm = (in - min)/(max - min);
  std::tuple<float,float,float> color;
  std::get<0>(color) = value_norm;
  std::get<1>(color) = 0;
  std::get<2>(color) = 1 - value_norm;
  return color;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPTest>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}