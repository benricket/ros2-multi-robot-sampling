#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <random>
#include <GaussianProcess/GaussianProcess.h>
#include <GaussianProcess/kernel/SquaredExponential.h>
#include <TrainingTools/iterative/solvers/GradientDescend.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sampling_interfaces/msg/sample_return.hpp"
#include "sampling_interfaces/msg/sample_return_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;
using std_msgs::msg::Float64MultiArray;
using visualization_msgs::msg::MarkerArray;

class GPTest : public rclcpp::Node
{
  public:
    GPTest() : Node("model"), 
    count_(0),
    rand_gen(std::chrono::system_clock::now().time_since_epoch().count()),
    waypt_rand(0.0,0.2),
    kernel_func(std::make_unique<gauss::gp::SquaredExponential>(1.0,1.0)),
    gauss_process(std::move(kernel_func),2,1) {

      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&GPTest::timer_callback, this));
      data_sub = this->create_subscription<SampleReturn>("/sample_in",10,std::bind(&GPTest::upload_data_callback,this,std::placeholders::_1));
      loc_sub = this->create_subscription<SampleReturn>("/loc_in",10,std::bind(&GPTest::upload_position_callback,this,std::placeholders::_1));

      // Visualization of model
      vis_pub = this->create_publisher<MarkerArray>("/model_vis",10);
      vis_scaled_pub = this->create_publisher<MarkerArray>("/model_vis_scaled",10);
      my_loc_pub = this->create_publisher<visualization_msgs::msg::Marker>("/my_loc",10);
      target_loc_pub = this->create_publisher<visualization_msgs::msg::Marker>("/target_loc",10);

      // waypoint pub (just one for now)
      waypt_pub = this->create_publisher<geometry_msgs::msg::Pose>("/waypt_in",10);

      // Visualization of robot position and target
      my_loc_pub = this->create_publisher<visualization_msgs::msg::Marker>("/my_loc",10);
      target_loc_pub = this->create_publisher<visualization_msgs::msg::Marker>("/target_loc",10);

      // waypoint pub (just one for now)
      waypt_pub = this->create_publisher<geometry_msgs::msg::Pose>("/waypt_in",10);

      // TODO change to support multiple robots
      single_data_sub = this->create_subscription<SampleReturn>("/robot999/data",10,std::bind(&GPTest::upload_data_callback,this,std::placeholders::_1));
      single_waypt_pub = this->create_publisher<geometry_msgs::msg::Pose>("/robot999/waypt_in",10);

      mpl_mean_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/model_mean",10);
      mpl_var_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/model_var",10);
      mpl_cost_unweighted_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/model_cost_unweighted",10);
      mpl_cost_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/model_cost",10);

      // Define map bounds
      map_x_max = 10.0;
      map_x_min = 0.0;
      map_y_max = 10.0;
      map_y_min = 0.0;

      res_x = 100;
      res_y = 100;
      num_cells = res_x * res_y;

      add_count = 0;

      // TODO setup a reasonable model prior rather than doing this
      double x_init_guesses[] = {0.1,0.2,3.0,9.7,9.5};
      double y_init_guesses[] = {0.2,9.8,2.5,9.9,0.1};
      for (int i = 0; i < 5; ++i) {
        double x = x_init_guesses[i];
        double y = y_init_guesses[i];
        // test function, multivariate normal centered at 5,5 
        double reading = exp(-0.25*(pow(x-5.0,2)+3*pow(y-5.0,2)));
        Eigen::Vector3d obs(x,y,reading);
        gauss_process.getTrainSet().addSample(obs);
      }
      this->visualize_model();
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
    void upload_position_callback(const SampleReturn& msg);
    std::vector<double> compute_reward(double var_weight);
    std::pair<double,double> weight_model_distance(double x, double y, double dist_weight, double var_weight);
    void display_scalar_field(std::vector<double> values,rclcpp::Publisher<MarkerArray>::SharedPtr& pub);
    void visualize_model();
    void retrain_hyperparams();
    std::tuple<float,float,float> colormap(double in, double max, double min);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    std::mt19937 rand_gen;
    std::normal_distribution<double> waypt_rand;

    // Parameters for sampling setup
    gauss::gp::KernelFunctionPtr kernel_func;
    gauss::gp::GaussianProcess gauss_process;
    rclcpp::Subscription<SampleReturn>::SharedPtr data_sub;
    rclcpp::Subscription<SampleReturn>::SharedPtr loc_sub;
    rclcpp::Publisher<MarkerArray>::SharedPtr vis_pub;
    rclcpp::Publisher<MarkerArray>::SharedPtr vis_scaled_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr my_loc_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_loc_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr waypt_pub;
    //std::vector<rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> waypt_pubs;
    rclcpp::Subscription<SampleReturn>::SharedPtr single_data_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr single_waypt_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mpl_mean_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mpl_var_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mpl_cost_unweighted_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mpl_cost_pub;

    double map_x_min;
    double map_x_max;
    double map_y_min;
    double map_y_max;

    int res_x;
    int res_y;
    int num_cells;

    int add_count;

    std::vector<Eigen::Vector3d> samples;
    
};

std::pair<double,double> GPTest::predict(Eigen::Vector2d loc) {
  //RCLCPP_INFO(this->get_logger(),("Samples input size: "+std::to_string(gauss_process.getTrainSet().GetSamplesInput().size())).c_str());
  if (gauss_process.getTrainSet().GetSamplesInput().size() == 0) {
    RCLCPP_INFO(this->get_logger(),"Attempted prediction with no samples");
    return {0.0, 0.0};
  }
  auto predicted = gauss_process.predict(loc,gauss::gp::SINGLE_PREDICTIVE_DISTRIBUTION_TAG);
  double mean = predicted.getMean()(0);
  double var = predicted.getCovariance()(0,0);
  return std::pair<double,double>(mean,var);
}

void GPTest::upload_data_callback(const SampleReturn& msg) {
  double x = msg.pose_stamped.pose.position.x;
  double y = msg.pose_stamped.pose.position.y;
  double reading = msg.reading;

  // Publish input location
  visualization_msgs::msg::Marker my_loc;
  my_loc.pose.position.x = x;
  my_loc.pose.position.y = y;
  my_loc.pose.position.z = 0.5;
  my_loc.scale.x = 1.0;
  my_loc.scale.y = 1.0;
  my_loc.scale.z = 1.0;
  my_loc.color.r = 1.0;
  my_loc.color.g = 1.0;
  my_loc.color.b = 1.0;
  my_loc.color.a = 1.0;
  my_loc_pub->publish(my_loc);

  // Calculate target point and visualize weights
  std::pair<double,double> waypoint = weight_model_distance(x,y,0.01,0.1);
  double waypt_x = waypoint.first;
  double waypt_y = waypoint.second;

  // Upload waypoint
  geometry_msgs::msg::Pose pt;
  pt.position.x = waypt_x;
  pt.position.y = waypt_y;
  this->single_waypt_pub->publish(pt);
  RCLCPP_INFO(this->get_logger(), ("Published new waypoint: "+std::to_string(pt.position.x)+std::to_string(pt.position.y)).c_str());

  Eigen::Vector3d obs(x,y,reading);
  gauss_process.getTrainSet().addSample(obs);
  visualize_model();

  ++add_count;
  if (add_count > 20) {
    add_count = 0;
    retrain_hyperparams();
  }

  // Publish target location
  visualization_msgs::msg::Marker target_loc;
  target_loc.pose.position.x = waypt_x;
  target_loc.pose.position.y = waypt_y;
  target_loc.pose.position.z = 0.5;
  target_loc.scale.x = 1.0;
  target_loc.scale.y = 1.0;
  target_loc.scale.z = 1.0;
  target_loc.color.r = 0.0;
  target_loc.color.g = 1.0;
  target_loc.color.b = 0.0;
  target_loc.color.a = 1.0;
  target_loc_pub->publish(target_loc);
}

void GPTest::upload_position_callback(const SampleReturn& msg) {
  double x = msg.pose_stamped.pose.position.x;
  double y = msg.pose_stamped.pose.position.y;

  // Publish input location
  visualization_msgs::msg::Marker my_loc;
  my_loc.pose.position.x = x;
  my_loc.pose.position.y = y;
  my_loc.pose.position.z = 0.5;
  my_loc.scale.x = 1.0;
  my_loc.scale.y = 1.0;
  my_loc.scale.z = 1.0;
  my_loc.color.r = 1.0;
  my_loc.color.g = 1.0;
  my_loc.color.b = 1.0;
  my_loc.color.a = 1.0;
  my_loc_pub->publish(my_loc);

  // Calculate target point and visualize weights
  std::pair<double,double> waypoint = weight_model_distance(x,y,0.1,0.0);
  double waypt_x = waypoint.first;
  double waypt_y = waypoint.second;

  // Upload waypoint
  geometry_msgs::msg::Pose pt;
  pt.position.x = waypt_x;
  pt.position.y = waypt_y;
  this->waypt_pub->publish(pt);

  // Publish target location
  visualization_msgs::msg::Marker target_loc;
  target_loc.pose.position.x = waypt_x;
  target_loc.pose.position.y = waypt_y;
  target_loc.pose.position.z = 0.5;
  target_loc.scale.x = 1.0;
  target_loc.scale.y = 1.0;
  target_loc.scale.z = 1.0;
  target_loc.color.r = 0.0;
  target_loc.color.g = 1.0;
  target_loc.color.b = 0.0;
  target_loc.color.a = 1.0;
  target_loc_pub->publish(target_loc);
}

void GPTest::retrain_hyperparams() {
  RCLCPP_INFO(this->get_logger(), "Retraining hyperparameters");
  train::GradientDescendFixed gradient_descender;
  gauss::gp::train(gauss_process,gradient_descender);
}

std::vector<double> GPTest::compute_reward(double var_weight = 0.1) {
  std::vector<double> reward_arr(num_cells);
  for (int i = 0; i < res_x; ++i) {
    double x = map_x_min + (double) (i * (map_x_max - map_x_min)) / (double)res_x; 
    for (int j = 0; j < res_y; ++j) {
      double y = map_y_min + (double) (j * (map_y_max - map_y_min)) / (double)res_y; 
      Eigen::Vector2d loc(x,y);
      std::pair<double,double> val = predict(loc);
      double mean = val.first;
      double var = val.second;

      // Upper confidence bound reward
      reward_arr[i*res_x + j] = mean + var_weight * var;
    }
  }
  return reward_arr;
}

std::pair<double,double> GPTest::weight_model_distance(double x, double y, double dist_weight = 0.5, double var_weight = 0.0) {
  std::vector<double> rewards = compute_reward(var_weight);
  std::vector<double> rewards_scaled(num_cells);
  double reward_scale_max = 0;
  double waypt_x = 0;
  double waypt_y = 0;

  for (int i = 0; i < num_cells; ++i) {
    int x_index = i / res_x;
    int y_index = i % res_x;
    double x_cell = map_x_min + (double) (x_index * (map_x_max - map_x_min)) / (double)res_x;
    double y_cell = map_y_min + (double) (y_index * (map_y_max - map_y_min)) / (double)res_y;
    double reward = rewards[i];
    //std::cout << "cells x, y: " << x_cell << ", " << y_cell << std::endl;

    double dist = sqrt(pow(x - x_cell,2)+pow(y - y_cell,2));
    double reward_scaled = reward - dist_weight * dist;
    rewards_scaled[i] = reward_scaled;
    if (reward_scaled > reward_scale_max) {
      reward_scale_max = reward_scaled;
      waypt_x = x_cell;
      waypt_y = y_cell;
      //std::cout << "waypt x, y: " << waypt_x << ", " << waypt_y << std::endl;
    }
  }
  
  // Visualize scaled and unscaled scalar fields
  Float64MultiArray cost_unweighted_arr;
  cost_unweighted_arr.layout.dim.resize(2);
  cost_unweighted_arr.layout.dim[0].stride = res_x * res_y;
  cost_unweighted_arr.layout.dim[1].stride = res_y;
  cost_unweighted_arr.data = rewards;
  mpl_cost_unweighted_pub->publish(cost_unweighted_arr);
  
  Float64MultiArray cost_arr;
  cost_arr.layout.dim.resize(2);
  cost_arr.layout.dim[0].stride = res_x * res_y;
  cost_arr.layout.dim[1].stride = res_y;
  cost_arr.data = rewards_scaled;
  mpl_cost_pub->publish(cost_arr);

  display_scalar_field(rewards_scaled,vis_scaled_pub);

  // Apply random noise to chosen waypoint
  waypt_x += waypt_rand(rand_gen);
  waypt_y += waypt_rand(rand_gen);

  return std::pair<double,double>(waypt_x,waypt_y);
}

void GPTest::display_scalar_field(std::vector<double> values,rclcpp::Publisher<MarkerArray>::SharedPtr& pub) {
  double value_max = *std::max_element(values.begin(),values.end());
  double value_min = *std::min_element(values.begin(),values.end());
  std::vector<visualization_msgs::msg::Marker> markers(num_cells);

  for (int i = 0; i < num_cells; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    int i_index = i / res_x;
    int j_index = i % res_x;
    marker.pose.position.x = map_x_min + (double) (i_index * (map_x_max - map_x_min)) / (double)res_x;
    marker.pose.position.y = map_y_min + (double) (j_index * (map_y_max - map_y_min)) / (double)res_y;
    marker.pose.position.z = 0.0;
    std::tuple<float,float,float> color = colormap(values[i],value_max,value_min);
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
  pub->publish(arr);
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

  // Make big arrays to pass to visualize
  Float64MultiArray mean_float_arr;
  mean_float_arr.layout.dim.resize(2);
  mean_float_arr.layout.dim[0].stride = res_x * res_y;
  mean_float_arr.layout.dim[1].stride = res_y;
  mean_float_arr.data = mean_arr;
  mpl_mean_pub->publish(mean_float_arr);

  Float64MultiArray var_float_arr;
  var_float_arr.layout.dim.resize(2);
  var_float_arr.layout.dim[0].stride = res_x * res_y;
  var_float_arr.layout.dim[1].stride = res_y;
  var_float_arr.data = var_arr;
  mpl_var_pub->publish(var_float_arr);

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