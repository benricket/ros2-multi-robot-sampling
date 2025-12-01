#include "sampler.hpp"

using namespace std::chrono_literals;
using sampling_interfaces::msg::SampleReturn;
using sampling_interfaces::msg::SampleReturnArray;
using geometry_msgs::msg::PoseStamped;

class SimpleRobot : public Sampler {
    public:
        SimpleRobot() : Sampler() {
            speed = 1.0;
            //std::string pos_topic = "/robot" + std::to_string(this->id) + "/pos"; removed in favor of namespace
            pos_pub = this->create_publisher<PoseStamped>("pos", 10);
            target_pub = this->create_publisher<PoseStamped>("target",10);
            waypt_sub = this->create_subscription<geometry_msgs::msg::Pose>("waypt_in",10,std::bind(&SimpleRobot::waypt_callback,this,std::placeholders::_1));
            timer_ = this->create_wall_timer(
                100ms, std::bind(&SimpleRobot::timer_callback, this));
            last_time_moved = this->get_clock().get()->now().seconds();
        }
    private:
        geometry_msgs::msg::Pose move_towards_waypoint(geometry_msgs::msg::Pose waypoint);
        geometry_msgs::msg::Pose pose;
        double speed;
        double last_time_moved;
        rclcpp::TimerBase::SharedPtr timer_;
        std::queue<geometry_msgs::msg::Pose> waypts;

        rclcpp::Publisher<PoseStamped>::SharedPtr pos_pub;
        rclcpp::Publisher<PoseStamped>::SharedPtr target_pub;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr waypt_sub;
        void timer_callback();
        void waypt_callback(geometry_msgs::msg::Pose msg);
        void sample();
};

void SimpleRobot::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, I'm  robot " + std::to_string(this->id);
  //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
  double time_now = this->get_clock().get()->now().seconds();

  // Publish where I am
  geometry_msgs::msg::PoseStamped ps;
  ps.pose = this->pose;
  ps.header.stamp.sec = time_now;
  this->pos_pub->publish(ps);

  double dt = time_now - this->last_time_moved;
  double travel = dt * this->speed;
  std::cout << "X " << this->pose.position.x << ", Y " << this->pose.position.y << std::endl;
  std::cout << "dt: " << dt << ", " << time_now << std::endl;
  this->last_time_moved = time_now;

  int num_waypts = this->waypts.size();
  if (num_waypts < 1) {
    return; // I have no more waypoints, can't do anything
  }

  // Try to move toward waypt
  geometry_msgs::msg::Pose target = this->waypts.front();
  geometry_msgs::msg::Pose new_pose = this->pose;
  double x_f = target.position.x;
  double y_f = target.position.y;
  
  // Publish my target waypoint
  geometry_msgs::msg::PoseStamped target_pos;
  target_pos.header.stamp.sec = time_now;
  target_pos.pose.position.x = x_f;
  target_pos.pose.position.y = y_f;
  target_pub->publish(target_pos);

  double x_diff = x_f - this->pose.position.x;
  double y_diff = y_f - this->pose.position.y;

  double length = sqrt(pow(x_diff,2)+pow(y_diff,2));

  if (length > travel) {
    double x_comp = x_diff / length;
    double y_comp = y_diff / length;

    new_pose.position.x = this->pose.position.x + travel * x_comp;
    new_pose.position.y = this->pose.position.y + travel * y_comp;
  } else {
    std::cout << "Reached target!" << std::endl;
    new_pose.position = target.position;
    this->waypts.pop();
    this->sample();
  }
  this->pose = new_pose;
}

void SimpleRobot::sample() {
  SampleReturnArray out;
  SampleReturn sample;
  sample.pose_stamped.header.frame_id = "map";
  sample.pose_stamped.header.stamp.sec = this->get_clock().get()->now().seconds();
  sample.pose_stamped.pose = this->pose;
  //sample.reading = this->pose.position.x - this->pose.position.y; // test to be replaced
  // Take a sample and upload it 

  // Test reward function --- normal centered at 5,5, longer along x
  double value = exp(-0.25*(pow(this->pose.position.x - 5.0,2)+3.0*pow(this->pose.position.y - 5.0,2)));

  sample.reading = value;
  this->upload_data(sample);
}

void SimpleRobot::waypt_callback(geometry_msgs::msg::Pose msg) {
  std::cout << "Got a waypoint!!";
  this->waypts.push(msg);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleRobot>());
  rclcpp::shutdown();
  return 0;
}
