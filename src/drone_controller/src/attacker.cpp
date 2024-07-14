#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "el_force/el_force.hpp"

class Attacker : public rclcpp::Node
{
private:
  std::string drone_name_;
  geometry_msgs::msg::Pose pose_;
  geometry_msgs::msg::Point32 target_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  ElForce el_force_;

  void pose_callback_(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    pose_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "pose: %f, %f, %f", pose_.position.x, pose_.position.y, pose_.position.z);
  }

  void target_callback_(const geometry_msgs::msg::Point32::SharedPtr msg)
  {
    target_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "target: %f, %f, %f", target_.x, target_.y, target_.z);
    std::vector<double> target_pos = {target_.x, target_.y, target_.z};
    el_force_.setTarget(target_pos);
  }

  void main_loop_()
  {
    geometry_msgs::msg::Twist msg;

    std::vector<double> pos = {pose_.position.x, pose_.position.y, pose_.position.z};
    std::vector<double> forceVec = el_force_.calculateForce(pos);

    for (int i = 0; i < (int)forceVec.size(); i++)
    {
      if (forceVec[i] > 1.0)
      {
        forceVec[i] = 1.0;
      }
      else if (forceVec[i] < -1.0)
      {
        forceVec[i] = -1.0;
      }
    }

    msg.linear.x = forceVec[0];
    msg.linear.y = forceVec[1];
    msg.linear.z = forceVec[2];

    cmd_vel_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Calculated force: %f, %f, %f", forceVec[0], forceVec[1], forceVec[2]);
  }

public:
  Attacker(const std::string &drone_name) : Node("attacker"), drone_name_(drone_name), el_force_(10.0, 10.0, 10.0, 10.0)
  {
    std::string cmd_vel_topic_ = "/" + drone_name_ + "/cmd_vel";
    std::string pose_topic_ = "/" + drone_name_ + "/pose";

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(pose_topic_, 10, std::bind(&Attacker::pose_callback_, this, std::placeholders::_1));
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point32>("/target", 10, std::bind(&Attacker::target_callback_, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Attacker::main_loop_, this));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc < 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run drone_controller attacker <drone_name>");
    return 1;
  }

  std::string drone_name = argv[1];
  rclcpp::spin(std::make_shared<Attacker>(drone_name));
  rclcpp::shutdown();
  return 0;
}
