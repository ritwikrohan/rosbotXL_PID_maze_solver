#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/subscription.hpp"
#include <memory>
#include <thread>
#include <vector>
#include "distance_controller/pid.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node
{
    public:
        DistanceController() : Node("distance_controller_node")
        {
            odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_odom;
            options_odom.callback_group = odom_callback_group_;
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&DistanceController::odomCallback, this, std::placeholders::_1), options_odom);
            rclcpp::sleep_for(std::chrono::seconds(1));
            timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_ = this->create_wall_timer(50ms,std::bind(&DistanceController::controlLoop, this), timer_callback_group_);
            speed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        }

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
        double current_yaw = 0.0;
        double current_x = 0.0;
        double current_y = 0.0;
        const double Kp = 1.5;
        const double Ki = 0.007;
        const double Kd = 0.9;
        const double MAX_SPEED = 0.8;
        int count = 1;

        double normalizeAngle(double angle) {
            while (angle > M_PI) {
                angle -= 2.0 * M_PI;
            }
            while (angle < -M_PI) {
                angle += 2.0 * M_PI;
            }
            return angle;
        }

        void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
        {
            double qx = msg->pose.pose.orientation.x;
            double qy = msg->pose.pose.orientation.y;
            double qz = msg->pose.pose.orientation.z;
            double qw = msg->pose.pose.orientation.w;
            double unnormalized_yaw = atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
            this->current_yaw = normalizeAngle(unnormalized_yaw);
            RCLCPP_DEBUG(this->get_logger(),"Current Yaw: %f", current_yaw);
            this->current_x = msg->pose.pose.position.x;
            this->current_y = msg->pose.pose.position.y;
            // USING TF2

            // tf2::Quaternion q(
            //     msg->pose.pose.orientation.x,
            //     msg->pose.pose.orientation.y,
            //     msg->pose.pose.orientation.z,
            //     msg->pose.pose.orientation.w);
            // tf2::Matrix3x3 m(q);
            // double roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);
        }

        double calculate_error(double x, double y)
        {
            double dx = std::pow((x - this->current_x),2);
            double dy = std::pow((y - this->current_y),2);
            double error = std::sqrt(dx+dy);
            return error;
        }

        void controlLoop()
        {
            this->timer_->cancel();
            std::vector<geometry_msgs::msg::Point32> setpoints;

            geometry_msgs::msg::Point32 point1;
            point1.x = round(this->current_x + 1.0);
            point1.y = 0.0;
            setpoints.push_back(point1);

            geometry_msgs::msg::Point32 point2;
            point2.x = round(point1.x + 1.0);
            point2.y = 0.0;
            setpoints.push_back(point2);

            geometry_msgs::msg::Point32 point3;
            point3.x = round(point2.x + 1.0);
            point3.y = 0.0;
            setpoints.push_back(point3);

            for (const auto &setpoint : setpoints)
            {
                moveRobot(setpoint);
            }
        }


        void moveRobot(const geometry_msgs::msg::Point32& goal)
        {
            PID pid_distance = PID();
            pid_distance.Init(Kp, Ki, Kd);
            geometry_msgs::msg::Twist vel_msg;
            double goal_x = goal.x;
            double goal_y = goal.y;
            bool goal_reached = false;
            rclcpp::Rate loop_rate(15);
            while (!goal_reached && rclcpp::ok())
            {
                double goal_error = calculate_error(goal_x, goal_y);
                RCLCPP_DEBUG(this->get_logger(),"Goal Error: %f", goal_error);
                if (std::abs(goal_error) > 0.01)
                {
                    pid_distance.UpdateError(goal_error);
                    double speed_x = ((pid_distance.Kp * pid_distance.p_error) + (pid_distance.Kd * pid_distance.d_error) + (pid_distance.Ki * pid_distance.i_error));
                    speed_x = (std::abs(speed_x) > MAX_SPEED) ? MAX_SPEED * std::abs(speed_x) / speed_x : speed_x;
                    vel_msg.linear.x = speed_x;
                    vel_msg.angular.z = 0.0;
                    speed_pub_->publish(vel_msg);
                }
                else
                {
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = 0.0;
                    goal_reached = true;
                    if (this->count!=3)
                    {
                        RCLCPP_INFO(this->get_logger(),"Reached waypoint %d. Waiting to get next waypoint", this->count);
                    }
                    else 
                    {
                        RCLCPP_INFO(this->get_logger(),"Reached Final Waypoint 3. Task Complete. Exiting.....");
                        rclcpp::shutdown();
                    }
                    rclcpp::sleep_for(std::chrono::seconds(2));
                    RCLCPP_INFO(this->get_logger(),"Target Waypoint: (%0.1f, %d)", round(this->current_x),0);
                    RCLCPP_INFO(this->get_logger(),"Achieved Waypoint: (%f, %d)", this->current_x, 0);
                    this->count++;
                }
                loop_rate.sleep();
            }
        }



};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<DistanceController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}

