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
#include "pid.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>

using namespace std::chrono_literals;

class MazeSolver : public rclcpp::Node
{
    public:
        MazeSolver() : Node("maze_solver_node")
        {
            odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_odom;
            options_odom.callback_group = odom_callback_group_;
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&MazeSolver::odomCallback, this, std::placeholders::_1), options_odom);
            rclcpp::sleep_for(std::chrono::seconds(1));
            timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_ = this->create_wall_timer(50ms,std::bind(&MazeSolver::controlLoop, this), timer_callback_group_);
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
        const double Kp_angle = 1.5; //0.9;
        const double Ki_angle = 0.007;
        const double Kd_angle = 0.9;
        const double Kp_distance = 1.5; //0.9;
        const double Ki_distance = 0.007;
        const double Kd_distance = 0.9;
        const double MAX_SPEED = 0.8;
        int count = 1;
        int count1 = 1;

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

        double calculate_distance_error(double x, double y)
        {
            double dx = std::pow((x - this->current_x),2);
            double dy = std::pow((y - this->current_y),2);
            double error = std::sqrt(dx+dy);
            return error;
        }

        double calculate_angle_error(double x, double y)
        {
            double dx = x - this->current_x;
            double dy = y - this->current_y;
            // RCLCPP_INFO(this->get_logger(),"dx: %f", dx);
            // RCLCPP_INFO(this->get_logger(),"dy: %f", dy);
            // RCLCPP_INFO(this->get_logger(),"current x: %f", this->current_x);
            // RCLCPP_INFO(this->get_logger(),"current y: %f", this->current_y);
            // RCLCPP_INFO(this->get_logger(),"x: %f", x);
            // RCLCPP_INFO(this->get_logger(),"y: %f", y);
            double error = atan2(dy,dx);
            return error;
        }

        void controlLoop()
        {
            this->timer_->cancel();
            std::vector<geometry_msgs::msg::Point32> setpoints;

            geometry_msgs::msg::Point32 point1;
            point1.x = 0.5;
            point1.y = -0.1;
            point1.z = 1.0;
            setpoints.push_back(point1);

            point1.z = 0.0;
            setpoints.push_back(point1);

            geometry_msgs::msg::Point32 point2;
            point2.x = 0.5;
            point2.y = -1.35;
            point2.z = 1.0;
            setpoints.push_back(point2);

            point2.z = 0.0;
            setpoints.push_back(point2);

            geometry_msgs::msg::Point32 point3;
            point3.x = 1.05;
            point3.y = -1.35;
            point3.z = 1.0;
            setpoints.push_back(point3);

            point3.z = 0.0;
            setpoints.push_back(point3);

            geometry_msgs::msg::Point32 point4;
            point4.x = 1.05;
            point4.y = -0.85;
            point4.z = 1.0;
            setpoints.push_back(point4);

            point4.z = 0.0;
            setpoints.push_back(point4);

            geometry_msgs::msg::Point32 point5;
            point5.x = 1.41;
            point5.y = -0.85;
            point5.z = 1.0;
            setpoints.push_back(point5);

            point5.z = 0.0;
            setpoints.push_back(point5);

            geometry_msgs::msg::Point32 point6;
            point6.x = 1.41;
            point6.y = -0.285;
            point6.z = 1.0;
            setpoints.push_back(point6);

            point6.z = 0.0;
            setpoints.push_back(point6);

            geometry_msgs::msg::Point32 point7;
            point7.x = 1.97;
            point7.y = -0.285;
            point7.z = 1.0;
            setpoints.push_back(point7);

            point7.z = 0.0;
            setpoints.push_back(point7);

            geometry_msgs::msg::Point32 point8;
            point8.x = 1.97;
            point8.y = 0.61;
            point8.z = 1.0;
            setpoints.push_back(point8);

            point8.z = 0.0;
            setpoints.push_back(point8);

            geometry_msgs::msg::Point32 point9;
            point9.x = 1.5;
            point9.y = 0.61;
            point9.z = 1.0;
            setpoints.push_back(point9);

            point9.z = 0.0;
            setpoints.push_back(point9);

            geometry_msgs::msg::Point32 point10;
            point10.x = 1.5;
            point10.y = 0.2;
            point10.z = 1.0;
            setpoints.push_back(point10);

            point10.z = 0.0;
            setpoints.push_back(point10);

            geometry_msgs::msg::Point32 point11;
            point11.x = 1.0;
            point11.y = 0.2;
            point11.z = 1.0;
            setpoints.push_back(point11);

            point11.z = 0.0;
            setpoints.push_back(point11);

            geometry_msgs::msg::Point32 point12;
            point12.x = 0.63;
            point12.y = 0.533;
            point12.z = 1.0;
            setpoints.push_back(point12);

            point12.z = 0.0;
            setpoints.push_back(point12);

            geometry_msgs::msg::Point32 point14;
            point14.x = 0.0;
            point14.y = 0.5;
            point14.z = 1.0;
            setpoints.push_back(point14);

            point14.z = 0.0;
            setpoints.push_back(point14);


            for (const auto &setpoint : setpoints)
            {
                moveRobot(setpoint);
            }
        }

        void moveRobot(const geometry_msgs::msg::Point32& goal)
        {
            PID pid_distance = PID();
            PID pid_angle = PID();
            pid_angle.Init(Kp_angle, Ki_angle, Kd_angle);
            pid_distance.Init(Kp_distance, Ki_distance, Kd_distance);
            geometry_msgs::msg::Twist vel_msg;
            double goal_x = goal.x;
            double goal_y = goal.y;
            double goal_type = goal.z;
            bool goal_reached = false;
            bool turning_message_printed = false;
            bool moving_message_printed = false;
            double goal_print = calculate_angle_error(goal_x, goal_y);
            rclcpp::Rate loop_rate(15);
            while (!goal_reached && rclcpp::ok())
            {
                double distance_goal_error = calculate_distance_error(goal_x, goal_y);
                double angle_goal_error = calculate_angle_error(goal_x, goal_y)-this->current_yaw;
                angle_goal_error = normalizeAngle(angle_goal_error);
                // RCLCPP_DEBUG(this->get_logger(),"Goal Error: %f", goal_error);
                // RCLCPP_DEBUG(this->get_logger(),"Current Yaw: %f", this->current_yaw);
                if (goal_type == 0.0 && std::abs(distance_goal_error) > 0.01)
                {
                    if (!moving_message_printed)
                    {
                        RCLCPP_INFO(this->get_logger(),"Moving to waypoint number: %d.", (this->count1+1));
                        moving_message_printed = true;
                    }
                    pid_distance.UpdateError(distance_goal_error);
                    double speed_x = ((pid_distance.Kp * pid_distance.p_error) + (pid_distance.Kd * pid_distance.d_error) + (pid_distance.Ki * pid_distance.i_error));
                    speed_x = (std::abs(speed_x) > MAX_SPEED) ? MAX_SPEED * std::abs(speed_x) / speed_x : speed_x;
                    vel_msg.linear.x = speed_x;
                    double omega_z = (std::abs(angle_goal_error * 0.1) > 0.1) ? 0.1 * std::abs(angle_goal_error * 0.1) / angle_goal_error * 0.1 : angle_goal_error * 0.1;
                    vel_msg.linear.x = speed_x;
                    vel_msg.angular.z = omega_z;
                    speed_pub_->publish(vel_msg);
                    
                }

                else if (goal_type == 1.0 && std::abs(angle_goal_error) > 0.01)
                {   
                    if (!turning_message_printed)
                    {
                        RCLCPP_INFO(this->get_logger(),"Turning towards next waypoint number: %d.", (this->count1+1));
                        turning_message_printed = true;
                    }
                    
                    pid_angle.UpdateError(angle_goal_error);
                    double speed_x = ((pid_angle.Kp * pid_angle.p_error) + (pid_angle.Kd * pid_angle.d_error) + (pid_angle.Ki * pid_distance.i_error));
                    speed_x = (std::abs(speed_x) > MAX_SPEED) ? MAX_SPEED * std::abs(speed_x) / speed_x : speed_x;
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = speed_x;
                    speed_pub_->publish(vel_msg);
                }
                else
                {
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = 0.0;
                    goal_reached = true;
                    if (this->count!=26 && moving_message_printed)
                    {
                        this->count1 = this->count1+1;
                        RCLCPP_INFO(this->get_logger(),"Reached waypoint. Waiting to get next waypoint");
                    }
                    else if (this->count!=26 && turning_message_printed)
                    {
                        RCLCPP_INFO(this->get_logger(),"Turn Complete");
                    }
                    else 
                    {
                        RCLCPP_INFO(this->get_logger(),"Reached Final Waypoint 14. Task Complete. Exiting.....");
                        rclcpp::shutdown();
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(500));
                    RCLCPP_DEBUG(this->get_logger(),"Target Yaw in degrees: %f", goal_print*180/M_PI);
                    RCLCPP_DEBUG(this->get_logger(),"Achieved Yaw in degrees: %f", this->current_yaw*180/M_PI);
                    RCLCPP_DEBUG(this->get_logger(),"Target Waypoint: (%0.1f, %d)", round(this->current_x),0);
                    RCLCPP_DEBUG(this->get_logger(),"Achieved Waypoint: (%f, %d)", this->current_x, 0);
                    this->count++;
                }
                loop_rate.sleep();
            }
        }



};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MazeSolver>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}



