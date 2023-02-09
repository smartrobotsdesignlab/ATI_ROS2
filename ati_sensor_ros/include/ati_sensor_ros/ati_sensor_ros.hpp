#ifndef ATI_SENSOR_ROS_HPP__
#define ATI_SENSOR_ROS_HPP__
// ati_sensor_lib include
#include "ati_sensor_ros/ati_sensor_lib.hpp"

// std include
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
// ros include
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
namespace ati_sensor
{
    class ati_sensor_ros : public rclcpp::Node
    {
    public:
        ati_sensor_ros();
        ~ati_sensor_ros();

    private:
        /* functions */
        // init
        void ros_init();
        void sensor_init();
        void run();

        // set bias call back function
        bool set_bias_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        // clear bias call back function
        bool clear_bias_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        /* variables */
        // ros node
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ati_sensor_pub_;
        // sensor variable
        std::string ether_name_;
        std::shared_ptr<ati_sensor::ati_sensor_lib> ati_sensor_;
        uint update_rate_;

        // srvs
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_bias_srv_; // 1 set bias against current load; 0 use last bias
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_bias_srv_; // 1 clear bias; 0 leave bias unchanges

        // loop thread
        std::thread loop_thread_;
    };
} // namespace ati_sensor


#endif // ATI_SENSOR_ROS_HPP__