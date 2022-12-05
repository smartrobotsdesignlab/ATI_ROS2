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
#include <geometry_msgs/msg/wrench.hpp>

namespace ati_sensor
{
    class ati_sensor_ros : public rclcpp::Node
    {
    public:
        ati_sensor_ros();

    private:
        /* functions */
        // init
        void ros_init();
        void sensor_init();
        void run();

        /* variables */
        // ros node
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr ati_sensor_pub_;
        // sensor variable
        std::string ether_name_;
        std::shared_ptr<ati_sensor::ati_sensor_lib> ati_sensor_;
        uint update_rate_;

        // loop thread
        std::thread loop_thread_;
    };
} // namespace ati_sensor


#endif // ATI_SENSOR_ROS_HPP__