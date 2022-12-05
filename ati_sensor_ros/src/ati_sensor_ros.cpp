#include "ati_sensor_ros/ati_sensor_ros.hpp"

namespace ati_sensor
{
    ati_sensor_ros::ati_sensor_ros() : Node("ati_sensor_ros")
    {
        RCLCPP_INFO(this->get_logger(), "ATI Sensor ROS Node Started");
        ros_init();
        sensor_init();
    }

    void ati_sensor_ros::ros_init()
    {
        // ros init
        this->declare_parameter("ether_name", "enx2c16dba732f8");
        if (!this->get_parameter("ether_name", ether_name_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get ether_name parameter");
        }
        this->declare_parameter("update_rate", 200);
        if (!this->get_parameter("update_rate", update_rate_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get update_rate parameter");
        }

        ati_sensor_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("ati_sensor", 10);
        RCLCPP_INFO(this->get_logger(),"ATI Sensor ROS Node Initialized");
    }

    void ati_sensor_ros::sensor_init()
    {
        // sensor init
        ati_sensor_.reset(new ati_sensor::ati_sensor_lib());
        ati_sensor_->ati_setup(ether_name_);

        ati_sensor_->start_ethercheck();

        loop_thread_ = std::thread(&ati_sensor_ros::run, this);
    }


    void ati_sensor_ros::run()
    {
        // ros loop
        auto ati_sensor_msg = geometry_msgs::msg::Wrench();
        auto loop_rate = std::chrono::milliseconds(1000 / update_rate_);
        RCLCPP_INFO(this->get_logger(),"ATI Sensor ROS Node Running");
        while (rclcpp::ok())
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            ati_sensor_->ati_write();
            ati_sensor_msg.force.x = ati_sensor_->get_fx();
            ati_sensor_msg.force.y = ati_sensor_->get_fy();
            ati_sensor_msg.force.z = ati_sensor_->get_fz();
            ati_sensor_msg.torque.x = ati_sensor_->get_tx();
            ati_sensor_msg.torque.y = ati_sensor_->get_ty();
            ati_sensor_msg.torque.z = ati_sensor_->get_tz();
            RCLCPP_INFO(this->get_logger(),"data is: %f", ati_sensor_msg.force.x);

            ati_sensor_pub_->publish(ati_sensor_msg);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            if (duration < loop_rate)
            {
                std::this_thread::sleep_for(loop_rate - duration);
            }
            osal_usleep(2000);
        }
        // stop sensor
        ati_sensor_->stop_ethercat();
    }
} // namespace ati_sensor

int main(int argc, char const *argv[])
{
    /* code */
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ati_sensor::ati_sensor_ros>();
    // executor.add_node(node);
    rclcpp::spin(node);
    // executor.spin();
    rclcpp::shutdown();

    return 0;
}