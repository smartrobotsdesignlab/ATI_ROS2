#include "ati_sensor_ros/ati_sensor_ros.hpp"

namespace ati_sensor
{
    ati_sensor_ros::ati_sensor_ros() : Node("ati_sensor_ros")
    {
        RCLCPP_INFO(this->get_logger(), "ATI Sensor ROS Node Started");
        ros_init();
        sensor_init();
    }
    ati_sensor_ros::~ati_sensor_ros()
    {
        RCLCPP_INFO(this->get_logger(),"ATI Sensor ROS Node Stopped");
        ati_sensor_->stop_ethercat();
        // loop_thread_.join();
        std::cout << "ATI Sensor ROS Node Stopped" << std::endl;
    }

    void ati_sensor_ros::ros_init()
    {
        // ros init
        reset_bias_ = false;
        this->declare_parameter("ether_name", "enp3s0f1");
        if (!this->get_parameter("ether_name", ether_name_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get ether_name parameter");
        }
        this->declare_parameter("update_rate", 500);
        if (!this->get_parameter("update_rate", update_rate_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get update_rate parameter");
        }

        ati_sensor_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("ft_sensor_wrench", 10);

        // srvs
        set_bias_srv_ = this->create_service<std_srvs::srv::SetBool>("set_bias", std::bind(&ati_sensor_ros::set_bias_cb, this, std::placeholders::_1, std::placeholders::_2));
        clear_bias_srv_ = this->create_service<std_srvs::srv::SetBool>("clear_bias", std::bind(&ati_sensor_ros::clear_bias_cb, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(),"ATI Sensor ROS Node Initialized");
    }

    void ati_sensor_ros::sensor_init()
    {
        // sensor init
        ati_sensor_.reset(new ati_sensor::ati_sensor_lib());
        ati_sensor_->ati_setup(ether_name_);
        ati_sensor_->show_ati_infomation();

        ati_sensor_->start_ethercheck();

        loop_thread_ = std::thread(&ati_sensor_ros::run, this);
    }

    void ati_sensor_ros::resume()
    {
        reset_bias_ = false;
        resume_timer_.reset();
    }

    // set bias call back function
    bool ati_sensor_ros::set_bias_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        ati_sensor_->set_bias(request->data);
        if (!request->data)
        {
            resume_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ati_sensor_ros::resume, this));
        }
        

        response->success = true;
        return true;
    }

    // clear bias call back function
    bool ati_sensor_ros::clear_bias_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // Start to clear bias
        if (request->data) 
        {
            reset_bias_ = true;
        }
        
        ati_sensor_->clear_bias(request->data);

        response->success = true;
        return true;
    }



    void ati_sensor_ros::run()
    {
        // ros loop
        geometry_msgs::msg::WrenchStamped ati_sensor_msg = geometry_msgs::msg::WrenchStamped();

        ati_sensor_msg.header.frame_id = "ati_sensor";
        auto loop_rate = std::chrono::milliseconds(1000/ update_rate_);
        
        RCLCPP_INFO(this->get_logger(),"ATI Sensor ROS Node Running");
        while (rclcpp::ok())
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            // rclcpp::Time now = this->now();
            ati_sensor_->ati_write();
            // rclcpp::Duration duration_t = this->now() - now;
            // RCLCPP_INFO(this->get_logger(),"duration: %f", duration_t.seconds());
            ati_sensor_msg.header.stamp = this->now();
            ati_sensor_msg.wrench.force.x = ati_sensor_->get_fx();
            ati_sensor_msg.wrench.force.y = ati_sensor_->get_fy();
            ati_sensor_msg.wrench.force.z = ati_sensor_->get_fz();
            ati_sensor_msg.wrench.torque.x = ati_sensor_->get_tx();
            ati_sensor_msg.wrench.torque.y = ati_sensor_->get_ty();
            ati_sensor_msg.wrench.torque.z = ati_sensor_->get_tz();
            if (!reset_bias_)
            {
                ati_sensor_pub_->publish(ati_sensor_msg);
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            if (duration < loop_rate)
            {
                std::this_thread::sleep_for(loop_rate - duration);
            }
            // osal_usleep(2000);
        }
        // stop sensor
        // ati_sensor_->stop_ethercat();
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