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

int main(int argc, char const *argv[])
{
    ati_sensor::ati_sensor_lib test_lib;
    if  (!test_lib.ati_setup(argv[1])){
      std::cout << COUT_RED << "ERROR : Ethercat setup failed." << COUT_RESET << std::endl;
      return -1;
    }
    test_lib.start_ethercheck();
    std::cout << "Start reading" << std::endl;
    while (true)
    {
        test_lib.ati_write(); 
        int* data = test_lib.ati_read();
        std::cout << "FT_data_ " << (double)data[0]/1000000.0 
                        << " " << (double)data[1]/1000000.0 
                        << " " << (double)data[2]/1000000.0 
                        << " " << (double)data[3]/1000000.0 
                        << " " << (double)data[4]/1000000.0 
                        << " " << (double)data[5]/1000000.0 << std::endl;
        osal_usleep(2000);
    }
    return 0;
}
