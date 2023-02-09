#ifndef ATI_SENSOR_LIB_HPP__
#define ATI_SENSOR_LIB_HPP__

// std include
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <thread>


// soem include
#include "soem/ethercat.h"

#define EC_TIMEOUTMON 500
#define STATECHECK_PERIOD 100000

#define COUT_RED "\033[1;31m"
#define COUT_YELLOW "\033[1;33m"
#define COUT_RESET "\033[0m"

using std::placeholders::_1;

namespace ati_sensor
{

// force units in index 0x2021 subindex 2x2f
static const std::vector<std::string> FORCE_UNITS = {"lfb", "N", "Klbf", "KN", "Kg"};

// torque units in index 0x2021 subindex 2x30
static const std::vector<std::string> TORQUE_UNITS = {"lbf*in", "lbf-ft", "Nm", "Nmm", "Kgf-cm", "kNm"};

class ati_sensor_lib
{
public:
    // setup and start
    bool ati_setup(std::string ether_name);
    // stop
    void stop_ethercat();
    void start_ethercheck();
    void show_ati_infomation();

    // read and write
    int* ati_read();
    void ati_write();

    // set bias
    void set_bias(bool cmd);
    void clear_bias(bool cmd);

    double get_fx();
    double get_fy();
    double get_fz();
    double get_tx();
    double get_ty();
    double get_tz();
private:
    // ethercat state check thread
    void ecat_statecheck();
    void read_control_codes();

    std::string ether_name_;
    char IOmap_[4096];

    int FT_data_[6];

    // state
    std::thread thread_statecheck_;
    bool pdo_transfer_active_;
    volatile int expectedWKC_;
    volatile int wkc_;

    uint32_t control_code_;
};
    

} // namespace ati_sensor





#endif