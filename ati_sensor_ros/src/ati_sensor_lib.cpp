#include "ati_sensor_ros/ati_sensor_lib.hpp"

namespace ati_sensor
{

bool ati_sensor_lib::ati_setup(std::string ether_name)
{
  ether_name_ = ether_name;
  pdo_transfer_active_ = false;
  
  // Initial control code && I/O map config
  /*
   * 12-15: 0=487Hz, 1=975Hz, 2=1990Hz, 3=3900Hz
   * 8 -11: Calibration 0 and Calibration 1
   * 4 - 7: 0=No filter, 1-8 Reference pp26.
   * 0 - 3: Clear and Set bias
  */
  control_code_ = 0x0080; // 
  ec_SDOwrite(1, 0x7010, 0x01, FALSE, sizeof(control_code_), &control_code_, EC_TIMEOUTRXM);
  

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ether_name_.c_str()))
  {
    std::cout << "ec_init on " << ether_name_.c_str() << " succeeded." << std::endl;
    /* find and auto-config slaves */

    if (ec_config_init(FALSE) > 0)
    {
        std::cout << ec_slavecount << " slaves found and configured." << std::endl;

        // set io config
        if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_PRE_OP)
        {
            fprintf(stderr, "Could not set EC_STATE_PRE_OP\n");
            return false;
        }

        ec_config_map(IOmap_);
        ec_configdc();

        std::cout << "Slaves mapped, state to SAFE_OP." << std::endl;
        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

        std::cout << "Request operational state for all slaves" << std::endl;
        expectedWKC_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        std::cout << "Calculated workcounter " << expectedWKC_ << std::endl;
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        /* send one valid process data to make outputs in slaves happy*/

        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        /* request OP state for all slaves */
        ec_writestate(0);
        int chk = 100;
        /* wait for all slaves to reach OP state */
        do
        {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

        if (ec_slave[0].state == EC_STATE_OPERATIONAL)
        {
            std::cout << "Operational state reached for all slaves." << std::endl;
            pdo_transfer_active_ = true;
            return true;
        }
        else
        {
            std::cout << COUT_YELLOW << "Not all slaves reached operational state." << std::endl;
            ec_readstate();
            for (int i = 1; i <= ec_slavecount; i++)
            {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL)
            {
                std::cout << COUT_YELLOW << "Slave " << i << 
                    " State=0x" << ec_slave[i].state  <<" StatusCode=0x" <<ec_slave[i].ALstatuscode <<": "
                    << ec_ALstatuscode2string(ec_slave[i].ALstatuscode) << COUT_RESET  << std::endl; 
            }
            }
        }
    }
    else
    {
      std::cout << COUT_RED << "No slaves found!" << COUT_RESET << std::endl;
    }
  }
  else
  {
      std::cout << COUT_RED << "No socket connection on "<< ether_name_.c_str() <<". Try excecuting the following "
                  "command: sudo setcap 'cap_net_raw=ep cap_sys_nice=eip' $(readlink $(catkin_find "
                  "ethercat_interface ethercat_interface))"
                    << COUT_RESET << std::endl;
  }


  return false;
}

void ati_sensor_lib::show_ati_infomation()
{
  uint8_t force_unit = 0, torque_unit = 0;
  int rdl;
  ec_SDOread(1, 0x2021, 0x2f, FALSE, &rdl, &force_unit, EC_TIMEOUTRXM);
  ec_SDOread(1, 0x2021, 0x30, FALSE, &rdl, &torque_unit, EC_TIMEOUTRXM);
  std::cout << "Force unit: " << FORCE_UNITS[force_unit] << std::endl;
  std::cout << "Torque unit: " << TORQUE_UNITS[torque_unit] << std::endl;

  // send sdo once
  ec_slave[0].outputs[0] = control_code_ & 0x00FF;
  ec_slave[0].outputs[1] = (control_code_ >> 4 )& 0x00FF;
  ec_slave[0].outputs[2] = (control_code_ >> 8 )& 0x00FF;
  ec_slave[0].outputs[3] = (control_code_ >> 12 )& 0x00FF;
}

void ati_sensor_lib::set_bias(bool cmd)
{
  if (cmd)
  {
    control_code_ |= 1UL << 0;
  }
  else
  {
    control_code_ &= ~(1UL << 0);
  }
  
  ec_slave[0].outputs[0] = control_code_ & 0x00FF;
  ec_slave[0].outputs[1] = (control_code_ >> 4 )& 0x00FF;
  ec_slave[0].outputs[2] = (control_code_ >> 8 )& 0x00FF;
  ec_slave[0].outputs[3] = (control_code_ >> 12 )& 0x00FF;

}

void ati_sensor_lib::clear_bias(bool cmd)
{
  if (cmd)
  {
    control_code_ |= 1UL << 2;
  }
  else
  {
    control_code_ &= ~(1UL << 2);
  }
  
  ec_slave[0].outputs[0] = control_code_ & 0x00FF;
  ec_slave[0].outputs[1] = (control_code_ >> 4 )& 0x00FF;
  ec_slave[0].outputs[2] = (control_code_ >> 8 )& 0x00FF;
  ec_slave[0].outputs[3] = (control_code_ >> 12 )& 0x00FF;
}

int* ati_sensor_lib::ati_read()
{  
  return FT_data_;
}

void ati_sensor_lib::ati_write()
{
    int rdl = 4 *6; // 4 bytes per int * 6 ints
    /* end one valid process data to make outputs in slaves happy */
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    // std::cout << "Write control code: " << std::hex << control_code_ << std::endl;
    if (pdo_transfer_active_)
    {            
      ec_SDOread(1, 0x6000, 0x01, TRUE, &rdl, &FT_data_, EC_TIMEOUTRXM);
    }
}

void ati_sensor_lib::read_control_codes()
{
    int read_size  = 4;
    int control_cmd[2];
    ec_SDOread(1, 0x7010, 0x01, FALSE, &read_size, &control_cmd, EC_TIMEOUTRXM);
    std::cout <<std::hex  <<  "read control cmd is: " << control_cmd[0] << std::endl;
}

void ati_sensor_lib::stop_ethercat()
{
    pdo_transfer_active_ = false;
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_INIT,  EC_TIMEOUTSTATE);
    
    ec_close();
    std::cout << "Ethercat stopped." << std::endl;
}


void ati_sensor_lib::start_ethercheck()
{
    thread_statecheck_ = std::thread(&ati_sensor_lib::ecat_statecheck, this);
}

void ati_sensor_lib::ecat_statecheck()
{
  int slave;
  uint8 currentgroup = 0;
  

  while (true)
  {
    if (pdo_transfer_active_ &&
        ((wkc_ < expectedWKC_) || ec_group[currentgroup].docheckstate))
    {
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++)
      {
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            std::cout << COUT_RED << "ERROR : slave "<< slave << " is in SAFE_OP + ERROR, attempting ack."
                       << COUT_RESET << std::endl;
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
          {
            std::cout << COUT_YELLOW << "slave " << slave << " is in SAFE_OP, change to OPERATIONAL." << COUT_RESET << std::endl;
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state > 0)
          {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              std::cout << "MESSAGE : slave" << slave << " reconfigured" << std::endl;
            }
          }
          else if (!ec_slave[slave].islost)
          {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[slave].state)
            {
              ec_slave[slave].islost = TRUE;
              std::cout << COUT_RED << "slave " << slave <<" lost"  << COUT_RESET << std::endl;
            }
          }
        }
        if (ec_slave[slave].islost)
        {
          if (!ec_slave[slave].state)
          {
            if (ec_recover_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              std::cout << "MESSAGE : slave" << slave << " recovered" << std::endl;
            }
          }
          else
          {
            ec_slave[slave].islost = FALSE;
            std::cout << "MESSAGE : slave " << slave << " found"  << std::endl;
          }
        }
      }

      if (ec_group[currentgroup].docheckstate)
      {
        std::cout << "ERROR : not all slaves resumed OPERATIONAL." << std::endl;
      }

    }
    osal_usleep(STATECHECK_PERIOD);
  }
}

// get data from sensor
  double ati_sensor_lib::get_fx()
  {
    return (double)FT_data_[0] * 0.000001;
  }

  double ati_sensor_lib::get_fy()
  {
    return (double)FT_data_[1] * 0.000001;
  }

  double ati_sensor_lib::get_fz()
  {
    return (double)FT_data_[2] * 0.000001;
  }

  double ati_sensor_lib::get_tx()
  {
    return (double)FT_data_[3] * 0.000001;
  }

  double ati_sensor_lib::get_ty()
  {
    return (double)FT_data_[4] * 0.000001;
  }

  double ati_sensor_lib::get_tz()
  {
    return (double)FT_data_[5] * 0.000001;
  }
    
} // namespace ati_sensor
