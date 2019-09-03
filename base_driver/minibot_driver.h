
#ifndef MINIBOT_DRIVER_H_
#define MINIBOT_DRIVER_H_

#include <ros/ros.h>

// commands
#include <minibot_msgs/BaseControl.h>
#include <minibot_msgs/LEDConfig.h>
#include <minibot_msgs/GPIOConfig.h>
#include <minibot_msgs/ADCConfig.h>
#include <minibot_msgs/DACConfig.h>
#include <minibot_msgs/MotorOperation.h>


// responses
#include <minibot_msgs/IMU.h>
#include <minibot_msgs/Battery.h>
#include <minibot_msgs/GPInput.h>
#include <minibot_msgs/ADC.h>
#include <minibot_msgs/Firmware.h>
#include <minibot_msgs/Status.h>

#include <string.h>
#include <iostream>

#include "serial/serial.h"

#include <boost/crc.hpp>

typedef std::vector<uint8_t> Buffer;

typedef boost::crc_optimal<16, 0x8005, 0xffff, 0, true, true> CRC;


// ROS services
ros::Subscriber base_control_sub;
ros::Subscriber firmware_enquiry_sub;
ros::Subscriber status_enquiry_sub;
ros::Subscriber miniarm_sub;


ros::Publisher  IMUSensors_pub;
//ros::Publisher  battery_pub;
ros::Publisher  firmware_pub;
//ros::Publisher  status_pub;

ros::Timer  timer_;


//ROS callbacks
void timerCallback(const ros::TimerEvent& event);

void baseControlCallback(const minibot_msgs::BaseControlConstPtr &base_control);
void miniarmCallback(const minibot_msgs::MotorOperationConstPtr &mini_arm);



#endif

