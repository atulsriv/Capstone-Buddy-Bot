#ifndef ROS_CONTROL__ROBOT_HARDWARE_H
#define ROS_CONTROL__ROBOT_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

//#include <boost/asio.hpp>
//#include <boost/asio/serial_port.hpp>

#include "serial/serial.h"

#include <ros/console.h>
using namespace boost;

#include <iostream>
#include <fstream>
using namespace std;

#include <ros/ros.h>

/// \brief Hardware interface for a robot
class buddybotHardware : public hardware_interface::RobotHW 
{
    public:
        buddybotHardware() : my_serial("/dev/ttyACM0", 57600, serial::Timeout::simpleTimeout(1000)) {

            // Resize vectors
            joint_position_[0] = 0;
            joint_velocity_[0] = 0;
            joint_effort_[0] = 0;
            joint_velocity_command_[0] = 0;
            joint_position_[1] = 0;
            joint_velocity_[1] = 0;
            joint_effort_[1] = 0;
            joint_velocity_command_[1] = 0;

             // Create joint state interface
            hardware_interface::JointStateHandle jointStateHandle_0("left_wheel_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
            joint_state_interface_.registerHandle(jointStateHandle_0);
            hardware_interface::JointStateHandle jointStateHandle_1("right_wheel_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
            joint_state_interface_.registerHandle(jointStateHandle_1);


            // Create velocity joint interface
            hardware_interface::JointHandle jointVelocityHandle_0(joint_state_interface_.getHandle("left_wheel_joint"), &joint_velocity_command_[0]);
            hardware_interface::JointHandle jointVelocityHandle_1(joint_state_interface_.getHandle("right_wheel_joint"), &joint_velocity_command_[1]);
            registerInterface(&joint_state_interface_);

            velocity_joint_interface_.registerHandle(jointVelocityHandle_0);
            velocity_joint_interface_.registerHandle(jointVelocityHandle_1);

            registerInterface(&velocity_joint_interface_);


            cout << "Is the serial port open?";
            if(my_serial.isOpen())
              cout << " Yes." << endl;
            else
              cout << " No." << endl;
        }
        ~buddybotHardware() { }
        void close() {

            char toWrite [30];

            // Create write string to write to arduino
            int n = sprintf (toWrite, "[%d,%d]\n", 0, 0);
            my_serial.write(string(toWrite));

            my_serial.close();
        }
        void read(){ 
            string result;
            result = my_serial.readline();
            //cout << result;

            sscanf(result.c_str(), "[%ld,%ld]", &this->leftEncoder, &this->rightEncoder);

            //cout << "LeftWheel:" << this->leftEncoder;
            //cout << "\tRightWheel:" << this->rightEncoder << endl;

            joint_position_[0] = (double)this->leftEncoder;
            joint_position_[1] = (double)this->rightEncoder;
        }
        void write() {
            // Left and Right Speed come in as rad/sec in the range of 0 to 5 m/s

            int leftSpeed = (int)joint_velocity_command_[0];
            int rightSpeed = (int) joint_velocity_command_[1];

            if (leftSpeed > 100) leftSpeed = 100;
            if (leftSpeed < -100) leftSpeed = -100;
            if (rightSpeed > 100) rightSpeed = 100;
            if (rightSpeed < -100) rightSpeed = -100;

            // The arduino motor driver accepts speeds from 0 to 100, but we dont 
            // want to run at max speed, so a limit of 50 will be applied

            char toWrite [30];

            // Create write string to arduino, multiply speed by 10 and convert to int
            int n = sprintf (toWrite, "[%d,%d]\n", leftSpeed, rightSpeed);
         
            my_serial.write(string(toWrite));

        }

    protected:
        // Interfaces
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        // Shared memory
        double joint_position_[2];
        double joint_velocity_[2];
        double joint_effort_[2];
        double joint_velocity_command_[2];
        long leftEncoder, rightEncoder;

        serial::Serial my_serial;

};

#endif
