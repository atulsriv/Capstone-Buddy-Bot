#ifndef ROS_CONTROL__ROBOT_HARDWARE_H
#define ROS_CONTROL__ROBOT_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

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
        buddybotHardware(){

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
        }
        ~buddybotHardware() { }
        void read(){ }
        void write() {
            // Left and Right Speed come in as rad/sec in the range of 0 to 5 m/s

            // The arduino motor driver accepts speeds from 0 to 100, but we dont 
            // want to run at max speed, so a limit of 50 will be applied

            /*
                asio::io_service io;
                asio::serial_port port(io);
             
                port.open("/dev/ttyACM0");
                port.set_option(asio::serial_port_base::baud_rate(57600));
             
                char toWrie [30];

                // Create write string to arduino, multiply speed by 10 and convert to int
                int n = sprintf (toWrite, "[%d,%d]\n", (int) (10* leftSpeed), (int) (10* rightSpeed));
             
                // Read 1 character into c, this will block
                // forever if no character arrives.
                asio::write(port, asio::buffer(&toWrite, n));
             
                port.close();
            */
            double leftSpeed = joint_velocity_command_[0];
            double rightSpeed = joint_velocity_command_[1];

            char toWrite [100];

            // Create write string to arduino, multiply speed by 10 and convert to int
            int n = std::sprintf(toWrite, "[%f,%f]\n", leftSpeed, rightSpeed);

            ofstream myfile;
            myfile.open ("/home/nick/Documents/Capstone-Buddy-Bot/buddybot_ws/src/buddybot_hardware_interface/src/test.txt");
            myfile << toWrite << std::endl;
            myfile.close();
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


}; // class

#endif