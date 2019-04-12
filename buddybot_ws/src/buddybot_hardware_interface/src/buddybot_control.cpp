#include <buddybot_control/buddybot_control.h>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <ros/console.h>
using namespace boost;

void buddybot_robot::moveWheels(double leftSpeed, double rightSpeed) {

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
	ROS_DEBUG("THIS TEST WORKED");
}
