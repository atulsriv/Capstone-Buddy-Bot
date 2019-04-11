#include <buddybot_control/buddybot_control.h>
#include <iostream>

void buddybot_robot::moveWheels(double leftSpeed, double rightSpeed) {
	// asio::io_service io;
	// asio::serial_port port(io);
 
	// port.open("COM3");
	// port.set_option(asio::serial_port_base::baud_rate(115200));
 
	// char c = 'a';
 
	// // Read 1 character into c, this will block
	// // forever if no character arrives.
	// asio::write(port, asio::buffer(&c,1));
 
	// port.close();
	std::cout << "I CALLED MOVE WHEELS FUNCTION" << std::endl;
}
