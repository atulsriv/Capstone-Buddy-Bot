#ifndef BUDDYBOT_CONTROL_H
#define BUDDYBOT_CONTROL_H

#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 

using namespace boost;

class buddybot_robot {
	public:
		void moveWheels(double, double);
};

#endif