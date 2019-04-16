#include <buddybot_hardware_interface/buddybot_hardware.h>

int main(int argc, char** argv)
{

	ros::init(argc, argv, "buddybot_hardware_interface");
	ros::NodeHandle nh;

	buddybotHardware buddybot;
	controller_manager::ControllerManager cm(&buddybot, nh);


	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Time ts = ros::Time::now();

	ros::Rate rate(50);
	while (ros::ok())
	{
	 ros::Duration d = ros::Time::now() - ts;
	 ts = ros::Time::now();
	 buddybot.read();
	 cm.update(ts, d);
	 buddybot.write();
	 rate.sleep();
	}

	spinner.stop();

	return 0;
}
