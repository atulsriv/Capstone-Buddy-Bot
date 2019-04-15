#include <buddybot_hardware_interface/buddybot_hardware_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "buddybot_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    buddybot_hardware_interface::buddybotHardwareInterface buddybot(nh);
    ros::spin();
    ROS_DEBUG("THIS TEST WORKED");
    return 0;
}
