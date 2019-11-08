//This is the ROS standard of "hello, world" program//

//This header defined standard ROS classed//
#include<ros/ros.h>

int main(int argc, char **argv){
//Initialize the ROS system
ros::init(argc, argv, "hello_ros");

//Establish this program as ROS node
ros::NodeHandle nh;

//Send some output as log message
ROS_INFO_STREAM("Hello, ROS!");
}
