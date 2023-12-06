#include "ros/ros.h"
#include "marker_package/Waypoint.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_publisher");

    ros::NodeHandle n;

    ros::Publisher waypoint_pub = n.advertise<marker_package::Waypoint>("/waypoint", 1000);

    ros::Rate loop_rate(10); // Control the publishing rate

    while (ros::ok())
    {
        marker_package::Waypoint msg;
        msg.num = 1; // Set the waypoint number

        waypoint_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
// To run the node: use: rosrun marker_package waypoint_publisher