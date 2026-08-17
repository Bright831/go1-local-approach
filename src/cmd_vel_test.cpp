#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_test");
    ros::NodeHandle nh;

    ros::Publisher pub =
        nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate rate(10);

    ROS_INFO("cmd_vel test started");

    int count = 0;

    while(ros::ok())
    {
        geometry_msgs::Twist cmd;

        if(count < 30)
        {
            // 前进3秒
            cmd.linear.x = 0.1;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
        }
        else
        {
            // 停止
            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.angular.z = 0;
        }

        pub.publish(cmd);

        count++;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
