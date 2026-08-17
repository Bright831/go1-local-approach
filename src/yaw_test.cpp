#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class YawTest
{

public:

    YawTest()
    {

        sub = nh.subscribe(
            "/odom",
            10,
            &YawTest::odomCallback,
            this
        );

    }


private:

    ros::NodeHandle nh;

    ros::Subscriber sub;


    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {

        // 获取四元数

        double x =
            msg->pose.pose.orientation.x;

        double y =
            msg->pose.pose.orientation.y;

        double z =
            msg->pose.pose.orientation.z;

        double w =
            msg->pose.pose.orientation.w;


        tf2::Quaternion q(
            x,
            y,
            z,
            w
        );


        double roll;
        double pitch;
        double yaw;


        tf2::Matrix3x3(q)
            .getRPY(
                roll,
                pitch,
                yaw
            );


        ROS_INFO_THROTTLE(
            1,
            "Robot yaw = %.3f rad (%.1f degree)",
            yaw,
            yaw*180.0/3.1415926
        );

    }

};


int main(
    int argc,
    char **argv
)
{

    ros::init(
        argc,
        argv,
        "yaw_test"
    );


    YawTest node;


    ros::spin();


    return 0;

}
