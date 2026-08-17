#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class YawErrorTest
{

public:

    YawErrorTest()
    {

        odom_sub =
            nh.subscribe(
                "/odom",
                10,
                &YawErrorTest::odomCallback,
                this
            );


        goal_sub =
            nh.subscribe(
                "/goal_pose",
                10,
                &YawErrorTest::goalCallback,
                this
            );

    }


private:

    ros::NodeHandle nh;


    ros::Subscriber odom_sub;
    ros::Subscriber goal_sub;


    double current_yaw = 0;

    double target_yaw = 0;


    bool has_goal = false;


    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {

        tf2::Quaternion q(

            msg->pose.pose.orientation.x,

            msg->pose.pose.orientation.y,

            msg->pose.pose.orientation.z,

            msg->pose.pose.orientation.w

        );


        double roll;
        double pitch;


        tf2::Matrix3x3(q)
            .getRPY(
                roll,
                pitch,
                current_yaw
            );


        printResult();

    }



    void goalCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg
    )
    {

        tf2::Quaternion q(

            msg->pose.orientation.x,

            msg->pose.orientation.y,

            msg->pose.orientation.z,

            msg->pose.orientation.w

        );


        double roll;
        double pitch;


        tf2::Matrix3x3(q)
            .getRPY(
                roll,
                pitch,
                target_yaw
            );


        has_goal = true;

    }



    void printResult()
    {

        if(!has_goal)
            return;


        double error =
            target_yaw - current_yaw;


        ROS_INFO_THROTTLE(
            1,
            "current=%.2f deg target=%.2f deg error=%.2f deg",
            current_yaw*180/3.1415926,
            target_yaw*180/3.1415926,
            error*180/3.1415926
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
        "yaw_error_test"
    );


    YawErrorTest node;


    ros::spin();


    return 0;

}
