#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>

#include <cmath>


class FakeVision
{

public:


    FakeVision()
    {

        odom_received = false;


        /*
         * 虚拟按钮世界坐标
         *
         * 模拟:
         * 电梯按钮位置
         */

        button_x = 3.0;
        button_y = 0.5;
        button_z = 1.0;



        odom_sub =
        nh.subscribe(
            "/odom",
            10,
            &FakeVision::odomCallback,
            this
        );



        pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>(
            "/detected_object_pose",
            10
        );



        ROS_INFO(
            "Fake vision v2 started"
        );

    }




    void spin()
    {

        ros::Rate rate(10);


        while(ros::ok())
        {

            ros::spinOnce();


            if(odom_received)
            {
                publishObject();
            }


            rate.sleep();

        }

    }




private:


    ros::NodeHandle nh;


    ros::Subscriber odom_sub;


    ros::Publisher pose_pub;



    double robot_x;
    double robot_y;
    double robot_yaw;



    double button_x;
    double button_y;
    double button_z;



    bool odom_received;





    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {


        robot_x =
        msg->pose.pose.position.x;


        robot_y =
        msg->pose.pose.position.y;



        tf::Quaternion q(

            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w

        );



        double roll;
        double pitch;


        tf::Matrix3x3(q)
        .getRPY(
            roll,
            pitch,
            robot_yaw
        );



        odom_received=true;

    }







    void publishObject()
    {


        /*
         *
         * 世界坐标中的按钮
         *
         */

        double dx =
        button_x - robot_x;


        double dy =
        button_y - robot_y;



        /*
         *
         * world
         *      ↓
         *      base
         *
         * 坐标转换
         *
         */


        double object_x =
        cos(robot_yaw)*dx
        +
        sin(robot_yaw)*dy;



        double object_y =
        -sin(robot_yaw)*dx
        +
        cos(robot_yaw)*dy;



        geometry_msgs::PoseStamped msg;



        msg.header.stamp =
        ros::Time::now();


        /*
         *
         * fake camera输出
         *
         */

        msg.header.frame_id =
        "base";



        msg.pose.position.x =
        object_x;


        msg.pose.position.y =
        object_y;


        msg.pose.position.z =
        button_z;



        /*
         *
         * 按钮朝向
         *
         */

        tf::Quaternion q;


        q.setRPY(
            0,
            0,
            M_PI
        );


        msg.pose.orientation.x =
        q.x();


        msg.pose.orientation.y =
        q.y();


        msg.pose.orientation.z =
        q.z();


        msg.pose.orientation.w =
        q.w();



        pose_pub.publish(msg);



        ROS_INFO_THROTTLE(
            1,
            "Fake button relative position: %.2f %.2f",
            object_x,
            object_y
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
        "fake_vision"
    );


    FakeVision node;


    node.spin();


    return 0;

}