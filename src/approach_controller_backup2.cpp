#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>

#include <cmath>
#include <algorithm>


enum ControllerState
{
    APPROACH,
    ALIGN,
    FINAL_APPROACH,
    STOP
};



class ApproachController
{

public:

    ApproachController()
    {

        target_received = false;
        odom_received = false;


        target_x = 0;
        target_y = 0;
        target_yaw = 0;


        robot_x = 0;
        robot_y = 0;
        robot_yaw = 0;


        state = ALIGN;



        target_sub =
        nh.subscribe(
            "/goal_pose",
            10,
            &ApproachController::targetCallback,
            this
        );


        odom_sub =
        nh.subscribe(
            "/odom",
            10,
            &ApproachController::odomCallback,
            this
        );



        cmd_pub =
        nh.advertise<geometry_msgs::Twist>(
            "/cmd_vel",
            10
        );


        ROS_INFO(
            "Approach controller started"
        );

    }





    void spin()
    {

        ros::Rate rate(20);


        while(ros::ok())
        {

            ros::spinOnce();


            if(target_received && odom_received)
            {
                control();
            }


            rate.sleep();

        }

    }





private:


    ros::NodeHandle nh;


    ros::Subscriber target_sub;
    ros::Subscriber odom_sub;


    ros::Publisher cmd_pub;



    double target_x;
    double target_y;
    double target_yaw;


    double robot_x;
    double robot_y;
    double robot_yaw;



    bool target_received;
    bool odom_received;



    ControllerState state;





    void targetCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg
    )
    {

        target_x =
        msg->pose.position.x;


        target_y =
        msg->pose.position.y;



        // 获取目标朝向

        tf::Quaternion q(

            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w

        );


        double roll;
        double pitch;


        tf::Matrix3x3(q)
        .getRPY(
            roll,
            pitch,
            target_yaw
        );



        target_received = true;



        ROS_INFO_THROTTLE(
            2,
            "Target x=%.2f y=%.2f yaw=%.2f",
            target_x,
            target_y,
            target_yaw
        );

    }






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





    double normalizeAngle(double angle)
    {

        while(angle > M_PI)
            angle -= 2*M_PI;


        while(angle < -M_PI)
            angle += 2*M_PI;


        return angle;

    }






    double limit(
        double value,
        double max_value
    )
    {

        if(value > max_value)
            return max_value;


        if(value < -max_value)
            return -max_value;


        return value;

    }







    void control()
    {

        geometry_msgs::Twist cmd;



        double dx =
        target_x - robot_x;


        double dy =
        target_y - robot_y;



        double distance =
        sqrt(
            dx*dx +
            dy*dy
        );



        /*
         * 目标方向
         *
         * 机器人 -> 目标
         */

        double target_angle =
        atan2(
            dy,
            dx
        );



        double angle_error =
        normalizeAngle(
            target_angle - robot_yaw
        );





        /*
         * 状态机
         *
         * 优先调整方向
         */


        if(fabs(angle_error) > 0.3)
        {

            state = ALIGN;

        }


        else if(distance > 1.0)
        {

            state = APPROACH;

        }


        else if(distance > 0.35)
        {

            state = FINAL_APPROACH;

        }


        else
        {

            state = STOP;

        }







        switch(state)
        {


        case ALIGN:


            cmd.linear.x = 0;


            cmd.angular.z =
            limit(
                0.8 * angle_error,
                0.8
            );


            break;





        case APPROACH:


            cmd.linear.x = 0.25;


            cmd.angular.z =
            limit(
                0.5 * angle_error,
                0.5
            );


            break;





        case FINAL_APPROACH:


            cmd.linear.x = 0.05;


            cmd.angular.z =
            limit(
                0.3 * angle_error,
                0.3
            );


            break;





        case STOP:


            cmd.linear.x = 0;
            cmd.angular.z = 0;


            break;

        }



        cmd_pub.publish(cmd);



        ROS_INFO_THROTTLE(
            1,
            "state=%d distance=%.2f angle_error=%.2f",
            state,
            distance,
            angle_error
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
        "approach_controller"
    );


    ApproachController controller;


    controller.spin();


    return 0;

}