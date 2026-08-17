#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <cmath>


class GoalChecker
{

public:

    GoalChecker()
    {

        target_received=false;


        target_sub =
            nh.subscribe(
                "/target_pose",
                10,
                &GoalChecker::targetCallback,
                this
            );


        odom_sub =
            nh.subscribe(
                "/odom",
                10,
                &GoalChecker::odomCallback,
                this
            );


        marker_pub =
            nh.advertise<visualization_msgs::Marker>(
                "/target_marker",
                1
            );

    }



    void spin()
    {

        ros::Rate rate(10);


        while(ros::ok())
        {

            ros::spinOnce();


            if(target_received)
            {
                publishMarker();
                checkDistance();
            }


            rate.sleep();

        }

    }


private:


    ros::NodeHandle nh;


    ros::Subscriber target_sub;
    ros::Subscriber odom_sub;


    ros::Publisher marker_pub;



    double target_x;
    double target_y;


    double robot_x;
    double robot_y;


    bool target_received;



    void targetCallback(
        const geometry_msgs::Point::ConstPtr& msg
    )
    {

        target_x=msg->x;
        target_y=msg->y;

        target_received=true;

    }




    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {

        robot_x=
        msg->pose.pose.position.x;


        robot_y=
        msg->pose.pose.position.y;

    }




    void checkDistance()
    {

        double dx =
        target_x-robot_x;


        double dy =
        target_y-robot_y;


        double distance =
        sqrt(
            dx*dx+
            dy*dy
        );


        if(distance<0.35)
        {

            ROS_INFO_THROTTLE(
                1,
                "ARRIVED | distance %.2f m",
                distance
            );

        }
        else
        {

            ROS_INFO_THROTTLE(
                1,
                "Distance %.2f m",
                distance
            );

        }

    }





    void publishMarker()
    {


        visualization_msgs::Marker marker;


        marker.header.frame_id="odom";

        marker.header.stamp =
            ros::Time::now();


        marker.ns="goal";

        marker.id=0;


        marker.type =
            visualization_msgs::Marker::SPHERE;


        marker.action =
            visualization_msgs::Marker::ADD;


        marker.pose.position.x =
            target_x;


        marker.pose.position.y =
            target_y;


        marker.pose.position.z =
            0.2;


        marker.scale.x=0.2;
        marker.scale.y=0.2;
        marker.scale.z=0.2;


        marker.color.a=1.0;
        marker.color.r=1.0;


        marker_pub.publish(marker);

    }


};



int main(
int argc,
char **argv)
{

    ros::init(
        argc,
        argv,
        "goal_checker"
    );


    GoalChecker node;


    node.spin();


    return 0;

}
