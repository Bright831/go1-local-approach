#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class VisionToGoal
{

public:

    VisionToGoal():
        tf_listener(tf_buffer)
    {

        vision_sub =
            nh.subscribe(
                "/detected_object_pose",
                10,
                &VisionToGoal::visionCallback,
                this
            );


        goal_pub =
            nh.advertise<geometry_msgs::PoseStamped>(
                "/goal_pose",
                10
            );


        ROS_INFO(
            "Vision to goal node started"
        );

    }



private:


    ros::NodeHandle nh;


    ros::Subscriber vision_sub;

    ros::Publisher goal_pub;



    tf2_ros::Buffer tf_buffer;

    tf2_ros::TransformListener tf_listener;



    void visionCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg
    )
    {

        geometry_msgs::PoseStamped target_base;


        try
        {

            /*
             *
             * camera_link
             *
             *      |
             *      |
             *
             *     base
             *
             */


            tf_buffer.transform(

                *msg,

                target_base,

                "base",

                ros::Duration(1.0)

            );


        }


        catch(tf2::TransformException &ex)
        {

            ROS_WARN(
                "TF error: %s",
                ex.what()
            );

            return;

        }



        /*
         * 输出目标(base坐标系)
         */


        goal_pub.publish(target_base);



        ROS_INFO_THROTTLE(
            1,
            "Goal in base frame: x=%.2f y=%.2f z=%.2f",
            target_base.pose.position.x,
            target_base.pose.position.y,
            target_base.pose.position.z
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
        "vision_to_goal"
    );


    VisionToGoal node;


    ros::spin();


    return 0;

}