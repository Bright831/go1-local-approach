#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <tf/tf.h>

#include <cmath>
#include <string>


class FakeButtonVision
{
public:

    FakeButtonVision()
        : pnh_("~")
    {
        odom_received_ = false;

        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;


        /*
         * Subscribe to the Go1 odometry.
         */
        odom_sub_ =
            nh_.subscribe(
                "/odom",
                10,
                &FakeButtonVision::odomCallback,
                this
            );


        /*
         * Publish simulated vision outputs.
         */
        valid_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/vision/target_valid",
                10
            );


        type_pub_ =
            nh_.advertise<std_msgs::String>(
                "/vision/target_type",
                10
            );


        center_pub_ =
            nh_.advertise<std_msgs::Float32>(
                "/vision/target_center_x",
                10
            );


        confidence_pub_ =
            nh_.advertise<std_msgs::Float32>(
                "/vision/target_confidence",
                10
            );


        ROS_INFO(
            "Dynamic fake button vision started"
        );
    }


    void spin()
    {
        ros::Rate rate(10);


        while(ros::ok())
        {
            ros::spinOnce();


            if(odom_received_)
            {
                publishVision();
            }


            rate.sleep();
        }
    }


private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;


    ros::Subscriber odom_sub_;

    ros::Publisher valid_pub_;
    ros::Publisher type_pub_;
    ros::Publisher center_pub_;
    ros::Publisher confidence_pub_;


    double robot_x_;
    double robot_y_;
    double robot_yaw_;

    bool odom_received_;


    double normalizeAngle(double angle)
    {
        while(angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }


        while(angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }


        return angle;
    }


    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {
        robot_x_ =
            msg->pose.pose.position.x;


        robot_y_ =
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
                robot_yaw_
            );


        odom_received_ = true;
    }


    void publishVision()
    {
        /*
         * ==============================
         * Simulation parameters that can be modified at runtime.
         * ==============================
         */

        double target_world_x;
        double target_world_y;

        double image_width;
        double hfov_deg;

        double confidence;

        bool enabled;

        std::string target_type;


        pnh_.param<double>(
            "target_world_x",
            target_world_x,
            3.0
        );


        pnh_.param<double>(
            "target_world_y",
            target_world_y,
            1.0
        );


        pnh_.param<double>(
            "image_width",
            image_width,
            640.0
        );


        pnh_.param<double>(
            "hfov_deg",
            hfov_deg,
            60.0
        );


        pnh_.param<double>(
            "confidence",
            confidence,
            0.90
        );


        pnh_.param<bool>(
            "enabled",
            enabled,
            true
        );


        pnh_.param<std::string>(
            "target_type",
            target_type,
            "elevator_call_button"
        );


        /*
         * =========================================
         * Coordinate-frame convention.
         *
         * robot_x/y:
         * odom frame
         *
         * target_world_x/y:
         * The simulated button position is also defined in the odom frame.
         *
         * Therefore, the relative position can be computed directly here.
         * =========================================
         */

        const double dx =
            target_world_x - robot_x_;


        const double dy =
            target_world_y - robot_y_;


        const double distance =
            std::sqrt(
                dx * dx +
                dy * dy
            );


        /*
         * Absolute direction from the robot to the button in the odom frame.
         */
        const double target_world_angle =
            std::atan2(
                dy,
                dx
            );


        /*
         * Direction of the button relative to the Go1 forward axis.
         */
        const double relative_angle =
            normalizeAngle(
                target_world_angle
                -
                robot_yaw_
            );


        /*
         * =========================================
         * Camera model.
         *
         * Image convention:
         *
         * 0 -------- 320 -------- 640
         * Left        Centre        Right
         *
         * ROS：
         *
         * relative_angle > 0
         * A negative horizontal angle means the target is to the robot's left.
         *
         * Therefore, the corresponding image x coordinate should be less than 320.
         * =========================================
         */

        const double hfov_rad =
            hfov_deg
            *
            M_PI
            /
            180.0;


        const double image_center =
            image_width
            /
            2.0;


        const double fx =
            image_center
            /
            std::tan(
                hfov_rad / 2.0
            );


        double center_x =
            image_center
            -
            fx
            *
            std::tan(
                relative_angle
            );


        /*
         * Check whether the target remains inside the camera field of view.
         */
        bool visible =
            std::fabs(relative_angle)
            <=
            hfov_rad / 2.0;


        bool valid =
            enabled
            &&
            visible;


        /*
         * Clamp the pixel coordinate to the valid image range.
         */
        if(center_x < 0.0)
        {
            center_x = 0.0;
        }


        if(center_x > image_width - 1.0)
        {
            center_x =
                image_width - 1.0;
        }


        /*
         * Publish ROS messages.
         */

        std_msgs::Bool valid_msg;

        std_msgs::String type_msg;

        std_msgs::Float32 center_msg;

        std_msgs::Float32 confidence_msg;


        valid_msg.data =
            valid;


        type_msg.data =
            target_type;


        center_msg.data =
            static_cast<float>(
                center_x
            );


        confidence_msg.data =
            static_cast<float>(
                confidence
            );


        valid_pub_.publish(
            valid_msg
        );


        type_pub_.publish(
            type_msg
        );


        center_pub_.publish(
            center_msg
        );


        confidence_pub_.publish(
            confidence_msg
        );


        ROS_INFO_THROTTLE(
            0.5,
            "Button | dist=%.2f m rel_angle=%.2f deg center_x=%.1f valid=%s",
            distance,
            relative_angle * 180.0 / M_PI,
            center_x,
            valid ? "true" : "false"
        );
    }
};


int main(
    int argc,
    char** argv
)
{
    ros::init(
        argc,
        argv,
        "fake_button_vision"
    );


    FakeButtonVision node;


    node.spin();


    return 0;
}