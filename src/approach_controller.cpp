#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <cmath>
#include <string>


enum ControllerState
{
    ALIGN,
    APPROACH,
    FINAL_APPROACH,
    STOP
};


class ApproachController
{

public:

    ApproachController()
    {
        target_received = false;
        task_received = false;

        target_x = 0.0;
        target_y = 0.0;

        task_state = "SEARCHING";
        state = STOP;

        last_target_time = ros::Time(0);


        target_sub =
            nh.subscribe(
                "/goal_pose",
                10,
                &ApproachController::targetCallback,
                this
            );


        task_sub =
            nh.subscribe(
                "/task_state",
                10,
                &ApproachController::taskCallback,
                this
            );


        cmd_pub =
            nh.advertise<geometry_msgs::Twist>(
                "/cmd_vel",
                10
            );


        ROS_INFO("Approach controller started");
    }


    void spin()
    {
        ros::Rate rate(20);

        while(ros::ok())
        {
            ros::spinOnce();

            control();

            rate.sleep();
        }
    }


private:

    ros::NodeHandle nh;

    ros::Subscriber target_sub;
    ros::Subscriber task_sub;

    ros::Publisher cmd_pub;


    double target_x;
    double target_y;

    bool target_received;
    bool task_received;

    std::string task_state;

    ros::Time last_target_time;

    ControllerState state;


    /*
     * 参数
     */

    const double ALIGN_THRESHOLD = 0.25;   // rad ≈ 14°
    const double TARGET_TIMEOUT  = 0.5;    // s

    const double MAX_ALIGN_W = 0.5;
    const double MAX_MOVE_W  = 0.20;


    void targetCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg
    )
    {
        /*
         * 这里强制要求 /goal_pose 位于 base frame。
         *
         * 后续绝对不要拿 base frame 的 x/y
         * 与 odom frame 的位置或 yaw 直接相减。
         */

        if(msg->header.frame_id != "base")
        {
            ROS_WARN_THROTTLE(
                1,
                "Ignored goal_pose: expected frame 'base', got '%s'",
                msg->header.frame_id.c_str()
            );

            return;
        }


        target_x = msg->pose.position.x;
        target_y = msg->pose.position.y;

        target_received = true;

        last_target_time = ros::Time::now();
    }


    void taskCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        task_state = msg->data;
        task_received = true;
    }


    double limit(
        double value,
        double max_abs
    )
    {
        if(value > max_abs)
            return max_abs;

        if(value < -max_abs)
            return -max_abs;

        return value;
    }


    void stopRobot()
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;

        cmd.angular.z = 0.0;

        cmd_pub.publish(cmd);

        state = STOP;
    }


    void control()
    {
        /*
         * 任何条件不满足，都停车。
         */

        if(!task_received)
        {
            stopRobot();
            return;
        }


        /*
         * 只有 APPROACHING 状态允许运动。
         */

        if(task_state != "APPROACHING")
        {
            stopRobot();
            return;
        }


        if(!target_received)
        {
            stopRobot();
            return;
        }


        /*
         * 独立安全保护：
         * 即使 task_manager 出现异常，
         * 目标超过 0.5s 没更新也停车。
         */

        double target_age =
            (ros::Time::now() - last_target_time).toSec();


        if(target_age > TARGET_TIMEOUT)
        {
            ROS_WARN_THROTTLE(
                1,
                "Target timeout -> STOP"
            );

            stopRobot();

            return;
        }


        /*
         * IMPORTANT:
         *
         * /goal_pose 已经位于 base frame。
         *
         * 所以：
         *
         * target_x = 目标相对机器人的前后距离
         * target_y = 目标相对机器人的横向距离
         *
         * 不需要再减 odom 中的位置或 yaw。
         */

        double distance =
            std::sqrt(
                target_x * target_x +
                target_y * target_y
            );


        /*
         * 目标相对于机器人正前方的角度。
         *
         * 因为目标已经在 base frame，
         * atan2(y,x) 本身就是角度误差。
         */

        double angle_error =
            std::atan2(
                target_y,
                target_x
            );


        geometry_msgs::Twist cmd;


        /*
         * 第一阶段：
         * 偏角较大 -> 原地调整方向
         */

        if(std::fabs(angle_error) > ALIGN_THRESHOLD)
        {
            state = ALIGN;

            cmd.linear.x = 0.0;

            cmd.angular.z =
                limit(
                    0.8 * angle_error,
                    MAX_ALIGN_W
                );
        }


        /*
         * 第二阶段：
         * 距离 > 1m -> 正常靠近
         */

        else if(distance > 1.0)
        {
            state = APPROACH;

            cmd.linear.x = 0.25;

            cmd.angular.z =
                limit(
                    0.25 * angle_error,
                    MAX_MOVE_W
                );
        }


        /*
         * 第三阶段：
         * 进入近距离 -> 分级减速
         *
         * READY 的最终判断仍然交给 task_manager。
         */

        else
        {
            state = FINAL_APPROACH;


            if(distance > 0.70)
            {
                cmd.linear.x = 0.12;
            }
            else
            {
                cmd.linear.x = 0.05;
            }


            cmd.angular.z =
                limit(
                    0.20 * angle_error,
                    0.15
                );
        }


        cmd_pub.publish(cmd);


        ROS_INFO_THROTTLE(
            1,
            "task=%s state=%d dist=%.3f bearing=%.3f vx=%.2f wz=%.2f",
            task_state.c_str(),
            state,
            distance,
            angle_error,
            cmd.linear.x,
            cmd.angular.z
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