#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <cmath>
#include <string>


enum ButtonControllerState
{
    WAIT_TARGET,
    SEARCH,
    ALIGN,
    FAR_APPROACH,
    DEPTH_APPROACH,
    READY,
    VISION_LOST,
    DEPTH_LOST
};


class ButtonApproachController
{
public:

    ButtonApproachController()
        : pnh_("~")
    {
        target_valid_ = false;
        target_type_ = "";
        target_center_x_ = 0.0;
        target_confidence_ = 0.0;

        vision_received_ = false;

        /*
         * 从程序启动到现在，
         * 是否至少成功识别过一次有效按钮。
         *
         * false:
         *   允许 SEARCH
         *
         * true:
         *   后续视觉丢失只 STOP，不自动搜索
         */
        has_seen_valid_target_ = false;

        depth_received_ = false;
        depth_ever_acquired_ = false;

        front_distance_ = 0.0;

        state_ = WAIT_TARGET;


        /*
         * ========================================
         * Vision parameters
         * ========================================
         */

        pnh_.param<double>(
            "image_width",
            image_width_,
            640.0
        );

        pnh_.param<double>(
            "center_threshold_px",
            center_threshold_px_,
            40.0
        );

        /*
         * 偏差超过这个值时，
         * 停止前进并重新原地 ALIGN。
         */
        pnh_.param<double>(
            "realign_threshold_px",
            realign_threshold_px_,
            80.0
        );

        pnh_.param<double>(
            "min_confidence",
            min_confidence_,
            0.60
        );

        pnh_.param<double>(
            "vision_timeout",
            vision_timeout_,
            0.5
        );


        /*
         * ========================================
         * Depth parameters
         * ========================================
         */

        pnh_.param<double>(
            "depth_timeout",
            depth_timeout_,
            0.5
        );

        pnh_.param<double>(
            "max_valid_depth",
            max_valid_depth_,
            1.50
        );

        /*
         * 当前 Demo 的预设交互停止距离。
         *
         * 后续机械臂接入后再根据机械臂
         * workspace 重新标定。
         */
        pnh_.param<double>(
            "stop_distance",
            stop_distance_,
            0.50
        );


        /*
         * ========================================
         * Motion parameters
         * ========================================
         */

        /*
         * SEARCH 原地旋转速度。
         *
         * ROS angular.z:
         *   正值 = 左转 / CCW
         *
         * 默认 0.15 rad/s，
         * 约 8.6 deg/s。
         */
        pnh_.param<double>(
            "search_angular_speed",
            search_angular_speed_,
            0.25
        );

        pnh_.param<double>(
            "far_speed",
            far_speed_,
            0.08
        );

        pnh_.param<double>(
            "medium_speed",
            medium_speed_,
            0.08
        );

        pnh_.param<double>(
            "near_speed",
            near_speed_,
            0.04
        );

        pnh_.param<double>(
            "kp_angular_align",
            kp_angular_align_,
            0.002
        );

        pnh_.param<double>(
            "kp_angular_move",
            kp_angular_move_,
            0.001
        );

        pnh_.param<double>(
            "max_align_angular",
            max_align_angular_,
            0.35
        );

        pnh_.param<double>(
            "max_move_angular",
            max_move_angular_,
            0.12
        );

        /*
         * RGB图像：
         *
         * x增大 = 目标在图像右边
         *
         * ROS angular.z：
         *
         * 正 = 左转
         * 负 = 右转
         */
        pnh_.param<double>(
            "yaw_sign",
            yaw_sign_,
            -1.0
        );


        /*
         * ========================================
         * Subscribers
         * ========================================
         */

        valid_sub_ =
            nh_.subscribe(
                "/vision/target_valid",
                10,
                &ButtonApproachController::validCallback,
                this
            );

        type_sub_ =
            nh_.subscribe(
                "/vision/target_type",
                10,
                &ButtonApproachController::typeCallback,
                this
            );

        center_sub_ =
            nh_.subscribe(
                "/vision/target_center_x",
                10,
                &ButtonApproachController::centerCallback,
                this
            );

        confidence_sub_ =
            nh_.subscribe(
                "/vision/target_confidence",
                10,
                &ButtonApproachController::confidenceCallback,
                this
            );

        depth_sub_ =
            nh_.subscribe(
                "/front_distance",
                10,
                &ButtonApproachController::depthCallback,
                this
            );


        /*
         * ========================================
         * Publisher
         * ========================================
         */

        cmd_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                "/cmd_vel",
                10
            );


        ROS_INFO(
            "Button approach controller with SEARCH started"
        );

        ROS_INFO(
            "Search angular speed = %.2f rad/s",
            search_angular_speed_
        );

        ROS_INFO(
            "Stop distance = %.2f m",
            stop_distance_
        );
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

        stopRobot();
    }


private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;


    ros::Subscriber valid_sub_;
    ros::Subscriber type_sub_;
    ros::Subscriber center_sub_;
    ros::Subscriber confidence_sub_;
    ros::Subscriber depth_sub_;

    ros::Publisher cmd_pub_;


    /*
     * ========================================
     * Vision data
     * ========================================
     */

    bool target_valid_;
    std::string target_type_;
    double target_center_x_;
    double target_confidence_;

    bool vision_received_;

    /*
     * 是否曾经识别成功过一次按钮。
     *
     * 用于区分：
     *
     * 初始没找到目标 -> SEARCH
     *
     * 已经找到过目标后丢失 -> STOP
     */
    bool has_seen_valid_target_;

    ros::Time last_valid_time_;
    ros::Time last_type_time_;
    ros::Time last_center_time_;
    ros::Time last_confidence_time_;


    /*
     * ========================================
     * Depth data
     * ========================================
     */

    double front_distance_;

    bool depth_received_;
    bool depth_ever_acquired_;

    ros::Time last_depth_time_;


    ButtonControllerState state_;


    /*
     * ========================================
     * Parameters
     * ========================================
     */

    double image_width_;
    double center_threshold_px_;
    double realign_threshold_px_;

    double min_confidence_;
    double vision_timeout_;

    double depth_timeout_;
    double max_valid_depth_;
    double stop_distance_;

    double search_angular_speed_;

    double far_speed_;
    double medium_speed_;
    double near_speed_;

    double kp_angular_align_;
    double kp_angular_move_;

    double max_align_angular_;
    double max_move_angular_;

    double yaw_sign_;


    /*
     * ========================================
     * Callbacks
     * ========================================
     */

    void validCallback(
        const std_msgs::Bool::ConstPtr& msg
    )
    {
        target_valid_ = msg->data;

        vision_received_ = true;

        last_valid_time_ = ros::Time::now();
    }


    void typeCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        target_type_ = msg->data;

        last_type_time_ = ros::Time::now();
    }


    void centerCallback(
        const std_msgs::Float32::ConstPtr& msg
    )
    {
        target_center_x_ = msg->data;

        last_center_time_ = ros::Time::now();
    }


    void confidenceCallback(
        const std_msgs::Float32::ConstPtr& msg
    )
    {
        target_confidence_ = msg->data;

        last_confidence_time_ = ros::Time::now();
    }


    void depthCallback(
        const std_msgs::Float32::ConstPtr& msg
    )
    {
        /*
         * /front_distance 是深度相机中央 ROI
         * 得到的标量距离。
         *
         * 不和 base / odom 坐标做位置运算。
         */

        if(
            std::isfinite(msg->data)
            &&
            msg->data > 0.10
            &&
            msg->data <= max_valid_depth_
        )
        {
            front_distance_ = msg->data;

            depth_received_ = true;

            depth_ever_acquired_ = true;

            last_depth_time_ = ros::Time::now();
        }
    }


    /*
     * ========================================
     * Helper functions
     * ========================================
     */

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


    bool visionFresh()
    {
        if(!vision_received_)
            return false;


        ros::Time now =
            ros::Time::now();


        if(
            (now - last_valid_time_).toSec()
            >
            vision_timeout_
        )
            return false;


        if(
            (now - last_type_time_).toSec()
            >
            vision_timeout_
        )
            return false;


        if(
            (now - last_center_time_).toSec()
            >
            vision_timeout_
        )
            return false;


        if(
            (now - last_confidence_time_).toSec()
            >
            vision_timeout_
        )
            return false;


        return true;
    }


    bool depthFresh()
    {
        if(!depth_received_)
            return false;


        return
            (ros::Time::now() - last_depth_time_).toSec()
            <=
            depth_timeout_;
    }


    bool validButtonTarget()
    {
        return
            target_valid_
            &&
            target_type_ == "elevator_call_button"
            &&
            target_confidence_ >= min_confidence_;
    }


    void stopRobot()
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;

        cmd.angular.z = 0.0;

        cmd_pub_.publish(cmd);
    }


    void publishMotion(
        double linear_x,
        double angular_z
    )
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = linear_x;
        cmd.linear.y = 0.0;

        cmd.angular.z = angular_z;

        cmd_pub_.publish(cmd);
    }


    /*
     * ========================================
     * Main controller
     * ========================================
     */

    void control()
    {
        /*
         * ====================================
         * 1. Vision stream safety
         * ====================================
         *
         * 注意：
         *
         * "视觉节点没有工作"
         *
         * 和
         *
         * "视觉节点工作正常，但是没有看到按钮"
         *
         * 是两种完全不同的状态。
         *
         * 如果视觉数据流本身 stale，
         * 不允许机器人盲目旋转。
         */

        if(!visionFresh())
        {
            if(!has_seen_valid_target_)
            {
                state_ = WAIT_TARGET;

                stopRobot();

                ROS_WARN_THROTTLE(
                    1.0,
                    "WAIT_TARGET: vision stream missing/stale -> STOP"
                );
            }
            else
            {
                state_ = VISION_LOST;

                stopRobot();

                ROS_WARN_THROTTLE(
                    1.0,
                    "VISION_LOST: vision stream stale after target acquisition -> STOP"
                );
            }

            return;
        }


        /*
         * ====================================
         * 2. Vision alive but button not found
         * ====================================
         */

        if(!validButtonTarget())
        {
            /*
             * 启动以后从未成功识别过按钮：
             *
             * 允许低速原地 SEARCH。
             */
            if(!has_seen_valid_target_)
            {
                state_ = SEARCH;


                publishMotion(
                    0.0,
                    search_angular_speed_
                );


                ROS_INFO_THROTTLE(
                    0.5,
                    "SEARCH | no valid button yet | wz=%.3f",
                    search_angular_speed_
                );
            }

            /*
             * 已经进入过按钮跟踪以后：
             *
             * 视觉突然失效时保持之前的
             * 安全逻辑：
             *
             * STOP。
             *
             * 不允许重新转圈搜索。
             */
            else
            {
                state_ = VISION_LOST;


                stopRobot();


                ROS_WARN_THROTTLE(
                    0.5,
                    "VISION_LOST: target invalid after acquisition -> STOP"
                );
            }


            return;
        }


        /*
         * ====================================
         * 3. 第一次真正获取按钮
         * ====================================
         */

        if(!has_seen_valid_target_)
        {
            has_seen_valid_target_ = true;


            /*
             * 非常重要：
             *
             * SEARCH过程中机器人可能旋转，
             * 深度相机可能看到其他墙面。
             *
             * 这些距离不能作为按钮所在
             * 电梯面板的距离。
             *
             * 因此第一次获得有效按钮时，
             * 清除 SEARCH期间积累的depth状态。
             */

            depth_received_ = false;
            depth_ever_acquired_ = false;


            ROS_INFO(
                "TARGET ACQUIRED: elevator_call_button"
            );

            ROS_INFO(
                "Depth history from SEARCH cleared"
            );
        }


        /*
         * ====================================
         * 4. Pixel error
         * ====================================
         *
         * 这里只处理图像像素坐标。
         *
         * 不与 base / odom / camera
         * 三维坐标直接混合计算。
         */

        const double image_center =
            image_width_ / 2.0;


        const double error_px =
            target_center_x_
            -
            image_center;


        /*
         * ====================================
         * 5. 偏差过大 -> ALIGN
         * ====================================
         *
         * 以下开始保持原有控制逻辑。
         */

        if(
            std::fabs(error_px)
            >
            realign_threshold_px_
        )
        {
            state_ = ALIGN;


            const double angular =
                limit(
                    yaw_sign_
                    *
                    kp_angular_align_
                    *
                    error_px,

                    max_align_angular_
                );


            publishMotion(
                0.0,
                angular
            );


            ROS_INFO_THROTTLE(
                0.5,
                "ALIGN | error=%.1f px wz=%.3f",
                error_px,
                angular
            );


            return;
        }


        /*
         * ====================================
         * 6. Depth从未出现
         *
         * -> FAR_APPROACH
         * ====================================
         */

        if(!depth_ever_acquired_)
        {
            state_ = FAR_APPROACH;


            const double angular =
                limit(
                    yaw_sign_
                    *
                    kp_angular_move_
                    *
                    error_px,

                    max_move_angular_
                );


            publishMotion(
                far_speed_,
                angular
            );


            ROS_INFO_THROTTLE(
                0.5,
                "FAR_APPROACH | no depth yet | error=%.1f px vx=%.2f wz=%.3f",
                error_px,
                far_speed_,
                angular
            );


            return;
        }


        /*
         * ====================================
         * 7. Depth曾经获得，但现在丢失
         *
         * -> DEPTH_LOST + STOP
         * ====================================
         */

        if(!depthFresh())
        {
            state_ = DEPTH_LOST;


            stopRobot();


            ROS_WARN_THROTTLE(
                0.5,
                "DEPTH_LOST -> STOP"
            );


            return;
        }


        /*
         * ====================================
         * 8. 到达停止距离
         *
         * -> READY
         * ====================================
         */

        if(
            front_distance_
            <=
            stop_distance_
        )
        {
            state_ = READY;


            stopRobot();


            ROS_INFO_THROTTLE(
                0.5,
                "READY | distance=%.3f m",
                front_distance_
            );


            return;
        }


        /*
         * ====================================
         * 9. DEPTH_APPROACH
         * ====================================
         */

        state_ = DEPTH_APPROACH;


        double linear_speed;


        if(front_distance_ > 1.0)
        {
            linear_speed =
                medium_speed_;
        }
        else
        {
            linear_speed =
                near_speed_;
        }


        const double angular =
            limit(
                yaw_sign_
                *
                kp_angular_move_
                *
                error_px,

                max_move_angular_
            );


        publishMotion(
            linear_speed,
            angular
        );


        ROS_INFO_THROTTLE(
            0.5,
            "DEPTH_APPROACH | distance=%.3f m error=%.1f px vx=%.2f wz=%.3f",
            front_distance_,
            error_px,
            linear_speed,
            angular
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
        "button_approach_controller"
    );


    ButtonApproachController controller;


    controller.spin();


    return 0;
}