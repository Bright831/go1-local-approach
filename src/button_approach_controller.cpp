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
         * Tracks whether at least one valid button target has been detected
         * since the controller started.
         *
         * false:
         *   SEARCH is allowed.
         *
         * true:
         *   Subsequent vision loss causes STOP only; SEARCH is not restarted automatically.
         */
        has_seen_valid_target_ = false;


        /*
         * Depth state
         */
        depth_received_ = false;
        depth_ever_acquired_ = false;

        /*
         * Tracks whether the initial visual alignment has been completed
         * and forward approach has formally started.
         *
         * false:
         *   WAIT_TARGET / SEARCH / initial ALIGN.
         *   Depth measurements must not establish formal tracking at this stage.
         *
         * true:
         *   FAR_APPROACH has already been entered.
         *   After this point, even if vision temporarily enters VISION_LOST,
         *   valid depth measurements are still allowed to update.
         */
        approach_started_ = false;


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
         * If the horizontal pixel error exceeds this threshold,
         * forward motion stops and the robot realigns in place.
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
         * The runtime value is provided by the YAML configuration.
         *
         * The following setting has been validated in simulation:
         *
         * yaw_sign = +1.0
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
         * Publishers
         * ========================================
         */

        cmd_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                "/cmd_vel",
                10
            );

        state_pub_ =
            nh_.advertise<std_msgs::String>(
                "/button_approach/state",
                1,
                true
            );


        publishState();


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

        ROS_INFO(
            "State topic: /button_approach/state"
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


    /*
     * ========================================
     * Subscribers
     * ========================================
     */

    ros::Subscriber valid_sub_;
    ros::Subscriber type_sub_;
    ros::Subscriber center_sub_;
    ros::Subscriber confidence_sub_;
    ros::Subscriber depth_sub_;


    /*
     * ========================================
     * Publishers
     * ========================================
     */

    ros::Publisher cmd_pub_;
    ros::Publisher state_pub_;


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

    /*
     * Historical phase flag.
     *
     * Once FAR_APPROACH has genuinely been entered,
     * this flag is not cleared by temporary state transitions.
     */
    bool approach_started_;

    ros::Time last_depth_time_;


    /*
     * ========================================
     * Controller state
     * ========================================
     */

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
     * Vision callbacks
     * ========================================
     */

    void validCallback(
        const std_msgs::Bool::ConstPtr& msg
    )
    {
        target_valid_ = msg->data;

        vision_received_ = true;

        last_valid_time_ =
            ros::Time::now();
    }


    void typeCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        target_type_ = msg->data;

        last_type_time_ =
            ros::Time::now();
    }


    void centerCallback(
        const std_msgs::Float32::ConstPtr& msg
    )
    {
        target_center_x_ = msg->data;

        last_center_time_ =
            ros::Time::now();
    }


    void confidenceCallback(
        const std_msgs::Float32::ConstPtr& msg
    )
    {
        target_confidence_ = msg->data;

        last_confidence_time_ =
            ros::Time::now();
    }


    /*
     * ========================================
     * Depth callback
     * ========================================
     */

    void depthCallback(
        const std_msgs::Float32::ConstPtr& msg
    )
    {
        /*
         * /front_distance is a scalar range measurement produced by
         * the depth node.
         *
         * It is not a 3D position expressed in the base or odom frame,
         * so it must not be directly combined with positional coordinates from those frames.
         *
         *
         * Initially:
         *
         * WAIT_TARGET
         * SEARCH
         * ALIGN
         *
         * At this stage, the robot has not completed visual alignment.
         * The depth camera may observe a wall or another unrelated surface,
         * so these measurements must not establish formal depth tracking.
         *
         *
         * Once:
         *
         * approach_started_ == true
         *
         * the controller has completed the initial alignment
         * and has entered FAR_APPROACH.
         *
         * After this point, even if the current state is temporarily VISION_LOST,
         * valid depth measurements continue to update.
         */


        /*
         * Return only when the depth measurement is invalid.
         */
        if(
            !std::isfinite(msg->data)
            ||
            msg->data <= 0.10
            ||
            msg->data > max_valid_depth_
        )
        {
            return;
        }


        /*
         * Store the latest valid depth value.
         */
        front_distance_ =
            msg->data;


        /*
         * Depth participates in controller tracking only after
         * the approach has formally started.
         *
         * Important:
         *
         * This logic no longer depends on the current state_.
         *
         * Therefore, even while the controller is in VISION_LOST,
         * as long as the approach has already started,
         * depth measurements may continue to update.
         */
        if(approach_started_)
        {
            depth_received_ = true;
            depth_ever_acquired_ = true;

            last_depth_time_ =
                ros::Time::now();
        }
    }


    /*
     * ========================================
     * State helper functions
     * ========================================
     */

    std::string stateToString(
        ButtonControllerState state
    ) const
    {
        switch(state)
        {
            case WAIT_TARGET:
                return "WAIT_TARGET";

            case SEARCH:
                return "SEARCH";

            case ALIGN:
                return "ALIGN";

            case FAR_APPROACH:
                return "FAR_APPROACH";

            case DEPTH_APPROACH:
                return "DEPTH_APPROACH";

            case READY:
                return "READY";

            case VISION_LOST:
                return "VISION_LOST";

            case DEPTH_LOST:
                return "DEPTH_LOST";

            default:
                return "UNKNOWN";
        }
    }


    void publishState()
    {
        std_msgs::String msg;

        msg.data =
            stateToString(
                state_
            );

        state_pub_.publish(
            msg
        );
    }


    void setState(
        ButtonControllerState new_state
    )
    {
        if(state_ == new_state)
        {
            return;
        }


        const std::string old_state =
            stateToString(
                state_
            );

        const std::string next_state =
            stateToString(
                new_state
            );


        state_ =
            new_state;


        publishState();


        ROS_INFO(
            "STATE TRANSITION: %s -> %s",
            old_state.c_str(),
            next_state.c_str()
        );
    }


    /*
     * ========================================
     * General helper functions
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
        {
            return false;
        }


        const ros::Time now =
            ros::Time::now();


        if(
            (now - last_valid_time_).toSec()
            >
            vision_timeout_
        )
        {
            return false;
        }


        if(
            (now - last_type_time_).toSec()
            >
            vision_timeout_
        )
        {
            return false;
        }


        if(
            (now - last_center_time_).toSec()
            >
            vision_timeout_
        )
        {
            return false;
        }


        if(
            (now - last_confidence_time_).toSec()
            >
            vision_timeout_
        )
        {
            return false;
        }


        return true;
    }


    bool depthFresh()
    {
        if(!depth_received_)
        {
            return false;
        }


        return
            (
                ros::Time::now()
                -
                last_depth_time_
            ).toSec()
            <=
            depth_timeout_;
    }


    bool validButtonTarget()
    {
        return
            target_valid_
            &&
            target_type_
                ==
                "elevator_call_button"
            &&
            target_confidence_
                >=
                min_confidence_;
    }


    void stopRobot()
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;

        cmd.angular.z = 0.0;

        cmd_pub_.publish(
            cmd
        );
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

        cmd_pub_.publish(
            cmd
        );
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
 * 0. READY is a terminal state
 * ====================================
 *
 * Once stop_distance is reached and the controller enters READY,
 * the current trial is considered complete.
 *
 * Subsequent depth measurements may fluctuate because of noise, for example
 * from 0.500 m to 0.502 m or 0.498 m.
 *
 * After entering READY, the controller must not return to DEPTH_APPROACH,
 * otherwise the robot could resume forward motion.
 *
 * READY is cleared only when the controller is restarted.
 */
        if(state_ == READY)
        {
            stopRobot();

            ROS_INFO_THROTTLE(
                1.0,
                "READY latched -> STOP | distance=%.3f m",
                front_distance_
            );

            return;
        }
        /*
         * ====================================
         * 1. Vision stream safety
         * ====================================
         */

        if(!visionFresh())
        {
            if(!has_seen_valid_target_)
            {
                setState(
                    WAIT_TARGET
                );

                stopRobot();


                ROS_WARN_THROTTLE(
                    1.0,
                    "WAIT_TARGET: vision stream missing/stale -> STOP"
                );
            }
            else
            {
                setState(
                    VISION_LOST
                );

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
         * 2. Vision alive but no valid button
         * ====================================
         */

        if(!validButtonTarget())
        {
            /*
             * No valid target has been detected since startup:
             * SEARCH is allowed.。
             */
            if(!has_seen_valid_target_)
            {
                setState(
                    SEARCH
                );


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
             * The target was previously acquired but is now lost:
             * STOP.
             *
             * Do not restart SEARCH.
             */
            else
            {
                setState(
                    VISION_LOST
                );


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
         * 3. First target acquisition
         * ====================================
         */

        if(!has_seen_valid_target_)
        {
            has_seen_valid_target_ =
                true;


            /*
             * During SEARCH, depth measurements may correspond to unrelated surfaces.
             *
             * When the button is acquired for the first time,
             * clear all previous depth-tracking history.
             */
            depth_received_ =
                false;

            depth_ever_acquired_ =
                false;

            approach_started_ =
                false;


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
         * Only image pixel coordinates are handled here.
         *
         * They are not directly combined with 3D positions from base, odom, or camera frames.
         */

        const double image_center =
            image_width_
            /
            2.0;


        const double error_px =
            target_center_x_
            -
            image_center;


        /*
         * ====================================
         * 5. Large pixel error -> ALIGN
         * ====================================
         */

        if(
            std::fabs(
                error_px
            )
            >
            realign_threshold_px_
        )
        {
            setState(
                ALIGN
            );


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
         * 6. No usable depth yet
         *
         * -> FAR_APPROACH
         * ====================================
         */

        if(!depth_ever_acquired_)
        {
            /*
             * Reaching this point means:
             *
             * vision fresh
             * target valid
             * error <= realign_threshold_px
             *
             * Therefore, the initial visual alignment requirement has been satisfied.
             *
             * From this point onward, depth is allowed to participate in formal tracking.
             */
            if(!approach_started_)
            {
                approach_started_ =
                    true;

                ROS_INFO(
                    "APPROACH STARTED: depth tracking enabled"
                );
            }


            setState(
                FAR_APPROACH
            );


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
         * 7. Depth acquired before,
         *    but now stale
         *
         * -> DEPTH_LOST + STOP
         * ====================================
         */

        if(!depthFresh())
        {
            setState(
                DEPTH_LOST
            );


            stopRobot();


            ROS_WARN_THROTTLE(
                0.5,
                "DEPTH_LOST -> STOP"
            );


            return;
        }


        /*
         * ====================================
         * 8. Stop distance reached
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
            setState(
                READY
            );


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

        setState(
            DEPTH_APPROACH
        );


        double linear_speed;


        if(
            front_distance_
            >
            1.0
        )
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