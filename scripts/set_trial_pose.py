#!/usr/bin/env python3

import math
import time

import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState

from geometry_msgs.msg import Twist

from std_srvs.srv import Empty


class TrialPoseSetter:
    def __init__(self):

        # ============================================================
        # Parameters
        # ============================================================

        self.distance = float(
            rospy.get_param(
                "~distance",
                2.5
            )
        )

        self.target_angle_deg = float(
            rospy.get_param(
                "~angle_deg",
                0.0
            )
        )

        # 这里使用你已经确认过的真实 Gazebo model name。
        # 如果你的实际名称不是 go1_gazebo，
        # 保留你刚刚已经修改正确的名称即可。
        self.robot_model_name = rospy.get_param(
            "~robot_model_name",
            "go1_gazebo"
        )

        self.elevator_model_name = rospy.get_param(
            "~elevator_model_name",
            "sim_elevator"
        )

        # ------------------------------------------------------------
        # Translation
        # ------------------------------------------------------------

        self.translation_settle_time = float(
            rospy.get_param(
                "~translation_settle_time",
                2.0
            )
        )

        # ------------------------------------------------------------
        # Physical turning
        #
        # 之前实测：
        # 0.15 rad/s 可能只有身体扭转而不能正常迈步；
        # 0.25 rad/s 能正常触发 Go1 转向步态。
        # ------------------------------------------------------------

        self.rotation_speed = abs(
            float(
                rospy.get_param(
                    "~rotation_speed",
                    0.25
                )
            )
        )

        self.angle_tolerance_deg = abs(
            float(
                rospy.get_param(
                    "~angle_tolerance_deg",
                    3.0
                )
            )
        )

        self.rotation_timeout_s = float(
            rospy.get_param(
                "~rotation_timeout_s",
                20.0
            )
        )

        self.stable_samples_required = int(
            rospy.get_param(
                "~stable_samples",
                5
            )
        )

        self.post_rotation_settle_time = float(
            rospy.get_param(
                "~post_rotation_settle_time",
                1.0
            )
        )


        # ============================================================
        # Parameter checks
        # ============================================================

        if self.distance <= 0.0:
            raise ValueError(
                "_distance must be > 0"
            )

        if self.rotation_speed <= 0.0:
            raise ValueError(
                "_rotation_speed must be > 0"
            )

        if self.angle_tolerance_deg <= 0.0:
            raise ValueError(
                "_angle_tolerance_deg must be > 0"
            )

        if self.rotation_timeout_s <= 0.0:
            raise ValueError(
                "_rotation_timeout_s must be > 0"
            )

        if self.stable_samples_required < 1:
            raise ValueError(
                "_stable_samples must be >= 1"
            )


        # ============================================================
        # /cmd_vel publisher
        #
        # IMPORTANT:
        # junior_ctrl must already be in MOVE_BASE mode:
        #
        #   2 -> 5
        # ============================================================

        self.cmd_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=10
        )


        # ============================================================
        # Gazebo services
        # ============================================================

        rospy.loginfo(
            "Waiting for Gazebo services..."
        )

        rospy.wait_for_service(
            "/gazebo/get_model_state"
        )

        rospy.wait_for_service(
            "/gazebo/set_model_state"
        )

        rospy.wait_for_service(
            "/gazebo/pause_physics"
        )

        rospy.wait_for_service(
            "/gazebo/unpause_physics"
        )


        self.get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state",
            GetModelState
        )

        self.set_model_state = rospy.ServiceProxy(
            "/gazebo/set_model_state",
            SetModelState
        )

        self.pause_physics = rospy.ServiceProxy(
            "/gazebo/pause_physics",
            Empty
        )

        self.unpause_physics = rospy.ServiceProxy(
            "/gazebo/unpause_physics",
            Empty
        )


        # Give ROS a short moment to connect /cmd_vel publisher.
        rospy.sleep(
            0.5
        )


    # ================================================================
    # Angle helpers
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = (
            2.0
            *
            (
                q.w * q.z
                +
                q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0
            *
            (
                q.y * q.y
                +
                q.z * q.z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )


    @staticmethod
    def wrap_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    # ================================================================
    # Gazebo model helpers
    # ================================================================

    def get_robot_state(self):

        result = self.get_model_state(
            self.robot_model_name,
            "world"
        )

        if not result.success:
            raise RuntimeError(
                "Failed to get robot model state: {}".format(
                    result.status_message
                )
            )

        return result


    def get_elevator_state(self):

        result = self.get_model_state(
            self.elevator_model_name,
            "world"
        )

        if not result.success:
            raise RuntimeError(
                "Failed to get elevator model state: {}".format(
                    result.status_message
                )
            )

        return result


    # ================================================================
    # Heading error
    #
    # IMPORTANT FRAME RULE:
    #
    # robot position:
    #     Gazebo WORLD frame
    #
    # elevator position:
    #     Gazebo WORLD frame
    #
    # robot yaw:
    #     Gazebo WORLD frame
    #
    # Therefore:
    #
    # heading_error =
    #     bearing_world(robot -> elevator)
    #     -
    #     robot_yaw_world
    #
    # No base / odom / camera frame is mixed here.
    # ================================================================

    def calculate_heading_error(
        self,
        robot_state,
        elevator_state
    ):

        robot_x = (
            robot_state.pose.position.x
        )

        robot_y = (
            robot_state.pose.position.y
        )

        elevator_x = (
            elevator_state.pose.position.x
        )

        elevator_y = (
            elevator_state.pose.position.y
        )


        dx = (
            elevator_x
            -
            robot_x
        )

        dy = (
            elevator_y
            -
            robot_y
        )


        bearing_world = math.atan2(
            dy,
            dx
        )


        robot_yaw_world = (
            self.quaternion_to_yaw(
                robot_state.pose.orientation
            )
        )


        heading_error = self.wrap_angle(
            bearing_world
            -
            robot_yaw_world
        )


        return (
            heading_error,
            bearing_world,
            robot_yaw_world
        )


    def calculate_model_distance(
        self,
        robot_state,
        elevator_state
    ):

        dx = (
            elevator_state.pose.position.x
            -
            robot_state.pose.position.x
        )

        dy = (
            elevator_state.pose.position.y
            -
            robot_state.pose.position.y
        )

        return math.hypot(
            dx,
            dy
        )


    # ================================================================
    # cmd_vel helpers
    # ================================================================

    def publish_cmd(
        self,
        angular_z
    ):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0

        cmd.angular.z = (
            angular_z
        )

        self.cmd_pub.publish(
            cmd
        )


    def stop_robot(
        self,
        repeat=10
    ):

        for _ in range(repeat):

            self.publish_cmd(
                0.0
            )

            rospy.sleep(
                0.05
            )


    # ================================================================
    # Step 1:
    # Translate robot to requested distance
    #
    # CRITICAL:
    #
    # We change ONLY:
    #
    #     x
    #     y
    #
    # We preserve:
    #
    #     z
    #     orientation
    #
    # Therefore yaw is NOT teleported.
    # ================================================================

    def translate_robot(self):

        robot_state = (
            self.get_robot_state()
        )

        elevator_state = (
            self.get_elevator_state()
        )


        elevator_x = (
            elevator_state.pose.position.x
        )

        elevator_y = (
            elevator_state.pose.position.y
        )

        elevator_yaw = (
            self.quaternion_to_yaw(
                elevator_state.pose.orientation
            )
        )


        # ------------------------------------------------------------
        # Place Go1 on the negative local-X side of the elevator.
        #
        # If elevator yaw = 0:
        #
        #     Go1 -------> Elevator
        #
        #       +X direction
        # ------------------------------------------------------------

        target_robot_x = (
            elevator_x
            -
            self.distance
            *
            math.cos(
                elevator_yaw
            )
        )

        target_robot_y = (
            elevator_y
            -
            self.distance
            *
            math.sin(
                elevator_yaw
            )
        )


        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "STEP 1: TRANSLATION"
        )

        rospy.loginfo(
            "Requested model distance = %.3f m",
            self.distance
        )

        rospy.loginfo(
            "Elevator WORLD pose: x=%.3f y=%.3f yaw=%.2f deg",
            elevator_x,
            elevator_y,
            math.degrees(
                elevator_yaw
            )
        )

        rospy.loginfo(
            "Target Go1 WORLD position: x=%.3f y=%.3f",
            target_robot_x,
            target_robot_y
        )

        rospy.loginfo(
            "Robot orientation will NOT be modified."
        )

        rospy.loginfo(
            "========================================"
        )


        # ------------------------------------------------------------
        # First make sure /cmd_vel is zero.
        # ------------------------------------------------------------

        self.stop_robot(
            repeat=5
        )


        # ------------------------------------------------------------
        # Pause physics while repositioning.
        # ------------------------------------------------------------

        rospy.loginfo(
            "Pausing Gazebo physics..."
        )

        self.pause_physics()


        try:

            state = ModelState()

            state.model_name = (
                self.robot_model_name
            )

            state.reference_frame = (
                "world"
            )


            # --------------------------------------------------------
            # Change ONLY X/Y.
            # --------------------------------------------------------

            state.pose.position.x = (
                target_robot_x
            )

            state.pose.position.y = (
                target_robot_y
            )


            # Keep current Z.
            state.pose.position.z = (
                robot_state.pose.position.z
            )


            # --------------------------------------------------------
            # Preserve the complete current orientation.
            #
            # NO yaw teleport.
            # --------------------------------------------------------

            state.pose.orientation.x = (
                robot_state.pose.orientation.x
            )

            state.pose.orientation.y = (
                robot_state.pose.orientation.y
            )

            state.pose.orientation.z = (
                robot_state.pose.orientation.z
            )

            state.pose.orientation.w = (
                robot_state.pose.orientation.w
            )


            # --------------------------------------------------------
            # Zero model velocity.
            # --------------------------------------------------------

            state.twist.linear.x = 0.0
            state.twist.linear.y = 0.0
            state.twist.linear.z = 0.0

            state.twist.angular.x = 0.0
            state.twist.angular.y = 0.0
            state.twist.angular.z = 0.0


            result = self.set_model_state(
                state
            )


            if not result.success:
                raise RuntimeError(
                    "Gazebo set_model_state failed: {}".format(
                        result.status_message
                    )
                )


        finally:

            rospy.loginfo(
                "Unpausing Gazebo physics..."
            )

            self.unpause_physics()


        # ------------------------------------------------------------
        # Let quadruped settle physically.
        # ------------------------------------------------------------

        rospy.loginfo(
            "Waiting %.1f s for translation settling...",
            self.translation_settle_time
        )

        self.stop_robot(
            repeat=5
        )

        rospy.sleep(
            self.translation_settle_time
        )


        # ------------------------------------------------------------
        # Verify translation.
        # ------------------------------------------------------------

        robot_after = (
            self.get_robot_state()
        )

        elevator_after = (
            self.get_elevator_state()
        )

        measured_distance = (
            self.calculate_model_distance(
                robot_after,
                elevator_after
            )
        )

        heading_error, _, _ = (
            self.calculate_heading_error(
                robot_after,
                elevator_after
            )
        )


        rospy.loginfo(
            "Translation complete"
        )

        rospy.loginfo(
            "Measured distance = %.3f m",
            measured_distance
        )

        rospy.loginfo(
            "Current heading error = %.2f deg",
            math.degrees(
                heading_error
            )
        )


    # ================================================================
    # Step 2:
    # Physically rotate Go1 using /cmd_vel
    #
    # Definition:
    #
    # current_heading_error =
    #     bearing_to_elevator
    #     -
    #     robot_yaw
    #
    # target_heading_error =
    #     requested _angle_deg
    #
    #
    # Let:
    #
    # delta =
    #     current_heading_error
    #     -
    #     target_heading_error
    #
    #
    # If delta > 0:
    #
    # robot yaw needs to INCREASE
    # => angular.z > 0
    #
    #
    # If delta < 0:
    #
    # robot yaw needs to DECREASE
    # => angular.z < 0
    #
    # ================================================================

    def rotate_robot_physically(self):

        target_error = math.radians(
            self.target_angle_deg
        )

        tolerance = math.radians(
            self.angle_tolerance_deg
        )


        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "STEP 2: PHYSICAL ROTATION"
        )

        rospy.loginfo(
            "Requested heading error = %.2f deg",
            self.target_angle_deg
        )

        rospy.loginfo(
            "Rotation speed = %.3f rad/s",
            self.rotation_speed
        )

        rospy.loginfo(
            "Tolerance = +/- %.2f deg",
            self.angle_tolerance_deg
        )

        rospy.loginfo(
            "Timeout = %.1f s",
            self.rotation_timeout_s
        )

        rospy.loginfo(
            "Yaw will be changed ONLY through /cmd_vel."
        )

        rospy.loginfo(
            "========================================"
        )


        start_wall_time = (
            time.monotonic()
        )

        stable_samples = 0

        loop_rate = rospy.Rate(
            20
        )


        while not rospy.is_shutdown():

            elapsed = (
                time.monotonic()
                -
                start_wall_time
            )


            # --------------------------------------------------------
            # Safety timeout
            # --------------------------------------------------------

            if elapsed > self.rotation_timeout_s:

                self.stop_robot()

                raise RuntimeError(
                    "POSE_SETUP_TIMEOUT: "
                    "could not reach requested heading "
                    "within {:.1f} s".format(
                        self.rotation_timeout_s
                    )
                )


            robot_state = (
                self.get_robot_state()
            )

            elevator_state = (
                self.get_elevator_state()
            )


            current_error, _, robot_yaw = (
                self.calculate_heading_error(
                    robot_state,
                    elevator_state
                )
            )


            # --------------------------------------------------------
            # Error between current heading error and desired
            # heading error.
            #
            # Both are angular quantities defined in the same way.
            # --------------------------------------------------------

            control_error = (
                self.wrap_angle(
                    current_error
                    -
                    target_error
                )
            )


            # --------------------------------------------------------
            # Within tolerance
            # --------------------------------------------------------

            if abs(control_error) <= tolerance:

                self.publish_cmd(
                    0.0
                )

                stable_samples += 1


                rospy.loginfo_throttle(
                    0.5,
                    "ANGLE OK | current=%.2f deg target=%.2f deg "
                    "error=%.2f deg stable=%d/%d",
                    math.degrees(
                        current_error
                    ),
                    self.target_angle_deg,
                    math.degrees(
                        control_error
                    ),
                    stable_samples,
                    self.stable_samples_required
                )


                if (
                    stable_samples
                    >=
                    self.stable_samples_required
                ):
                    break


            # --------------------------------------------------------
            # Need positive yaw rotation
            # --------------------------------------------------------

            elif control_error > 0.0:

                stable_samples = 0

                self.publish_cmd(
                    self.rotation_speed
                )


                rospy.loginfo_throttle(
                    0.5,
                    "TURN + | current=%.2f deg target=%.2f deg "
                    "delta=%.2f deg | wz=+%.3f",
                    math.degrees(
                        current_error
                    ),
                    self.target_angle_deg,
                    math.degrees(
                        control_error
                    ),
                    self.rotation_speed
                )


            # --------------------------------------------------------
            # Need negative yaw rotation
            # --------------------------------------------------------

            else:

                stable_samples = 0

                self.publish_cmd(
                    -self.rotation_speed
                )


                rospy.loginfo_throttle(
                    0.5,
                    "TURN - | current=%.2f deg target=%.2f deg "
                    "delta=%.2f deg | wz=-%.3f",
                    math.degrees(
                        current_error
                    ),
                    self.target_angle_deg,
                    math.degrees(
                        control_error
                    ),
                    self.rotation_speed
                )


            loop_rate.sleep()


        # ============================================================
        # Stop physically
        # ============================================================

        self.stop_robot()


        rospy.loginfo(
            "Rotation command finished."
        )

        rospy.loginfo(
            "Waiting %.1f s for rotation settling...",
            self.post_rotation_settle_time
        )

        rospy.sleep(
            self.post_rotation_settle_time
        )

        self.stop_robot(
            repeat=5
        )


    # ================================================================
    # Final verification
    # ================================================================

    def verify_final_condition(self):

        robot_state = (
            self.get_robot_state()
        )

        elevator_state = (
            self.get_elevator_state()
        )


        final_distance = (
            self.calculate_model_distance(
                robot_state,
                elevator_state
            )
        )


        final_heading_error, _, final_robot_yaw = (
            self.calculate_heading_error(
                robot_state,
                elevator_state
            )
        )


        final_error_from_request = (
            self.wrap_angle(
                final_heading_error
                -
                math.radians(
                    self.target_angle_deg
                )
            )
        )


        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "TRIAL POSE READY"
        )

        rospy.loginfo(
            "Requested distance       = %.3f m",
            self.distance
        )

        rospy.loginfo(
            "Measured model distance  = %.3f m",
            final_distance
        )

        rospy.loginfo(
            "Requested heading error  = %.2f deg",
            self.target_angle_deg
        )

        rospy.loginfo(
            "Measured heading error   = %.2f deg",
            math.degrees(
                final_heading_error
            )
        )

        rospy.loginfo(
            "Heading setup error      = %.2f deg",
            math.degrees(
                final_error_from_request
            )
        )

        rospy.loginfo(
            "Final Go1 WORLD yaw      = %.2f deg",
            math.degrees(
                final_robot_yaw
            )
        )

        rospy.loginfo(
            "========================================"
        )


        # ------------------------------------------------------------
        # This is only a warning.
        #
        # The trial_logger will record the ACTUAL initial condition,
        # which is what should be used for later analysis.
        # ------------------------------------------------------------

        if (
            abs(
                math.degrees(
                    final_error_from_request
                )
            )
            >
            self.angle_tolerance_deg
            *
            2.0
        ):

            rospy.logwarn(
                "Final heading drift is larger than expected."
            )


    # ================================================================
    # Main procedure
    # ================================================================

    def run(self):

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "Go1 reproducible trial pose setup"
        )

        rospy.loginfo(
            "Robot model: %s",
            self.robot_model_name
        )

        rospy.loginfo(
            "Elevator model: %s",
            self.elevator_model_name
        )

        rospy.loginfo(
            "IMPORTANT: junior_ctrl should already be in MOVE_BASE."
        )

        rospy.loginfo(
            "========================================"
        )


        # Step 1:
        # translate without changing orientation
        self.translate_robot()


        # Step 2:
        # physically rotate through Go1 gait
        self.rotate_robot_physically()


        # Step 3:
        # final measurement
        self.verify_final_condition()


def main():

    rospy.init_node(
        "set_trial_pose"
    )


    setter = None


    try:

        setter = TrialPoseSetter()

        setter.run()


    except rospy.ROSInterruptException:

        if setter is not None:
            setter.stop_robot()


    except Exception as e:

        if setter is not None:

            try:
                setter.stop_robot()

            except Exception:
                pass


        rospy.logfatal(
            "Failed to set trial pose: %s",
            str(e)
        )

        raise


    finally:

        if setter is not None:

            try:
                setter.stop_robot(
                    repeat=5
                )

            except Exception:
                pass


if __name__ == "__main__":
    main()