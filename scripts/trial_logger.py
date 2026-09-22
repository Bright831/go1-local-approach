#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from pathlib import Path

import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


class TrialLogger:
    def __init__(self):

        # ============================================================
        # Parameters
        # ============================================================

        self.trial_id = int(
            rospy.get_param(
                "~trial_id",
                1
            )
        )

        self.output_dir = Path(
            os.path.expanduser(
                rospy.get_param(
                    "~output_dir",
                    "~/go1_experiment_logs"
                )
            )
        )

        self.robot_model_name = rospy.get_param(
            "~robot_model_name",
            "go1_gazebo"
        )

        self.elevator_model_name = rospy.get_param(
            "~elevator_model_name",
            "sim_elevator"
        )

        self.min_confidence = float(
            rospy.get_param(
                "~min_confidence",
                0.60
            )
        )

        self.timeout_s = float(
            rospy.get_param(
                "~timeout_s",
                120.0
            )
        )

        self.log_rate_hz = float(
            rospy.get_param(
                "~log_rate_hz",
                10.0
            )
        )

        self.overwrite = bool(
            rospy.get_param(
                "~overwrite",
                False
            )
        )


        # ============================================================
        # Output files
        # ============================================================

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        self.trial_name = (
            "trial_{:03d}".format(
                self.trial_id
            )
        )

        self.trial_csv_path = (
            self.output_dir
            / (
                self.trial_name
                + ".csv"
            )
        )

        self.summary_csv_path = (
            self.output_dir
            / "summary.csv"
        )


        # ============================================================
        # Protect old experiments
        # ============================================================

        if (
            self.trial_csv_path.exists()
            and not self.overwrite
        ):
            raise RuntimeError(
                "Trial file already exists: {}. "
                "Use another _trial_id or set "
                "_overwrite:=true".format(
                    self.trial_csv_path
                )
            )


        # ============================================================
        # Current controller / perception data
        # ============================================================

        self.state = "UNKNOWN"

        self.target_valid = False
        self.target_type = "none"
        self.target_center_x = 0.0
        self.target_confidence = 0.0

        self.front_distance = None

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0


        # ============================================================
        # Gazebo world-frame data
        # ============================================================

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.elevator_x = None
        self.elevator_y = None

        self.current_model_distance = None
        self.current_heading_error = None


        # ============================================================
        # Initial-condition data
        #
        # IMPORTANT:
        #
        # Robot pose and elevator pose both come from
        # /gazebo/model_states.
        #
        # Therefore they are both expressed in Gazebo WORLD frame
        # before any subtraction / relative-angle calculation.
        #
        # No base / odom / camera frame is mixed here.
        # ============================================================

        self.initial_pose_recorded = False

        self.initial_robot_x = None
        self.initial_robot_y = None
        self.initial_robot_yaw = None

        self.initial_elevator_x = None
        self.initial_elevator_y = None

        self.initial_model_distance = None
        self.initial_heading_error = None


        # ============================================================
        # Timing
        # ============================================================

        self.trial_start_time = None

        self.target_acquired_time = None
        self.first_depth_time = None
        self.ready_time = None

        self.state_first_times = {}


        # ============================================================
        # Failure / recovery information
        # ============================================================

        self.vision_lost_count = 0
        self.depth_lost_count = 0

        self.success = False
        self.completed = False
        self.finalized = False

        self.failure_reason = ""

        self.final_distance = None


        # ============================================================
        # CSV time-series file
        # ============================================================

        self.csv_fields = [
            "time_s",

            "state",

            "target_valid",
            "target_type",
            "target_center_x_px",
            "target_confidence",

            "front_distance_m",

            "cmd_linear_x_mps",
            "cmd_angular_z_radps",

            "robot_x_world_m",
            "robot_y_world_m",
            "robot_yaw_world_rad",

            "elevator_x_world_m",
            "elevator_y_world_m",

            "model_distance_world_m",
            "heading_error_rad"
        ]

        self.trial_file = open(
            self.trial_csv_path,
            "w",
            newline="",
            buffering=1
        )

        self.trial_writer = csv.DictWriter(
            self.trial_file,
            fieldnames=self.csv_fields
        )

        self.trial_writer.writeheader()


        # ============================================================
        # Subscribers
        # ============================================================

        self.state_sub = rospy.Subscriber(
            "/button_approach/state",
            String,
            self.state_callback,
            queue_size=10
        )

        self.valid_sub = rospy.Subscriber(
            "/vision/target_valid",
            Bool,
            self.valid_callback,
            queue_size=10
        )

        self.type_sub = rospy.Subscriber(
            "/vision/target_type",
            String,
            self.type_callback,
            queue_size=10
        )

        self.center_sub = rospy.Subscriber(
            "/vision/target_center_x",
            Float32,
            self.center_callback,
            queue_size=10
        )

        self.confidence_sub = rospy.Subscriber(
            "/vision/target_confidence",
            Float32,
            self.confidence_callback,
            queue_size=10
        )

        self.depth_sub = rospy.Subscriber(
            "/front_distance",
            Float32,
            self.depth_callback,
            queue_size=10
        )

        self.cmd_sub = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.cmd_callback,
            queue_size=10
        )

        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_callback,
            queue_size=10
        )


        # ============================================================
        # Periodic CSV logging
        # ============================================================

        self.log_timer = rospy.Timer(
            rospy.Duration(
                1.0 / self.log_rate_hz
            ),
            self.log_timer_callback
        )


        rospy.on_shutdown(
            self.on_shutdown
        )


        rospy.loginfo(
            "Trial logger started"
        )

        rospy.loginfo(
            "Trial ID: %03d",
            self.trial_id
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
            "Trial CSV: %s",
            str(self.trial_csv_path)
        )

        rospy.loginfo(
            "Summary CSV: %s",
            str(self.summary_csv_path)
        )

        rospy.loginfo(
            "Timeout: %.1f s",
            self.timeout_s
        )


    # ================================================================
    # Utility
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(q):
        """
        Quaternion -> yaw.

        Input pose comes from gazebo_msgs/ModelStates,
        therefore this yaw is expressed relative to Gazebo WORLD frame.
        """

        siny_cosp = (
            2.0
            * (
                q.w * q.z
                +
                q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0
            * (
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
        """
        Wrap angle to [-pi, pi].
        """

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    @staticmethod
    def value_or_blank(value):
        if value is None:
            return ""

        if isinstance(value, float):
            if not math.isfinite(value):
                return ""

        return value


    def relative_time(self, timestamp=None):

        if self.trial_start_time is None:
            return None

        if timestamp is None:
            timestamp = rospy.Time.now()

        return (
            timestamp
            -
            self.trial_start_time
        ).to_sec()


    # ================================================================
    # Start trial
    # ================================================================

    def ensure_trial_started(
        self,
        timestamp
    ):
        """
        Trial starts when the first controller-state message arrives.

        Because /button_approach/state is latched,
        the logger can immediately discover the current controller state.
        """

        if self.trial_start_time is not None:
            return

        self.trial_start_time = timestamp

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "TRIAL %03d STARTED",
            self.trial_id
        )

        rospy.loginfo(
            "========================================"
        )

        self.capture_initial_pose_if_ready()


    # ================================================================
    # Controller state
    # ================================================================

    def state_callback(
        self,
        msg
    ):
        now = rospy.Time.now()

        self.ensure_trial_started(
            now
        )

        new_state = msg.data.strip()

        if new_state == "":
            return


        if new_state != self.state:

            old_state = self.state

            self.state = new_state

            elapsed = self.relative_time(
                now
            )

            rospy.loginfo(
                "LOGGER STATE: %s -> %s | t=%.3f s",
                old_state,
                new_state,
                elapsed
            )


            # First entry time for each state
            if new_state not in self.state_first_times:

                self.state_first_times[
                    new_state
                ] = elapsed


            if new_state == "VISION_LOST":

                self.vision_lost_count += 1


            elif new_state == "DEPTH_LOST":

                self.depth_lost_count += 1


            elif new_state == "READY":

                if not self.success:

                    self.success = True

                    self.ready_time = now

                    self.final_distance = (
                        self.front_distance
                    )

                    self.completed = True

                    rospy.loginfo(
                        "========================================"
                    )

                    rospy.loginfo(
                        "TRIAL %03d SUCCESS -> READY",
                        self.trial_id
                    )

                    if (
                        self.final_distance
                        is not None
                    ):
                        rospy.loginfo(
                            "Final distance = %.3f m",
                            self.final_distance
                        )

                    rospy.loginfo(
                        "Total time = %.3f s",
                        self.relative_time(
                            self.ready_time
                        )
                    )

                    rospy.loginfo(
                        "========================================"
                    )

        else:
            self.state = new_state


    # ================================================================
    # Vision
    # ================================================================

    def valid_callback(
        self,
        msg
    ):
        self.target_valid = msg.data

        self.maybe_record_target_acquisition()


    def type_callback(
        self,
        msg
    ):
        self.target_type = msg.data

        self.maybe_record_target_acquisition()


    def center_callback(
        self,
        msg
    ):
        self.target_center_x = float(
            msg.data
        )


    def confidence_callback(
        self,
        msg
    ):
        self.target_confidence = float(
            msg.data
        )

        self.maybe_record_target_acquisition()


    def maybe_record_target_acquisition(self):

        if self.trial_start_time is None:
            return

        if self.target_acquired_time is not None:
            return

        valid_target = (
            self.target_valid
            and
            self.target_type
            ==
            "elevator_call_button"
            and
            self.target_confidence
            >=
            self.min_confidence
        )

        if not valid_target:
            return


        self.target_acquired_time = (
            rospy.Time.now()
        )

        rospy.loginfo(
            "TARGET ACQUISITION recorded at %.3f s",
            self.relative_time(
                self.target_acquired_time
            )
        )


    # ================================================================
    # Depth
    # ================================================================

    def depth_callback(
        self,
        msg
    ):
        value = float(
            msg.data
        )

        if (
            not math.isfinite(value)
            or
            value <= 0.0
        ):
            return

        self.front_distance = value


# First depth is only meaningful for this experiment
# AFTER the visual target has been acquired.
#
# Depth observed during SEARCH may belong to another wall.

        if (
            self.target_acquired_time
            is not None
            and
            self.first_depth_time
            is None
        ):
            self.first_depth_time = (
                rospy.Time.now()
            )

            rospy.loginfo(
                "First target-phase depth recorded at %.3f s | %.3f m",
                self.relative_time(
                    self.first_depth_time
                ),
                self.front_distance
            )


    # ================================================================
    # cmd_vel
    # ================================================================

    def cmd_callback(
        self,
        msg
    ):
        self.cmd_linear_x = float(
            msg.linear.x
        )

        self.cmd_angular_z = float(
            msg.angular.z
        )


    # ================================================================
    # Gazebo model states
    # ================================================================

    def model_states_callback(
        self,
        msg
    ):
        # ------------------------------------------------------------
        # Robot
        # ------------------------------------------------------------

        try:
            robot_index = msg.name.index(
                self.robot_model_name
            )

        except ValueError:

            rospy.logwarn_throttle(
                5.0,
                "Robot model '%s' not found in /gazebo/model_states",
                self.robot_model_name
            )

            return


        # ------------------------------------------------------------
        # Elevator
        # ------------------------------------------------------------

        try:
            elevator_index = msg.name.index(
                self.elevator_model_name
            )

        except ValueError:

            rospy.logwarn_throttle(
                5.0,
                "Elevator model '%s' not found in /gazebo/model_states",
                self.elevator_model_name
            )

            return


        robot_pose = msg.pose[
            robot_index
        ]

        elevator_pose = msg.pose[
            elevator_index
        ]


        # ------------------------------------------------------------
        # Both values below are in Gazebo WORLD frame.
        # ------------------------------------------------------------

        self.robot_x = float(
            robot_pose.position.x
        )

        self.robot_y = float(
            robot_pose.position.y
        )

        self.robot_yaw = (
            self.quaternion_to_yaw(
                robot_pose.orientation
            )
        )


        self.elevator_x = float(
            elevator_pose.position.x
        )

        self.elevator_y = float(
            elevator_pose.position.y
        )


        dx = (
            self.elevator_x
            -
            self.robot_x
        )

        dy = (
            self.elevator_y
            -
            self.robot_y
        )


        # Planar model-origin distance in WORLD frame.
        self.current_model_distance = (
            math.hypot(
                dx,
                dy
            )
        )


        # Bearing from Go1 toward elevator,
        # expressed in WORLD frame.
        bearing_world = math.atan2(
            dy,
            dx
        )


        # Robot yaw is also WORLD-frame yaw,
        # therefore subtraction is frame-consistent.
        self.current_heading_error = (
            self.wrap_angle(
                bearing_world
                -
                self.robot_yaw
            )
        )


        self.capture_initial_pose_if_ready()


    # ================================================================
    # Initial condition
    # ================================================================

    def capture_initial_pose_if_ready(self):

        if self.initial_pose_recorded:
            return

        if self.trial_start_time is None:
            return

        values = [
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            self.elevator_x,
            self.elevator_y,
            self.current_model_distance,
            self.current_heading_error
        ]

        if any(
            value is None
            for value in values
        ):
            return


        self.initial_robot_x = (
            self.robot_x
        )

        self.initial_robot_y = (
            self.robot_y
        )

        self.initial_robot_yaw = (
            self.robot_yaw
        )

        self.initial_elevator_x = (
            self.elevator_x
        )

        self.initial_elevator_y = (
            self.elevator_y
        )

        self.initial_model_distance = (
            self.current_model_distance
        )

        self.initial_heading_error = (
            self.current_heading_error
        )

        self.initial_pose_recorded = True


        rospy.loginfo(
            "Initial condition captured"
        )

        rospy.loginfo(
            "Initial model distance = %.3f m",
            self.initial_model_distance
        )

        rospy.loginfo(
            "Initial heading error = %.2f deg",
            math.degrees(
                self.initial_heading_error
            )
        )


    # ================================================================
    # Time-series logging
    # ================================================================

    def log_timer_callback(
        self,
        event
    ):
        if self.trial_start_time is None:
            return

        if self.finalized:
            return


        elapsed = self.relative_time()


        row = {
            "time_s":
                elapsed,

            "state":
                self.state,

            "target_valid":
                int(
                    self.target_valid
                ),

            "target_type":
                self.target_type,

            "target_center_x_px":
                self.target_center_x,

            "target_confidence":
                self.target_confidence,

            "front_distance_m":
                self.value_or_blank(
                    self.front_distance
                ),

            "cmd_linear_x_mps":
                self.cmd_linear_x,

            "cmd_angular_z_radps":
                self.cmd_angular_z,

            "robot_x_world_m":
                self.value_or_blank(
                    self.robot_x
                ),

            "robot_y_world_m":
                self.value_or_blank(
                    self.robot_y
                ),

            "robot_yaw_world_rad":
                self.value_or_blank(
                    self.robot_yaw
                ),

            "elevator_x_world_m":
                self.value_or_blank(
                    self.elevator_x
                ),

            "elevator_y_world_m":
                self.value_or_blank(
                    self.elevator_y
                ),

            "model_distance_world_m":
                self.value_or_blank(
                    self.current_model_distance
                ),

            "heading_error_rad":
                self.value_or_blank(
                    self.current_heading_error
                )
        }


        self.trial_writer.writerow(
            row
        )


    # ================================================================
    # Summary helper
    # ================================================================

    def state_entry_time(
        self,
        state_name
    ):
        return self.state_first_times.get(
            state_name,
            ""
        )


    def build_summary_row(self):

        if self.success:
            final_state = "READY"
            failure_reason = ""
        else:
            final_state = self.state

            failure_reason = (
                self.failure_reason
                if self.failure_reason
                else
                "MANUAL_STOP"
            )


        if self.ready_time is not None:

            total_task_time = (
                self.relative_time(
                    self.ready_time
                )
            )

        elif self.trial_start_time is not None:

            total_task_time = (
                self.relative_time()
            )

        else:
            total_task_time = ""


        acquisition_time = ""

        if self.target_acquired_time is not None:

            acquisition_time = (
                self.relative_time(
                    self.target_acquired_time
                )
            )


        first_depth_time = ""

        if self.first_depth_time is not None:

            first_depth_time = (
                self.relative_time(
                    self.first_depth_time
                )
            )


        initial_heading_deg = ""

        if self.initial_heading_error is not None:

            initial_heading_deg = (
                math.degrees(
                    self.initial_heading_error
                )
            )


        return {
            "trial_id":
                self.trial_id,

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "success":
                int(
                    self.success
                ),

            "final_state":
                final_state,

            "failure_reason":
                failure_reason,

            "initial_model_distance_m":
                self.value_or_blank(
                    self.initial_model_distance
                ),

            "initial_heading_error_deg":
                self.value_or_blank(
                    initial_heading_deg
                ),

            "target_acquisition_time_s":
                acquisition_time,

            "search_entry_s":
                self.state_entry_time(
                    "SEARCH"
                ),

            "align_entry_s":
                self.state_entry_time(
                    "ALIGN"
                ),

            "far_approach_entry_s":
                self.state_entry_time(
                    "FAR_APPROACH"
                ),

            "first_depth_time_s":
                first_depth_time,

            "depth_approach_entry_s":
                self.state_entry_time(
                    "DEPTH_APPROACH"
                ),

            "ready_entry_s":
                self.state_entry_time(
                    "READY"
                ),

            "total_task_time_s":
                total_task_time,

            "final_distance_m":
                self.value_or_blank(
                    self.final_distance
                ),

            "vision_lost_count":
                self.vision_lost_count,

            "depth_lost_count":
                self.depth_lost_count
        }


    # ================================================================
    # Summary CSV
    # ================================================================

    def write_summary(self):

        summary_fields = [
            "trial_id",
            "timestamp",

            "success",
            "final_state",
            "failure_reason",

            "initial_model_distance_m",
            "initial_heading_error_deg",

            "target_acquisition_time_s",

            "search_entry_s",
            "align_entry_s",
            "far_approach_entry_s",

            "first_depth_time_s",
            "depth_approach_entry_s",

            "ready_entry_s",
            "total_task_time_s",

            "final_distance_m",

            "vision_lost_count",
            "depth_lost_count"
        ]


        row = self.build_summary_row()


        existing_rows = []


        if self.summary_csv_path.exists():

            with open(
                self.summary_csv_path,
                "r",
                newline=""
            ) as f:

                reader = csv.DictReader(
                    f
                )

                for old_row in reader:

                    # If overwrite is enabled,
                    # replace the existing trial row.
                    if (
                        self.overwrite
                        and
                        old_row.get(
                            "trial_id"
                        )
                        ==
                        str(
                            self.trial_id
                        )
                    ):
                        continue

                    existing_rows.append(
                        old_row
                    )


        existing_rows.append(
            row
        )


        with open(
            self.summary_csv_path,
            "w",
            newline=""
        ) as f:

            writer = csv.DictWriter(
                f,
                fieldnames=summary_fields
            )

            writer.writeheader()

            writer.writerows(
                existing_rows
            )


    # ================================================================
    # Finalize experiment
    # ================================================================

    def finalize(self):

        if self.finalized:
            return


        self.finalized = True


        try:
            self.trial_file.flush()

            self.trial_file.close()

        except Exception:
            pass


        self.write_summary()


        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "TRIAL %03d SAVED",
            self.trial_id
        )

        rospy.loginfo(
            "Success: %s",
            str(
                self.success
            )
        )

        rospy.loginfo(
            "Final state: %s",
            self.state
        )

        rospy.loginfo(
            "Time-series CSV: %s",
            str(
                self.trial_csv_path
            )
        )

        rospy.loginfo(
            "Summary CSV: %s",
            str(
                self.summary_csv_path
            )
        )

        rospy.loginfo(
            "========================================"
        )


    # ================================================================
    # Shutdown
    # ================================================================

    def on_shutdown(self):

        if (
            not self.success
            and
            not self.failure_reason
        ):
            self.failure_reason = (
                "MANUAL_STOP"
            )

        self.finalize()


    # ================================================================
    # Main loop
    # ================================================================

    def spin(self):

        rate = rospy.Rate(
            20
        )


        while not rospy.is_shutdown():

            # --------------------------------------------------------
            # Trial timeout
            # --------------------------------------------------------

            if (
                self.trial_start_time
                is not None
                and
                not self.completed
                and
                self.timeout_s > 0.0
            ):

                elapsed = (
                    self.relative_time()
                )

                if elapsed >= self.timeout_s:

                    self.failure_reason = (
                        "TIMEOUT"
                    )

                    self.completed = True

                    rospy.logwarn(
                        "TRIAL %03d TIMEOUT after %.1f s",
                        self.trial_id,
                        elapsed
                    )


            # --------------------------------------------------------
            # Trial complete
            # --------------------------------------------------------

            if self.completed:

                self.finalize()

                rospy.signal_shutdown(
                    "Trial completed"
                )

                break


            rate.sleep()


def main():

    rospy.init_node(
        "trial_logger"
    )


    try:

        logger = TrialLogger()

        logger.spin()


    except rospy.ROSInterruptException:

        pass


    except Exception as e:

        rospy.logfatal(
            "Trial logger fatal error: %s",
            str(e)
        )

        raise


if __name__ == "__main__":
    main()