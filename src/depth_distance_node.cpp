#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>


class DepthDistanceNode
{
public:
    DepthDistanceNode()
        : nh_(),
          pnh_("~")
    {
        // ============================================================
        // Parameters
        // ============================================================

        pnh_.param<std::string>(
            "depth_topic",
            depth_topic_,
            "/camera_face/depth/image_raw"
        );

        pnh_.param(
            "min_depth",
            min_depth_,
            0.10
        );

        pnh_.param(
            "max_depth",
            max_depth_,
            1.50
        );

        pnh_.param(
            "roi_half_width",
            roi_half_width_,
            30
        );

        pnh_.param(
            "roi_half_height",
            roi_half_height_,
            30
        );

        pnh_.param(
            "min_valid_pixels",
            min_valid_pixels_,
            30
        );

        pnh_.param(
            "search_step_px",
            search_step_px_,
            20
        );

        pnh_.param(
            "max_search_offset_px",
            max_search_offset_px_,
            250
        );

        // ============================================================
        // Publisher
        // ============================================================

        distance_pub_ =
            nh_.advertise<std_msgs::Float32>(
                "/front_distance",
                1
            );

        // ============================================================
        // Subscriber
        // ============================================================

        depth_sub_ =
            nh_.subscribe(
                depth_topic_,
                1,
                &DepthDistanceNode::depthCallback,
                this
            );

        ROS_INFO(
            "Adaptive depth distance node started"
        );

        ROS_INFO(
            "Depth topic: %s",
            depth_topic_.c_str()
        );

        ROS_INFO(
            "Depth range: %.2f m ~ %.2f m",
            min_depth_,
            max_depth_
        );

        ROS_INFO(
            "ROI: +/- %d px horizontally, +/- %d px vertically",
            roi_half_width_,
            roi_half_height_
        );

        ROS_INFO(
            "Adaptive horizontal search: step=%d px, max_offset=%d px",
            search_step_px_,
            max_search_offset_px_
        );

        ROS_INFO(
            "Minimum valid pixels: %d",
            min_valid_pixels_
        );
    }


private:

    // ================================================================
    // Calculate median depth inside one ROI
    // ================================================================

    bool evaluateROI(
        const sensor_msgs::ImageConstPtr& msg,
        int center_x,
        int center_y,
        float& median_depth,
        int& valid_count
    )
    {
        const int width =
            static_cast<int>(msg->width);

        const int height =
            static_cast<int>(msg->height);

        const int x1 =
            std::max(
                0,
                center_x - roi_half_width_
            );

        const int x2 =
            std::min(
                width,
                center_x + roi_half_width_
            );

        const int y1 =
            std::max(
                0,
                center_y - roi_half_height_
            );

        const int y2 =
            std::min(
                height,
                center_y + roi_half_height_
            );

        std::vector<float> values;

        values.reserve(
            (x2 - x1) * (y2 - y1)
        );

        // 32FC1 = float32, one float per depth pixel.
        //
        // msg->step may contain row padding, so use step explicitly.

        for (int y = y1; y < y2; ++y)
        {
            const uint8_t* row_ptr =
                &msg->data[
                    static_cast<size_t>(y)
                    * msg->step
                ];

            for (int x = x1; x < x2; ++x)
            {
                float depth = 0.0f;

                std::memcpy(
                    &depth,
                    row_ptr
                        + static_cast<size_t>(x)
                        * sizeof(float),
                    sizeof(float)
                );

                if (!std::isfinite(depth))
                {
                    continue;
                }

                if (depth < min_depth_)
                {
                    continue;
                }

                if (depth > max_depth_)
                {
                    continue;
                }

                values.push_back(
                    depth
                );
            }
        }

        valid_count =
            static_cast<int>(
                values.size()
            );

        if (
            valid_count
            < min_valid_pixels_
        )
        {
            return false;
        }

        // Median
        const size_t mid =
            values.size() / 2;

        std::nth_element(
            values.begin(),
            values.begin() + mid,
            values.end()
        );

        median_depth =
            values[mid];

        return true;
    }


    // ================================================================
    // Depth callback
    // ================================================================

    void depthCallback(
        const sensor_msgs::ImageConstPtr& msg
    )
    {
        // ------------------------------------------------------------
        // Encoding check
        // ------------------------------------------------------------

        if (
            msg->encoding != "32FC1"
            && msg->encoding != "32FC"
        )
        {
            ROS_WARN_THROTTLE(
                2.0,
                "Unsupported depth encoding: %s",
                msg->encoding.c_str()
            );

            return;
        }

        if (
            msg->width == 0
            || msg->height == 0
            || msg->data.empty()
        )
        {
            ROS_WARN_THROTTLE(
                2.0,
                "Empty depth image"
            );

            return;
        }

        const int image_width =
            static_cast<int>(
                msg->width
            );

        const int image_height =
            static_cast<int>(
                msg->height
            );

        const int center_x =
            image_width / 2;

        const int center_y =
            image_height / 2;

        // ------------------------------------------------------------
        // Adaptive search offsets
        //
        // Order:
        //
        // 0
        // -20, +20
        // -40, +40
        // ...
        //
        // We take the first ROI containing enough valid depth pixels.
        // Therefore the selected ROI is the valid one nearest the
        // original depth-image centre.
        // ------------------------------------------------------------

        std::vector<int> offsets;

        offsets.push_back(
            0
        );

        for (
            int offset = search_step_px_;
            offset <= max_search_offset_px_;
            offset += search_step_px_
        )
        {
            // Left first because current Gazebo depth data
            // is observed to be shifted left of image centre.
            offsets.push_back(
                -offset
            );

            offsets.push_back(
                offset
            );
        }

        bool found = false;

        float selected_depth =
            0.0f;

        int selected_offset =
            0;

        int selected_valid_count =
            0;

        // ------------------------------------------------------------
        // Search
        // ------------------------------------------------------------

        for (const int offset : offsets)
        {
            const int test_center_x =
                center_x + offset;

            if (
                test_center_x
                    - roi_half_width_
                < 0
            )
            {
                continue;
            }

            if (
                test_center_x
                    + roi_half_width_
                >= image_width
            )
            {
                continue;
            }

            float median_depth =
                0.0f;

            int valid_count =
                0;

            const bool valid =
                evaluateROI(
                    msg,
                    test_center_x,
                    center_y,
                    median_depth,
                    valid_count
                );

            if (!valid)
            {
                continue;
            }

            found = true;

            selected_depth =
                median_depth;

            selected_offset =
                offset;

            selected_valid_count =
                valid_count;

            break;
        }

        // ------------------------------------------------------------
        // Nothing usable
        // ------------------------------------------------------------

        if (!found)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "No valid depth ROI found within +/- %d px",
                max_search_offset_px_
            );

            // Important:
            //
            // Do NOT publish fake depth.
            //
            // The controller will detect timeout and safely enter
            // DEPTH_LOST if depth had already been acquired.
            return;
        }

        // ------------------------------------------------------------
        // Publish
        // ------------------------------------------------------------

        std_msgs::Float32 distance_msg;

        distance_msg.data =
            selected_depth;

        distance_pub_.publish(
            distance_msg
        );

        ROS_INFO_THROTTLE(
            1.0,
            "Front distance = %.3f m | "
            "valid pixels = %d | "
            "ROI offset = %+d px | "
            "ROI center x = %d | "
            "depth image center x = %d | "
            "frame = %s",
            selected_depth,
            selected_valid_count,
            selected_offset,
            center_x + selected_offset,
            center_x,
            msg->header.frame_id.c_str()
        );
    }


private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber depth_sub_;
    ros::Publisher distance_pub_;

    std::string depth_topic_;

    double min_depth_;
    double max_depth_;

    int roi_half_width_;
    int roi_half_height_;

    int min_valid_pixels_;

    int search_step_px_;
    int max_search_offset_px_;
};


int main(
    int argc,
    char** argv
)
{
    ros::init(
        argc,
        argv,
        "depth_distance_node"
    );

    DepthDistanceNode node;

    ros::spin();

    return 0;
}