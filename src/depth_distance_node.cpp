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
        : pnh_("~")
    {
        /*
         * 可以以后通过 launch / rosparam 修改
         */
        pnh_.param<std::string>(
            "depth_topic",
            depth_topic_,
            "/camera_face/depth/image_raw"
        );

        pnh_.param<int>(
            "roi_half_width",
            roi_half_width_,
            30
        );

        pnh_.param<int>(
            "roi_half_height",
            roi_half_height_,
            30
        );

        pnh_.param<double>(
            "min_depth",
            min_depth_,
            0.10
        );

        pnh_.param<double>(
            "max_depth",
            max_depth_,
            1.50
        );

        pnh_.param<int>(
            "min_valid_pixels",
            min_valid_pixels_,
            30
        );


        depth_sub_ =
            nh_.subscribe(
                depth_topic_,
                1,
                &DepthDistanceNode::depthCallback,
                this
            );


        distance_pub_ =
            nh_.advertise<std_msgs::Float32>(
                "/front_distance",
                10
            );


        ROS_INFO(
            "Depth distance node started"
        );

        ROS_INFO(
            "Depth topic: %s",
            depth_topic_.c_str()
        );

        ROS_INFO(
            "ROI: center +/- %d px horizontally, +/- %d px vertically",
            roi_half_width_,
            roi_half_height_
        );

        ROS_INFO(
            "Valid depth range: %.2f m ~ %.2f m",
            min_depth_,
            max_depth_
        );
    }


private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber depth_sub_;
    ros::Publisher distance_pub_;


    std::string depth_topic_;

    int roi_half_width_;
    int roi_half_height_;

    double min_depth_;
    double max_depth_;

    int min_valid_pixels_;


    void depthCallback(
        const sensor_msgs::Image::ConstPtr& msg
    )
    {
        /*
         * 当前节点只支持已经确认的：
         *
         * encoding = 32FC1
         *
         * 每个像素占 4 bytes。
         */

        if(msg->encoding != "32FC1")
        {
            ROS_ERROR_THROTTLE(
                2.0,
                "Unsupported depth encoding: %s, expected 32FC1",
                msg->encoding.c_str()
            );

            return;
        }


        if(msg->width == 0 || msg->height == 0)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "Depth image is empty"
            );

            return;
        }


        /*
         * 目前你的数据：
         *
         * frame_id = camera_optical_face
         *
         * 这里只计算图像中央区域深度，
         * 不把这个坐标与 base / odom 直接混合。
         */

        const int center_u =
            static_cast<int>(msg->width / 2);

        const int center_v =
            static_cast<int>(msg->height / 2);


        int u_min =
            center_u - roi_half_width_;

        int u_max =
            center_u + roi_half_width_;

        int v_min =
            center_v - roi_half_height_;

        int v_max =
            center_v + roi_half_height_;


        /*
         * 防止 ROI 超出图像范围
         */

        u_min =
            std::max(
                0,
                u_min
            );

        v_min =
            std::max(
                0,
                v_min
            );

        u_max =
            std::min(
                static_cast<int>(msg->width) - 1,
                u_max
            );

        v_max =
            std::min(
                static_cast<int>(msg->height) - 1,
                v_max
            );


        std::vector<float> valid_depths;

        valid_depths.reserve(
            (u_max - u_min + 1) *
            (v_max - v_min + 1)
        );


        /*
         * 读取 ROI 内所有深度值
         */

        for(int v = v_min; v <= v_max; ++v)
        {
            for(int u = u_min; u <= u_max; ++u)
            {
                /*
                 * 32FC1:
                 *
                 * 每个像素 4 bytes
                 *
                 * offset =
                 * row_start + pixel_offset
                 */

                const size_t offset =
                    static_cast<size_t>(v) *
                    msg->step
                    +
                    static_cast<size_t>(u) *
                    sizeof(float);


                if(
                    offset + sizeof(float)
                    >
                    msg->data.size()
                )
                {
                    continue;
                }


                float depth;


                /*
                 * 当前消息：
                 *
                 * is_bigendian = 0
                 *
                 * WSL / x86 同样是 little endian，
                 * 所以可以直接 memcpy。
                 */

                std::memcpy(
                    &depth,
                    &msg->data[offset],
                    sizeof(float)
                );


                /*
                 * 去掉：
                 *
                 * NaN
                 * Inf
                 * <=0
                 * 太近
                 * 太远
                 */

                if(!std::isfinite(depth))
                {
                    continue;
                }


                if(depth < min_depth_)
                {
                    continue;
                }


                if(depth > max_depth_)
                {
                    continue;
                }


                valid_depths.push_back(depth);
            }
        }


        /*
         * 有效像素太少：
         *
         * 不发布旧值，也不编造距离。
         */

        if(
            static_cast<int>(valid_depths.size())
            <
            min_valid_pixels_
        )
        {
            ROS_WARN_THROTTLE(
                1.0,
                "Not enough valid depth pixels: %zu",
                valid_depths.size()
            );

            return;
        }


        /*
         * 求中位数
         *
         * 相比平均值：
         * 对异常深度值更稳定。
         */

        const size_t middle =
            valid_depths.size() / 2;


        std::nth_element(
            valid_depths.begin(),
            valid_depths.begin() + middle,
            valid_depths.end()
        );


        float median_depth =
            valid_depths[middle];


        /*
         * 偶数个数据时，
         * 为了控制程序简单，
         * 直接使用中间的一个值即可。
         *
         * 对几十/几千个深度像素来说
         * 区别可以忽略。
         */


        std_msgs::Float32 distance_msg;

        distance_msg.data =
            median_depth;


        distance_pub_.publish(
            distance_msg
        );


        ROS_INFO_THROTTLE(
            0.5,
            "Front distance = %.3f m | valid pixels = %zu | frame = %s",
            median_depth,
            valid_depths.size(),
            msg->header.frame_id.c_str()
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
        "depth_distance_node"
    );


    DepthDistanceNode node;


    ros::spin();


    return 0;
}