#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <cmath>


enum TaskState
{
    SEARCHING,
    APPROACHING,
    READY,
    LOST
};



class TaskManager
{

public:

    TaskManager()
    {

        state = SEARCHING;


        target_valid = false;


        target_x = 0;
        target_y = 0;


        first_seen = false;


        target_sub =
        nh.subscribe(
            "/goal_pose",
            10,
            &TaskManager::targetCallback,
            this
        );


        state_pub =
        nh.advertise<std_msgs::String>(
            "/task_state",
            10
        );


        last_seen_time =
        ros::Time::now();


        first_seen_time =
        ros::Time::now();



        ROS_INFO(
            "Task manager started"
        );

    }



    void spin()
    {

        ros::Rate rate(10);


        while(ros::ok())
        {

            ros::spinOnce();


            updateState();


            publishState();


            rate.sleep();

        }

    }



private:


    ros::NodeHandle nh;


    ros::Subscriber target_sub;

    ros::Publisher state_pub;



    double target_x;
    double target_y;



    bool target_valid;


    bool first_seen;



    ros::Time first_seen_time;

    ros::Time last_seen_time;



    TaskState state;



    /*
     * 参数
     */


    const double READY_DISTANCE = 0.5;


    const double CONFIRM_TIME = 0.5;


    const double LOST_TIME = 2.0;


    const double READY_HOLD_TIME = 1.0;





    void targetCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg
    )
    {


        /*
         *
         * 注意：
         *
         * /goal_pose
         * 已经是 base frame
         *
         */


        target_x =
        msg->pose.position.x;


        target_y =
        msg->pose.position.y;



        ros::Time now =
        ros::Time::now();



        last_seen_time = now;



        if(!first_seen)
        {

            first_seen = true;

            first_seen_time = now;

        }



        double confirm_time =
        (
            now-first_seen_time
        ).toSec();



        if(confirm_time > CONFIRM_TIME)
        {

            target_valid = true;

        }


    }






    void updateState()
    {

        ros::Time now =
        ros::Time::now();



        double lost_time =
        (
            now-last_seen_time
        ).toSec();



        /*
         *
         * 长时间没有目标
         *
         */

        if(lost_time > LOST_TIME)
        {

            state = LOST;

            return;

        }



        /*
         *
         * 目标还未确认
         *
         */

        if(!target_valid)
        {

            state = SEARCHING;

            return;

        }




        /*
         *
         * 目标在base坐标系
         *
         * 直接计算距离
         *
         */


        double distance =
        sqrt(
            target_x*target_x
            +
            target_y*target_y
        );



        if(distance < READY_DISTANCE)
        {

            state = READY;

        }

        else
        {

            state = APPROACHING;

        }


    }





    void publishState()
    {

        std_msgs::String msg;



        switch(state)
        {


        case SEARCHING:

            msg.data="SEARCHING";

            break;


        case APPROACHING:

            msg.data="APPROACHING";

            break;


        case READY:

            msg.data="READY";

            break;


        case LOST:

            msg.data="LOST";

            break;

        }



        state_pub.publish(msg);



        ROS_INFO_THROTTLE(
            1,
            "Task state: %s",
            msg.data.c_str()
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
        "task_manager"
    );


    TaskManager node;


    node.spin();


    return 0;

}