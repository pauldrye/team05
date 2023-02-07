#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    double err_ang;
    double prev_err_ang = 0;
    double p_ang = 0;
    double i_ang = 0;
    double d_ang = 0;

    double err_lin;
    double prev_err_lin = 0;
    double p_lin = 0;
    double i_lin = 0;
    double d_lin = 0;
    
    double prev_cmd_lin_vel = 0;
    double prev_cmd_ang_vel = 0;
    double lin_acc;
    double ang_acc;

    double t1 = 0;
    bool t01 = 0;
    double t2 = 0;
    bool rise_time = 0;
    double max_overshoot = 0;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();
            /* if((limit_angle(ang_rbt) >= 0.1*M_PI) and (!t01))// if((target.x - pos_rbt.x <= 0.9*(target.x+2)) and (target.x - pos_rbt.x >= 0.88*(target.x+2)) and (!rise_time))
            {
                t1 = prev_time;
                t01 = 1;
            }
            if((limit_angle(ang_rbt) >= 0.9*M_PI) and (!rise_time))  //if((target.x - pos_rbt.x <= 0.1*(target.x+2)) and (target.x - pos_rbt.x >= 0.05*(target.x+2)) and (!rise_time))
            {
                t2 = prev_time;
                rise_time = 1;
            }
            ROS_INFO("TIME(%6.3f), T1(%6.3f), T2(%6.3f), RISE TIME(%6.3f)", prev_time, t1, t2, t2-t1);
            if(err_ang < 0)
            {
                ROS_INFO("OVERSHOOT(%6.3f)", -err_ang);
                if(-err_ang > max_overshoot)
                {
                    max_overshoot = -err_ang;
                }
            }            
            if(pos_rbt.x > target.x)
            {
                ROS_INFO("OVERSHOOT(%6.3f)", pos_rbt.x-target.x);
                if(pos_rbt.x-target.x > max_overshoot)
                {
                    max_overshoot = pos_rbt.x-target.x;
                }
            } */
            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            err_ang = limit_angle(atan2(target.y-pos_rbt.y,target.x-pos_rbt.x) - ang_rbt);
            p_ang = Kp_ang*err_ang;
            i_ang += Ki_ang*err_ang*dt;
            d_ang = Kd_ang*(err_ang - prev_err_ang)/dt;
            prev_err_ang = err_ang;
            cmd_ang_vel = p_ang + i_ang + d_ang;
            
            err_lin = dist_euc(pos_rbt.x, pos_rbt.y, target.x, target.y);
            p_lin = Kp_lin*err_lin;
            i_lin += Ki_lin*err_lin*dt;
            d_lin = Kd_lin*(err_lin - prev_err_lin)/dt;
            cmd_lin_vel = pow(cos(err_ang/2),2)*(p_lin + i_lin + d_lin);
            prev_err_lin = err_lin;
           
            sat((cmd_lin_vel - prev_cmd_lin_vel)/dt, max_lin_acc);
            sat(prev_cmd_lin_vel + lin_acc*dt, max_lin_vel);
            prev_cmd_lin_vel = cmd_lin_vel;
            
            sat((cmd_ang_vel - prev_cmd_ang_vel)/dt, max_ang_acc);
            sat(prev_cmd_ang_vel + ang_acc*dt, max_ang_vel);
            prev_cmd_ang_vel = cmd_ang_vel;
            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);
            // oui
            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
                ROS_INFO("POS : x(%6.3f), y(%6.3f)", pos_rbt.x, pos_rbt.y);
                ROS_INFO("ERR_ANG(%6.3f)", err_ang);
                ROS_INFO("TARGET : x(%6.3f), y(%6.3f)", target.x, target.y);
                ROS_INFO("MAX_O : (%6.3f))", max_overshoot);   
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}