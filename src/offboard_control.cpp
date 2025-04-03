#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Thrust.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

std_msgs::Bool takeOff;
void takeoff_cb(const std_msgs::Bool::ConstPtr& msg) {
  takeOff = *msg;
}

std_msgs::Bool landing;
void landing_cb(const std_msgs::Bool::ConstPtr& msg) {
  landing = *msg;
}

std_msgs::Bool offboard;
void offboard_cb(const std_msgs::Bool::ConstPtr& msg) {
  offboard = *msg;
}

std_msgs::Bool velocityMode;
void detectionCallback(const std_msgs::Bool::ConstPtr& msg) {
  velocityMode = *msg;
}

std_msgs::Bool attitudeMode;
void attitudeModeCallback(const std_msgs::Bool::ConstPtr& msg) {
  attitudeMode = *msg;
}

geometry_msgs::PoseStamped targetPose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  targetPose = *msg;
}

geometry_msgs::TwistStamped targetVelocity;
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  targetVelocity = *msg;
}

geometry_msgs::Quaternion targetAttitude;
void attitude_cb(const geometry_msgs::Quaternion::ConstPtr& msg) {
  targetAttitude = *msg;
}

std_msgs::Float64 targetThrust;
void thrustSetpointCallback(const std_msgs::Float64::ConstPtr& msg) {
  targetThrust = *msg;
}

int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "setpoint_control_node_uav_0");
    ros::NodeHandle nh;
    /*******************************************************/
    /********************* SUBSCRIBERS *********************/
    /*******************************************************/
    
    // MODES
    ros::Subscriber takeOff_sub = nh.subscribe<std_msgs::Bool>
            ("takeOff", 1, takeoff_cb);
    
    ros::Subscriber attitudeMode_sub = nh.subscribe<std_msgs::Bool>
            ("attitude_mode", 1, attitudeModeCallback);
    
    ros::Subscriber landing_sub = nh.subscribe<std_msgs::Bool>
            ("landing", 1, landing_cb);

    ros::Subscriber offboard_sub = nh.subscribe<std_msgs::Bool>
            ("offboard", 1, offboard_cb);
        
    ros::Subscriber velocityMode_sub = nh.subscribe<std_msgs::Bool>
            ("velocity_mode", 1, detectionCallback);
    
    // CONTROL INPUTS
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("target_position", 1, pose_cb);
    
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("target_velocity", 1, velocity_cb);
    
    ros::Publisher local_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    // STATES
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    /*******************************************************/
    /********************** SERVICES ***********************/
    /*******************************************************/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    /*******************************************************/
    /********************* PUBLISHERS***********************/
    /*******************************************************/  
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    
    ros::Subscriber target_thrust_sub = nh.subscribe<std_msgs::Float64>
            ("target_thrust", 1, thrustSetpointCallback);
    
    ros::Subscriber attitude_sub = nh.subscribe<geometry_msgs::Quaternion>
            ("target_attitude", 1, attitude_cb);
    
    ros::Publisher local_thrust_pub = nh.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);
    
    ros::Rate rate(20.0);
    geometry_msgs::PoseStamped set_pose;
    geometry_msgs::TwistStamped set_vel;
    mavros_msgs::AttitudeTarget set_attitude;
    mavros_msgs::Thrust set_thrust;
    targetPose.pose.position.x = 0;
    targetPose.pose.position.y = 0;
    targetPose.pose.position.z = 3;
    
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(set_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode takeoff_set_mode;
    takeoff_set_mode.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if (takeOff.data && !landing.data && offboard.data && current_state.connected){

            if (current_state.system_status != 4){
                if( current_state.mode != "AUTO.TAKEOFF" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( set_mode_client.call(takeoff_set_mode) &&
                        takeoff_set_mode.response.mode_sent){
                        ROS_INFO("Takeoff Mode Set");
                    }
                    last_request = ros::Time::now();
                } else {
                    if( !current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                }
            }

            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                    for(int i = 100; ros::ok() && i > 0; --i){
                        local_pos_pub.publish(set_pose);
                        ros::spinOnce();
                        rate.sleep();
                    }
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            if (velocityMode.data == false &&  attitudeMode.data == false){
                set_pose.pose.position.x = targetPose.pose.position.x;
                set_pose.pose.position.y = targetPose.pose.position.y;
                set_pose.pose.position.z = targetPose.pose.position.z;
                set_pose.pose.orientation.x = targetPose.pose.orientation.x;
                set_pose.pose.orientation.y = targetPose.pose.orientation.y;
                set_pose.pose.orientation.z = targetPose.pose.orientation.z;
                set_pose.pose.orientation.w = targetPose.pose.orientation.w;
                local_pos_pub.publish(set_pose);
                }

            if (velocityMode.data == true && attitudeMode.data == false){
                set_vel.twist.linear.x = targetVelocity.twist.linear.x;
                set_vel.twist.linear.y = targetVelocity.twist.linear.y;
                set_vel.twist.linear.z = targetVelocity.twist.linear.z;
                set_vel.twist.angular.z = targetVelocity.twist.angular.z;
                local_vel_pub.publish(set_vel);
                }
            
            if (velocityMode.data == false && attitudeMode.data == true){
                set_attitude.orientation.x = targetAttitude.x; 
                set_attitude.orientation.y = targetAttitude.y;
                set_attitude.orientation.z = targetAttitude.z;
                set_attitude.orientation.w = targetAttitude.w;
                set_attitude.thrust = targetThrust.data;
                local_att_pub.publish(set_attitude);
                }
        }
        
        // if (!takeOff.data && landing.data && current_state.connected && current_state.system_status == 4){
        //     mavros_msgs::SetMode offb_set_mode;
        //     offb_set_mode.request.custom_mode = "AUTO.LAND";
        //     arm_cmd.request.value = false;

        //     if( current_state.mode != "AUTO.LAND" &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( set_mode_client.call(offb_set_mode) &&
        //             offb_set_mode.response.mode_sent){
        //             ROS_INFO("Landing enabled");
        //         }
        //         last_request = ros::Time::now();
        //     } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //             if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        //                 ROS_INFO("Vehicle dis-armed");
        //                 return 0;
        //             }
        //             last_request = ros::Time::now();
        //         }
        //     }
        // }
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
