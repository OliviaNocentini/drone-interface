#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2_ros/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::TransformStamped transformStamped;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
     
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	    
    ros::Publisher set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    ("mavros/setpoint_velocity/cmd_vel", 10);	   
   

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

  

    //send a few setpoints before starting
   /* for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }*/

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    
 

  
  while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
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

  float x_drone=transformStamped.transform.translation.x;
  float y_drone=transformStamped.transform.translation.y;
  float z_drone=transformStamped.transform.translation.z;
  
  float roll_drone(0),pitch_drone(0),yaw_drone;
  float q0_drone=transformStamped.transform.rotation.x;
  float q1_drone=transformStamped.transform.rotation.y;
  float q2_drone=transformStamped.transform.rotation.z;
  float q3_drone=transformStamped.transform.rotation.w;
  
  
  yaw_drone= tf::getYaw(transformStamped.transform.rotation);
  
  geometry_msgs::PoseStamped set_pose;
  
  set_pose.pose.position.x=0;
  set_pose.pose.position.y=0;
  set_pose.pose.position.z=2;
  
  
  geometry_msgs::TwistStamped com;
  
  com.header.stamp=ros::Time::now();
  com.header.frame_id="iris/base_link";

  float Kp_linear=0;
  
  // settare linear com
  
  com.twist.linear.x = Kp_linear *fabs(set_pose.pose.position.x-x_drone) ;
  com.twist.linear.y = Kp_linear *fabs(set_pose.pose.position.y-y_drone) ;
  com.twist.linear.z = Kp_linear *fabs(set_pose.pose.position.z-z_drone);
  
 
  // set angular command
  com.twist.angular.x = 0;
  com.twist.angular.y = 0;
  com.twist.angular.z =0.5;

  
  set_vel_pub.publish(com);
  
  std::cout << "com:" << com << std::endl;
        
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}