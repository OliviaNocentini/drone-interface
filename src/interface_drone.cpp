#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

struct point{
  
std::vector<float> position;

} waypoint[3];

geometry_msgs::PoseStamped set_pose;

void set_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    set_pose = *msg;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{

  //float soglia=0.5;
  
  std::vector<float> p0={0,0,5}; 
  std::vector<float> p1={3,0,5}; 
  std::vector<float> p2={0,0,2};
  
  waypoint[0].position=p0;
  waypoint[1].position=p1;
  waypoint[2].position=p2;
  
  
  
  
  
    ros::init(argc, argv, "interface_drone_node");
    ros::NodeHandle nh;
  
    ROS_INFO("Test_info");
   
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("mavros/state", 10, state_cb);
    ros::Subscriber set_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
           ("mavros/setpoint_position/local", 10, set_pose_cb);
    ros::Publisher set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);		
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
	ROS_INFO("State Not Connected");

    }
    
    //geometry_msgs::PoseStamped current_pose;
    
   // std::cout << "Definisco il mio primo obiettivo" <<std::endl;

   //set_pose.pose.position.x=p0[0];
   //set_pose.pose.position.y=p0[1];
  // set_pose.pose.position.z=p0[2];
   
   //int waypoint_index = 0;
   
   std::cout << "pos x"<<"\n"<< set_pose.pose.position.x << "\n" << " pos y"<< "\n"<<set_pose.pose.position.y<<"\n"<< "pos z"<<"\n"<< set_pose.pose.position.z << std::endl;
   
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        set_pose_pub.publish(set_pose);
        ros::spinOnce();
        rate.sleep();
	ROS_INFO("%d...",i);
    }
    
    ROS_INFO("Position Selected");

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
        
        
        set_pose_pub.publish(set_pose);
	
        
	/*if((fabs(set_pose.pose.position.x-current_pose.pose.position.x) < soglia) && 
	  ( fabs(set_pose.pose.position.y-current_pose.pose.position.y) <soglia) && 
	  ( fabs(set_pose.pose.position.z-current_pose.pose.position.z) < soglia))
      
	{
	    waypoint_index++;
	    set_pose.pose.position.x=waypoint[waypoint_index%3].position[0];
	    set_pose.pose.position.y=waypoint[waypoint_index%3].position[1];
	    set_pose.pose.position.z=waypoint[waypoint_index%3].position[2];
	      
	    std::cout << waypoint_index << "- new set pose (x,y,z):" << 
	    set_pose.pose.position.x << ", " 
	    <<set_pose.pose.position.y<<", "
	    <<set_pose.pose.position.z << std::endl;
	}
    
	//std::cout << "pose x:"<<"\n"<< current_pose.pose.position.x << "\n" << " pose y:"<< "\n"<<current_pose.pose.position.y<<"\n"<< "pose z:"<<"\n"<< current_pose.pose.position.z << std::endl;
      
	 */
         ros::spinOnce();
         rate.sleep();
          
    }
      
    return 0;   
    
    
}