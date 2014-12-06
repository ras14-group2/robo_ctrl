#include "ros/ros.h"
#include <ras_arduino_msgs/Encoders.h>
#include <ir_reader/distance_readings.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <robo_globals.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

//Mapping modes
#define STILL 0
#define LEFT_WALL_FOLLOW 1
#define RIGHT_WALL_FOLLOW 2
#define LEFT_ROTATE 3
#define RIGHT_ROTATE 4
#define STRAIGHT_FORWARD 5
#define RIGHT_WALL_ALIGN 6
#define LEFT_WALL_ALIGN 7
#define CONFUSED 8



//Static Offsets for mapping
#define  LFXOFF	-5.5
#define  LFYOFF  9
#define  LBXOFF -5.5
#define  LBYOFF -10
#define  RFXOFF 5.5
#define  RFYOFF 9
#define  RBXOFF 5.5
#define  RBYOFF 10
#define  CFXOFF 0
#define  CFYOFF 9.8
#define  CBXOFF 0
#define  CBYOFF -10
#define  IR_SHORT_LIMIT 25

//static const char* MODE_NAMES_MAPPING[9] = {"STILL", "LEFT_WALL_FOLLOW", "RIGHT_WALL_FOLLOW", "LEFT_ROTATE", "RIGHT_ROTATE", "STRAIGHT_FORWARD", "RIGHT_WALL_ALIGN", "LEFT_WALL_ALIGN", "CONFUSED"};

class maze_mapping
//class maze_mapping_node
{
private:
    int current_mode, current_mapping_mode, prev_mapping_mode;
    int modechange;
	geometry_msgs::Point out_rightpoint;
	geometry_msgs::Point out_leftpoint;
	ir_reader::distance_readings in_ir;
	std_msgs::String in_mode;
	geometry_msgs::TwistStamped in_pose;
	
public:

//	bool hasIR;

	ros::NodeHandle n_;
	ros::Subscriber ir_reader_subscriber_, navigator_mode_subscriber_, pose_subscriber_, navigator_prev_mode_subscriber_;
	ros::Publisher leftpoint_publisher_, rightpoint_publisher_;

	maze_mapping(): current_mapping_mode(STRAIGHT_FORWARD)
	//maze_mapping_node() 
	{
		n_ = ros::NodeHandle("~");
		ir_reader_subscriber_ = n_.subscribe("/ir_reader_node/cdistance", 1, &maze_mapping::irCallback, this);
		navigator_mode_subscriber_ = n_.subscribe<>("/maze_navigator/mode", 1, &maze_mapping::modeCallback, this);
		navigator_prev_mode_subscriber_ = n_.subscribe<>("/maze_navigator/prevmode", 1, &maze_mapping::prevmodeCallback, this);
		pose_subscriber_ = n_.subscribe<>("/posori/Twist", 1, &maze_mapping::poseCallback, this);
		leftpoint_publisher_ = n_.advertise<geometry_msgs::Point>("/mapping/leftpoint", 1000);
		rightpoint_publisher_ = n_.advertise<geometry_msgs::Point>("/mapping/rightpoint", 1000);		
	}

	void modeCallback(const std_msgs::Int16::ConstPtr &msg)
	{	
		if (current_mapping_mode!= msg->data){
			modechange=1;
		}
		else {
			modechange = 0;
		}
		current_mapping_mode = msg->data;		
	}
	void prevmodeCallback(const std_msgs::Int16::ConstPtr &msg)
	{
		prev_mapping_mode = msg->data;	
	}

	void irCallback(const ir_reader::distance_readings::ConstPtr &msg)
	{
		in_ir = *msg;
	}

	void poseCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
		in_pose = *msg;
	}

	void publish()
	{
//	ROS_INFO("modechange is %d", modechange);
//	ROS_INFO("The mode in mapping is %d", current_mapping_mode);
	ROS_INFO("The left wallpoint x and y are %lf, %lf", out_leftpoint.x, out_leftpoint.y);
	ROS_INFO("The right wallpoint x and y are %lf, %lf", out_rightpoint.x, out_rightpoint.y);
	ROS_INFO("The orientation is %lf", in_pose.twist.angular.z);
		switch (current_mapping_mode) {
			case LEFT_WALL_FOLLOW:
			case RIGHT_WALL_FOLLOW:	
//			if (modechange==1){
//				ROS_INFO("am in modechange");

					if(fabs((in_pose.twist.angular.z+TWOPI)-(TWOPI/4))<TWOPI/6){
						if(in_ir.back_left<IR_SHORT_LIMIT){
		//					out_leftpoint.x = in_pose.twist.linear.x -in_ir.back_left/100 + LBXOFF/100;
								out_leftpoint.x = in_pose.twist.linear.x + in_ir.back_left/100 + LBXOFF/100;  
		//				out_leftpoint.y = in_pose.twist.y - pose_ref.y; 
							out_leftpoint.y = in_pose.twist.linear.y;
							out_leftpoint.z=0;
						}
						if(in_ir.back_right<IR_SHORT_LIMIT){
		//					out_rightpoint.x = in_pose.twist.linear.x + in_ir.back_right/100 + RBXOFF/100; 
							out_rightpoint.x = in_pose.twist.linear.x - in_ir.back_right/100 + RBXOFF/100; 
		//					out_rightpoint.y = pose.y - pose_ref.y; 
							out_rightpoint.y = in_pose.twist.linear.y; 
							out_rightpoint.z=0;
						}					
				}
					else if (fabs((in_pose.twist.angular.z+TWOPI)-(TWOPI/2))<TWOPI/6){
						if(in_ir.back_left<IR_SHORT_LIMIT){
		//				out_leftpoint.x = in_pose_ref.x - pose.; //EDIT
							out_leftpoint.x = in_pose.twist.linear.x; //EDIT
	//						out_leftpoint.y = in_pose.twist.linear.y - in_ir.back_left/100 + LBXOFF/100; 
							out_leftpoint.y = in_pose.twist.linear.y + in_ir.back_left/100 + LBXOFF/100; 
							out_leftpoint.z=0;
						}
						if(in_ir.back_right<IR_SHORT_LIMIT){
							out_rightpoint.x =in_pose.twist.linear.x;
		//					out_rightpoint.y = in_pose.twist.linear.y + in_ir.back_right/100 + RBYOFF/100; //EDIT
							out_rightpoint.y = in_pose.twist.linear.y - in_ir.back_right/100 + RBYOFF/100; //EDIT
							out_rightpoint.z=0;
						}					
					}
					
		//			else if (fabs(in_pose.twist.angular.z-(3*TWOPI/4))<TWOPI/6){
		else if (fabs((in_pose.twist.angular.z+TWOPI)-(3*TWOPI/4))<TWOPI/6){				
						if(in_ir.back_left<IR_SHORT_LIMIT){
		//				out_leftpoint.x = pose_ref.x - pose.; //EDIT
		//					out_leftpoint.x = in_pose.twist.linear.x + in_ir.back_left/100 + LBXOFF/100; //EDIT
							out_leftpoint.x = in_pose.twist.linear.x - in_ir.back_left/100 + LBXOFF/100; //EDIT
							out_leftpoint.y = in_pose.twist.linear.y;  
							out_leftpoint.z=0;
						}
						if(in_ir.back_right<IR_SHORT_LIMIT){
	//						out_rightpoint.x = in_pose.twist.linear.x - in_ir.back_right/100 + RBXOFF/100; //EDIT
							out_rightpoint.x = in_pose.twist.linear.x + in_ir.back_right/100 + RBXOFF/100; //EDIT
							out_rightpoint.y = in_pose.twist.linear.y; 
							out_rightpoint.z=0;
						}					
					}
					
					else if (fabs(in_pose.twist.angular.z-0)<TWOPI/6){
						if(in_ir.back_left<IR_SHORT_LIMIT){
		//				out_leftpoint.x = in_pose_ref.x - pose.; //EDIT
							out_leftpoint.x = in_pose.twist.linear.x; //EDIT
						out_leftpoint.y = in_pose.twist.linear.y -in_ir.back_left/100 + LBXOFF/100.0;
	//				out_leftpoint.y = in_pose.twist.linear.y + in_ir.back_left/100 + LBXOFF/100.0;
							out_leftpoint.z=0;
						}
						if(in_ir.back_right<IR_SHORT_LIMIT){
							out_rightpoint.x = in_pose.twist.linear.x;
	//						out_rightpoint.y = in_pose.twist.linear.y - in_ir.back_right/100 + RBYOFF/100.0; //EDIT
							out_rightpoint.y = in_pose.twist.linear.y + in_ir.back_right/100 + RBYOFF/100.0; //EDIT
							out_rightpoint.z=0;
						}					
					}
//			}			
			leftpoint_publisher_.publish(out_leftpoint);
			rightpoint_publisher_.publish(out_rightpoint);
			break;
		}
	}
};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "maze_mapping");
  maze_mapping mmnode;
	ros::Rate loop_rate(10);
	
	for (int i = 0; i < 20; ++i) {
		loop_rate.sleep();
	}

	while (ros::ok())
	{
	//	if (mmnode.hasIR) {
	   mmnode.publish();
	//	}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

