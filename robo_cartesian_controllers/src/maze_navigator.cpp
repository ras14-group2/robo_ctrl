#include "ros/ros.h"
#include <ras_arduino_msgs/Encoders.h>
#include <ir_reader/distance_readings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <robo_globals.h>
#include <mapper/WallInFront.h>
#include <mapper/PathToObject.h>
#include <mapper/PathToUnknown.h>
#include <object_finder/Positions.h>
#include <cmath>
#include <list>
// basic file operations
#include <iostream>
#include <cstdio>
//#include <fstream>

#include <robo_cartesian_controllers/Node.h>


#define MINDIST 12	//12 centimeters
#define STOPDIST 17	//17 centimeters

#define IR_SHORT_LIMIT 25

#define NODE_DIST_LIMIT 0.1

//Controller modes
#define STILL 0
#define LEFT_WALL_FOLLOW 1
#define RIGHT_WALL_FOLLOW 2
#define LEFT_ROTATE 3
#define RIGHT_ROTATE 4
#define STRAIGHT_FORWARD 5
#define RIGHT_WALL_ALIGN 6
#define LEFT_WALL_ALIGN 7
#define CONFUSED 8
#define ROBOLENGTH .22


static const int MAXINT = (((1 << 30)-1) << 1)+1;	//2^31-1

static const char* MODE_NAMES[9] = {"STILL", "LEFT_WALL_FOLLOW", "RIGHT_WALL_FOLLOW",
"LEFT_ROTATE", "RIGHT_ROTATE", "STRAIGHT_FORWARD", "RIGHT_WALL_ALIGN", "LEFT_WALL_ALIGN", "CONFUSED"};


class maze_navigator_node
{
private:

	int mode;
	int prevmode; // Rohit: to know the previous mode
	bool followsPath;	//Tells whether the robot is following a path (true), or exploring (false).
	geometry_msgs::Point targetPos;	//The current target position, if following a path.
	std::list<geometry_msgs::Point> path;	//The path to the currently wanted object
	std::string goalObject;	//The object currently wanted
	
	double alpha, alpha1; // P gain {left, right}	//DON'T CHANGE THIS DURING RUNTIME
	double alpha_align;
	double angle;	//angular orientation
	double targetAngle;	//The angle to aim for when rotating
	double v; // linear velocity constant
	double w; // angular_vel
	double delta_enc[2];	//Read delta encoder values
	double irdiff; //Difference between IR_sensors
	double irav; //IR_sensors average
	ros::ServiceClient client; //client for map service
	ros::ServiceClient pathClient;
	ros::Subscriber pathToUnknownSub;
	int updateCounter;

	geometry_msgs::Twist out_twist;
	geometry_msgs::Twist curPosOri;	//Current position and orientation according to the odometry
	ir_reader::distance_readings in_ir;
	//std_msgs::String out_mode;
	std_msgs::Int16 out_mode;
	std_msgs::Int16 out_previous_mode;
	geometry_msgs::Point p1_left,p2_left, p1_right, p2_right;

public:

	bool hasIR;
	bool leftBool1, leftBool2, rightBool1, rightBool2;

	ros::NodeHandle n_;
	ros::Subscriber ir_reader_subscriber_, imu_subscriber_, encoders_subscriber_, odometry_subscriber_, obj_rec_;
	ros::Publisher twist_publisher_, mode_publisher_, prev_mode_publisher_, node_creation_publisher_;
	//alpha(0.0177) for wall-following without reference distance
	//alpha(0.0245), alpha1(2.53586),

	//Advertize a node creation request
	void advertize_node(double x, double y) const {
		robo_cartesian_controllers::Node n;
		n.x.data = x;
		n.y.data = y;
		n.west.data = true;
		n.south.data = true;
		n.east.data = true;
		n.north.data = true;
		node_creation_publisher_.publish(n);
	}


	maze_navigator_node() : alpha(0.0175), alpha1(0.01), alpha_align(0.0195), v(.17), w(0), mode(STRAIGHT_FORWARD), prevmode(STRAIGHT_FORWARD), goalObject("red cube")
	{
		
		//pathClient = n_.serviceClient<mapper::PathToObject>("path_to_object");

		followsPath = false;
		hasIR = false;
		leftBool1 = false;
		rightBool1 = false;
		leftBool2 = false;
		rightBool2 = false;
		n_ = ros::NodeHandle("~");
		ir_reader_subscriber_ = n_.subscribe("/ir_reader_node/cdistance", 1, &maze_navigator_node::irCallback, this);
		encoders_subscriber_ = n_.subscribe("/arduino/encoders", 1, &maze_navigator_node::encodersCallback, this);
		imu_subscriber_ = n_.subscribe("/imu_angle", 1, &maze_navigator_node::imuAngleCallback, this);
		odometry_subscriber_ = n_.subscribe("/posori/Twist", 1, &maze_navigator_node::odometryCallback, this);
		pathToUnknownSub = n_.subscribe("/mapper/pathToUnknown", 1, &maze_navigator_node::pathToUnknownCallback, this);
		obj_rec_ = n_.subscribe("/object_identifier/positions_in", 1, &maze_navigator_node::objRecCallback, this);
		twist_publisher_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
		//Rohit: publish mode
		//mode_publisher_ = n_.advertise<std_msgs::String>("/maze_navigator/mode", 1000);
		mode_publisher_ = n_.advertise<std_msgs::Int16>("/maze_navigator/mode", 1000);
		prev_mode_publisher_ = n_.advertise<std_msgs::Int16>("/maze_navigator/prevmode", 1000);
		node_creation_publisher_ = n_.advertise<robo_cartesian_controllers::Node>("/node_creation", 100);

		//service client
		client = n_.serviceClient<mapper::WallInFront>("/wall_in_front");
		updateCounter = 0;
	}
	
	void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
	{
		delta_enc[0] = msg->delta_encoder1;
		delta_enc[1] = msg->delta_encoder2;
	}

	void irCallback(const ir_reader::distance_readings::ConstPtr &msg)
	{
		in_ir = *msg;
		hasIR = true;
	}

	void imuAngleCallback(const std_msgs::Float64::ConstPtr &msg) {
		//angle = msg->data;
	}

	void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
		curPosOri = msg->twist;
		angle = curPosOri.angular.z;
	}

	void objRecCallback(const object_finder::Positions::ConstPtr &msg) {
		ROS_INFO("GOING TO SLEEP FOR 3 SECONDS");
		ros::Duration(3).sleep(); // sleep for 3 second
	}


	void pathToUnknownCallback(const mapper::PathToUnknown::ConstPtr &msg){
        ROS_INFO("received path to unknown part of the maze");
		followsPath = true;
		path.clear();
		for(size_t i = 0; i < msg->points.size(); i++){
			path.push_back(msg->points[i]);
            ROS_INFO("(%f, %f)", msg->points[i].x, msg->points[i].y);
        }
        targetPos.x = curPosOri.linear.x;
        targetPos.y = curPosOri.linear.y;
        return;
	}
	
	//From still, decide what the next mode should be
	int decideNextMode() {

        out_twist.linear.x = 0.0;
        out_twist.angular.z = 0.0;
		
		if (followsPath) {
            ROS_INFO("following path");
			
			//Check if we have arrived
			if (fabs(curPosOri.linear.x - targetPos.x) <= NODE_DIST_LIMIT && 
			fabs(curPosOri.linear.y - targetPos.y) <= NODE_DIST_LIMIT) {
				//Close enough
                ROS_INFO("reached node (%f, %f)", targetPos.x, targetPos.y);

                if(path.empty()){
                    ROS_INFO("reached end of path - go to STRAIGHT_FORWARD");
                    mode = STRAIGHT_FORWARD;
                    prevmode = STILL;
                    followsPath = false;
                    return mode;
                }

				//Compute direction to next node
				geometry_msgs::Point next = path.front();
				ROS_INFO("target position: (%f, %f)", next.x, next.y);
				ROS_INFO("current position: (%f, %f)", curPosOri.linear.x, curPosOri.linear.y);

                while(fabs(curPosOri.linear.x - next.x) <= NODE_DIST_LIMIT &&
                      fabs(curPosOri.linear.y - next.y) <= NODE_DIST_LIMIT){
                    //possible dirty bugfix, check if next node is already reached
                    if(path.empty()){
                        ROS_INFO("reached end of path - go to STRAIGHT_FORWARD");
                        mode = STRAIGHT_FORWARD;
                        prevmode = STILL;
                        followsPath = false;
                        return mode;
                    }
                    ROS_INFO("removing close points from path");
                    path.pop_front();
                    next = path.front();

                    ROS_INFO("target position: (%f, %f)", next.x, next.y);
                    ROS_INFO("current position: (%f, %f)", curPosOri.linear.x, curPosOri.linear.y);
                }

				int dir;
                ROS_INFO("x diff: %f", fabs(curPosOri.linear.x - next.x));
                ROS_INFO("y diff: %f", fabs(curPosOri.linear.y - next.y));
				if (fabs(curPosOri.linear.x - next.x) > fabs(curPosOri.linear.y - next.y)) {
					if (curPosOri.linear.x > next.x) {
						dir = 0;	//west
					} else {
						dir = 2;	//east
					}
				} else {
					if (curPosOri.linear.y > next.y) {
						dir = 1;	//south
					} else {
						dir = 3;	//north
					}
				}
				int curDir = (int)(floor(angle/(PI/2.0)+0.5));	//Find the closest NSWE orientation
				if (curDir >= 0) {
					curDir = curDir % 4;
				} else {
					curDir = (curDir + 400000000) % 4;
				}

				ROS_INFO("current direction: %d, target direction: %d", curDir, dir);
				
				if (dir == curDir) {	//No turn required
					targetPos = path.front();
					path.pop_front();
                    mode = STRAIGHT_FORWARD;
//					if (path.empty()) {
//						//We are facing the final node, which should hold the object.
//						//We shouldn't go there - just observe from where we are standing.
//						//TODO Decide what to do next. I don't know if we will be given a new object
//						//to find the path to, or if something else should be done.
						
//						mode = STRAIGHT_FORWARD;
						
//					} else {
//						mode = STRAIGHT_FORWARD;
//					}
				} else if (dir == (curDir+1)%4) {	//Rotate left
					mode = LEFT_ROTATE;
					prevmode = STILL;
                    targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 + PI/2.0;
				} else if ((dir+1)%4 == curDir) {	//Rotate right
					mode = RIGHT_ROTATE;
					prevmode = STILL;
                    targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 - PI/2.0;
				} else {	//Rotate left, continuing 180 degrees.
					mode = LEFT_ROTATE;
					prevmode = STILL;
                    targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 + PI/2.0;
				}
                return mode;
			}			
		}
		
		out_twist.linear.x = 0.0;
		out_twist.angular.z = 0.0;			
		if (delta_enc[0] != 0 || delta_enc[1] != 0) {
			//Do nothing
			ROS_INFO("still in still");
			return STILL;
		}
		
		//Rohit: check the previous mode and accordingly decide the next mode	 
		if (prevmode==RIGHT_WALL_ALIGN){ 	
			prevmode=mode;
			rightBool1 = true;
			leftBool1 = true; 
			mode = RIGHT_WALL_FOLLOW;
				
		}
		
		else if (prevmode==LEFT_WALL_ALIGN){	
			prevmode=mode;
			leftBool1 = true;
			rightBool1 = true;
			mode = LEFT_WALL_FOLLOW;
									
		}

		else if(prevmode==LEFT_ROTATE){
			prevmode=mode;
			mode = RIGHT_WALL_ALIGN;
		}

		else if(prevmode==RIGHT_ROTATE){
			prevmode=mode;
			mode = LEFT_WALL_ALIGN;
		}
		
		else if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
			prevmode=mode;
			mode = LEFT_ROTATE;
			targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 + PI/2.0;
			//ROS_INFO("target Angle = %lf", (targetAngle*360)/TWOPI);
			//ROS_INFO("Left rotate:First if");
		} else if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
			prevmode=mode;
			mode = RIGHT_ROTATE;
			targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 - PI/2.0;
			//ROS_INFO("target Angle = %lf", (targetAngle*360)/TWOPI);
		} else {
			//TODO go back
			prevmode=mode;
			mode = LEFT_ROTATE;
			targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 + PI/2.0;
			//ROS_INFO("target Angle = %lf", (targetAngle*360)/TWOPI);
			//ROS_INFO("Left rotate:Second if");
		}

		return mode;
	}

	
	void publish()
	{
		//ROS_INFO("The current angle is %lf", (angle*180)/M_PI);	//degrees
		//ROS_INFO("The current angle is %lf", angle);	//radians
		//ROS_INFO("The target angle is %lf", targetAngle);	//radians
		//ROS_INFO("IR front_left: [%lf]", in_ir.front_left);
		//ROS_INFO("IR back_left: [%lf]", in_ir.back_left);
		//ROS_INFO("IR front_right: [%lf]", in_ir.front_right);
		//ROS_INFO("IR back_right: [%lf]", in_ir.back_right);
		//ROS_INFO("IR front_center: [%lf]", in_ir.front_center);

		//get wall in front from map
		bool wallInFront = false;
		//divide frequency

	//	ROS_INFO("get info from map? (counter: %d)", updateCounter);
		if(updateCounter >= 5){
	//	ROS_INFO("try to get info from map");
			updateCounter = 0;
			mapper::WallInFront srv;
			srv.request.position.x = curPosOri.linear.x;
			srv.request.position.y = curPosOri.linear.y;
			srv.request.angle = angle;
			if(client.call(srv)){
				ros::Time now = ros::Time::now();
				//ROS_INFO("wall_in_front request at %d.%d", now.sec, now.nsec);
				wallInFront = srv.response.wallInFront;
				if(wallInFront){
					//ROS_INFO("WALL IN FRONT");
				}
				else{
					//ROS_INFO("everything free");
				}
			}
		}
		else{
			updateCounter++;
		}
		switch (mode) {
					
            case STILL:
				//Rohit: When in still, decide upon the next mode based on the previous mode
				decideNextMode();
				break;


			case LEFT_WALL_FOLLOW:
            ROS_INFO("case left wall follow");
				if (in_ir.front_left>25){
					if (!followsPath) {
						if(leftBool1 == true){
							p1_left.x = curPosOri.linear.x;
							p1_left.y = curPosOri.linear.y;
							leftBool1 = false;
							leftBool2 = true;
							ROS_INFO("In p1 left condition");
						}
					} else {
						//TODO Check if have arrived at the targetPoint
					}
				}
					
				if (in_ir.front_right>25){
					if (!followsPath) {
						if(rightBool1 == true){
							p1_right.x = curPosOri.linear.x;
							p1_right.y = curPosOri.linear.y;
							rightBool1 = false;
							rightBool2 = true;
							ROS_INFO("In p1 left condition");
						}
					
						if(rightBool2 == true){
							if((sqrt(pow((curPosOri.linear.x - p1_right.x),2) + pow((curPosOri.linear.y - p1_right.y),2))) > ROBOLENGTH){
								advertize_node(curPosOri.linear.x, curPosOri.linear.y);
								rightBool1 = true;
								rightBool2 = false;
							}
						}
			
					} else {
						//TODO Check if have arrived at the targetPoint
					}	
				}
					
//				if ((in_ir.front_right>25) && (in_ir.back_right>25)){
//				if (rightBool == true){							
//					//Advertise node creation request for right open spaces before still
//					if (!followsPath) {
//						geometry_msgs::Point p;
//						p.x = curPosOri.linear.x;
//						p.y = curPosOri.linear.y;
//						node_creation_publisher_.publish(p);
//						rightBool = false;
//					} else {
//						//TODO Check if have arrived at the targetPoint
//					}
//				}
//				}
				
//				ROS_ERROR("in_ir.front_center is %lf and STOPDIST is %d", in_ir.front_center, STOPDIST);
//                ROS_INFO("wall in front: %s", wallInFront ? "true" : "false");
				
				if (in_ir.front_center<STOPDIST || wallInFront) {
	//					if (in_ir.front_center<STOPDIST) {
					prevmode=mode;
					mode = STILL;
	//				leftBool = true;
	//				rightBool= true;
					//Stop and determine how to rotate
	//				ROS_INFO("IR front_center inside switch : [%lf]", in_ir.front_center);
					
					//Advertise node creation request
					if (!followsPath) {
						advertize_node(curPosOri.linear.x, curPosOri.linear.y);
					} else {
						//TODO Check if have arrived at the targetPoint
					}
					
					break;
				}

				if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
//					leftBool = true;
//					rightBool= true;
					break;
				}
				
				//prevmode == LEFT_WALL_FOLLOW;
				
				irav = 0.5*(in_ir.front_left + in_ir.back_left);
				irdiff = in_ir.front_left - in_ir.back_left;
				
				out_twist.angular.z = alpha * (irdiff);// [m/s]
				if(fabs(MINDIST-irav) > 0.5)	
					out_twist.angular.z += alpha1 * (irav - MINDIST);
					
				//out_twist.angular.z = alpha * (irav - MINDIST + (alpha1 * irdiff)); // [m/s]
				out_twist.linear.x = v;
				
				//w = alpha * (MINDIST-0.5(in_ir.front_left+in_ir.back_left) + 2*(ir[0]-ir[1]));
				break;


			case RIGHT_WALL_FOLLOW:
				if (in_ir.front_right>25){
					if (!followsPath) {
						if(rightBool1 == true){
							p1_right.x = curPosOri.linear.x;
							p1_right.y = curPosOri.linear.y;
							rightBool1 = false;
							rightBool2 = true;
						}
					} else {
						//TODO Check if have arrived at the targetPoint
					}
				}
				if (in_ir.front_left>25){
					if (!followsPath) {
						if(leftBool1 == true){
							p1_left.x = curPosOri.linear.x;
							p1_left.y = curPosOri.linear.y;
							leftBool1 = false;
							leftBool2 = true;
							ROS_INFO("In p1 left condition");
						}
					
						if(leftBool2 == true){
							if((sqrt(pow((curPosOri.linear.x - p1_left.x),2) + pow((curPosOri.linear.y - p1_left.y),2))) > ROBOLENGTH){
								advertize_node(curPosOri.linear.x, curPosOri.linear.y);
								leftBool1 = true;
								leftBool2 = false;
							}
						}
					} else {
						//TODO Check if have arrived at the targetPoint
					}
				}

				if (in_ir.front_center<STOPDIST || wallInFront) {
				//if (in_ir.front_center<STOPDIST ) {
					prevmode=mode;
					mode = STILL;
					//leftBool = true;
					//rightBool= true;
					//ROS_INFO("IR front_center inside switch : [%lf]", in_ir.front_center);
					
					//Advertise node creation request
					if (!followsPath) {
						advertize_node(curPosOri.linear.x, curPosOri.linear.y);
					} else {
						//TODO Check if have arrived at the targetPoint
					}
					
					break;
				}
				if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					//leftBool = true;
					//rightBool= true;
					break;
				}		
			  
			  
				irav = 0.5*(in_ir.front_right + in_ir.back_right);
				irdiff = in_ir.front_right - in_ir.back_right;
				
				out_twist.angular.z = - alpha * (irdiff);// [m/s]
				if(fabs(MINDIST-irav) > 0.5)		
					out_twist.angular.z += alpha1 * (-irav + MINDIST);
				//out_twist.angular.z = alpha * ( MINDIST - (irav)  - (alpha1 * irdiff)); // [m/s]
				out_twist.linear.x = v;
				//w = alpha * (MINDIST-0.5(in_ir.front_right+in_ir.back_right) + 2*(ir[2]-ir[3]));
				break;


			case LEFT_ROTATE:
				//ROS_INFO("angle & targetAngle are %lf & %lf.", angle, targetAngle);
				//if (fabs(angle - refAngle) >=(angcomp+M_PI_2-angle)){
				if (angle >= (targetAngle-0.2)) {
					//ROS_INFO("Difference rotation = %lf", (1/M_PI)*(angcomp+M_PI_2)*180 );
					//Align to the right wall
					prevmode = mode;
					mode = STILL;
				}	
				out_twist.angular.z = .9;	
	//			out_twist.angular.z = ((M_PI_2-fabs(angle-refAngle))*1.5) + 0.6;	
				out_twist.linear.x = 0.0;
				break;


			case RIGHT_ROTATE:
				//ROS_INFO("angle & targetAngle are %lf & %lf.", angle, targetAngle);
				//if (fabs(angle - refAngle) >=(angcomp+M_PI_2-angle)){
				//if (fabs(angle-targetAngle) < 0.1) {
				if (angle <= (targetAngle+0.2)) {
					//ROS_INFO("Difference rotation = %lf", (1/M_PI)*(angcomp+M_PI_2)*180 );
					//Align to the left wall
					prevmode = mode;
					mode = STILL;
				}
//					out_twist.angular.z = -((M_PI_2-fabs(angle-refAngle))*1.5) - 0.6;
				out_twist.angular.z = -.9;//-(M_PI_2-fabs(angle-refAngle))*.8;
				out_twist.linear.x = 0.0;
				break;


			case STRAIGHT_FORWARD:
				//TODO
				if (in_ir.front_right>25){
					if (prevmode == RIGHT_WALL_FOLLOW){
						if(rightBool2 == true){
							if((sqrt(pow((curPosOri.linear.x - p1_right.x),2) + pow((curPosOri.linear.y - p1_right.y),2))) > ROBOLENGTH){
								p2_right.x = curPosOri.linear.x;
								p2_right.y = curPosOri.linear.y;
								advertize_node(p2_right.x, p2_right.y);
								rightBool1 = true;
								rightBool2 = false;
							}
						}
					}
				}
				
				if (in_ir.front_left>25){
					if (prevmode == LEFT_WALL_FOLLOW){
						if(leftBool2 == true){
							if((sqrt(pow((curPosOri.linear.x - p1_left.x),2) + pow((curPosOri.linear.y - p1_left.y),2))) > ROBOLENGTH){
								p2_left.x = curPosOri.linear.x;
								p2_left.y = curPosOri.linear.y;
								advertize_node(p2_left.x, p2_left.y);
								leftBool1 = true;
								leftBool2 = false;
							}
						}
					}
				}
				
				if (in_ir.front_center < STOPDIST || wallInFront) {
				//if (in_ir.front_center < STOPDIST) {
					prevmode=mode;
					mode = STILL;	//Stop
					
					//Advertise node creation request
					if (!followsPath) {
						advertize_node(curPosOri.linear.x, curPosOri.linear.y);
					} else {
						//TODO Check if have arrived at the targetPoint
					}
					
				} else if (in_ir.front_left < IR_SHORT_LIMIT && in_ir.back_left < IR_SHORT_LIMIT) {
					prevmode=mode;
					leftBool1 = true;
					rightBool1 = true;
					mode = LEFT_WALL_FOLLOW;	
				} else if (in_ir.front_right < IR_SHORT_LIMIT && in_ir.back_right < IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = RIGHT_WALL_FOLLOW;
					rightBool1 = true;
					leftBool1 = true;	
				} else {
					//Keep Straight
					out_twist.linear.x = .15;
					out_twist.angular.z = 0.0;
				}
				break;

				
			case RIGHT_WALL_ALIGN:
				if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}	
				if (fabs(in_ir.front_right - in_ir.back_right) > 2) {
					out_twist.angular.z = -alpha_align*3* (in_ir.front_right - in_ir.back_right);
				} else {
						prevmode = mode;
						mode = STILL;
				}
				break;

				
			case LEFT_WALL_ALIGN:
				if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}
				if (fabs(in_ir.front_left - in_ir.back_left) > 2) {
					out_twist.angular.z = alpha_align*3* (in_ir.front_left - in_ir.back_left);
				} else {
						prevmode = mode;
						mode = STILL;
				}	
				break;
		}
		
		if ((mode == LEFT_WALL_FOLLOW || mode == RIGHT_WALL_FOLLOW || mode == STRAIGHT_FORWARD) && followsPath) {
            ROS_INFO("check if arrived at next node");
			if (fabs(curPosOri.linear.x - targetPos.x) <= NODE_DIST_LIMIT && 
            fabs(curPosOri.linear.y - targetPos.y) <= NODE_DIST_LIMIT) {
				//We have arrived
                ROS_INFO("arrived at node - go to STILL mode");
				mode = STILL;
			}
            else{
                //node not reached - check if we are still on track

                //check robot rotation
                double cosRot = cos(curPosOri.angular.z);
                double cosBorder = cos(M_PI_4);
                double sinRot = sin(curPosOri.angular.z);
                double sinBorder = sin(M_PI_4);

                if(cosRot >= cosBorder || cosRot <= -cosBorder){
                    //moving along x-axis
                    if(fabs(curPosOri.linear.x - targetPos.x) <= NODE_DIST_LIMIT){
                        //we are off
                        ROS_ERROR("off path - try to recover");

                        //create new node in front of us
                        geometry_msgs::Point newNode;
                        newNode.x = targetPos.x;
                        newNode.y = curPosOri.linear.y;
                        ROS_INFO("adding turning point at (%f, %f)", newNode.x, newNode.y);
                        path.push_front(newNode);
                        mode = STILL;
                    }

                }
                else{
                    //moving along y-axis
                    if(fabs(curPosOri.linear.y - targetPos.y) <= NODE_DIST_LIMIT){
                        //we are off
                        ROS_ERROR("off path - try to recover");

                        //create new node in front of us
                        geometry_msgs::Point newNode;
                        newNode.x = curPosOri.linear.x;
                        newNode.y = targetPos.y;
                        ROS_INFO("adding turning point at (%f, %f)", newNode.x, newNode.y);
                        path.push_front(newNode);
                        mode = STILL;
                    }
                }
            }
		}
		
		out_mode.data=mode;
		out_previous_mode.data=prevmode;
		twist_publisher_.publish(out_twist);
		mode_publisher_.publish(out_mode);
		prev_mode_publisher_.publish(out_previous_mode);
		//ROS_INFO("the linear twist is = %lf", out_twist.linear.x);
		//ROS_INFO("the twist is = %lf", out_twist.angular.z);
		ROS_ERROR("The current mode is %s, angle is %lf", MODE_NAMES[mode], angle);
		ROS_ERROR("The previous mode is %s", MODE_NAMES[prevmode]);
	}
};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "maze_navigator");
	maze_navigator_node mnnode;
	ros::Rate loop_rate(CTRL_FREQ*5);
	
	//for (int i = 0; i < 20; ++i) {
		//loop_rate.sleep();
	//}

	while (ros::ok())
	{
		if (mnnode.hasIR) {
			mnnode.publish();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	if(ros::isShuttingDown()){
		//
	}
	return 0;
}
