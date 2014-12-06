#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ir_reader/distance_readings.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robo_globals.h>
#include <cmath>
#include <cstdio>
#include <geometry_msgs/Point.h>
#include <sstream>   // std::cout
#include <string>     // std::string, std::to_string


class navigation_plotter_node
{

private:
	
	geometry_msgs::Twist posori_msg;
	visualization_msgs::Marker marker_msg;
	geometry_msgs::Point in_rightpoint;
	geometry_msgs::Point in_leftpoint;
	int count;

	void visualizepoint(double x, double y) {
		std::string Result;          // string which will contain the result
		std::ostringstream convert;   // stream used for the conversion
		convert << count;      // insert the textual representation of 'Number' in the characters in the stream

		visualization_msgs::Marker points;
		Result=convert.str();
		points.ns = Result.c_str();
		points.header.frame_id = "/map";
    points.header.stamp =  ros::Time::now();
//	points.ns = Result;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.02;
    points.scale.y = 0.02;
    // Points are green
    points.color.b = 1.0f;
    points.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
		points.points.push_back(p);
		marker_publisher_.publish(points);
		count++;
		
	}

	
public:
	ros::Time last_time, current_time;
	ros::NodeHandle n_;
	ros::Subscriber posori_subscriber_, ir_subscriber_, leftpoint_subscriber_, rightpoint_subscriber_;
	ros::Publisher odometry_publisher_, marker_publisher_, ir_marker_publisher_;
	tf::TransformBroadcaster odom_broadcaster;
	bool shallPlotIR;
	
	void leftpointCallback(const geometry_msgs::Point::ConstPtr &msg)
	{
//		in_leftpoint = msg;
		ROS_INFO("am in leftpoint plotter");
		visualizepoint(msg->x, msg->y);
	}
	
	void rightpointCallback(const geometry_msgs::Point::ConstPtr &msg)
	{
//		in_rightpoint = msg;
		ROS_INFO("am in rightpoint plotter");
		visualizepoint(msg->x, msg->y);
	}
	
	navigation_plotter_node() : shallPlotIR(false)
	{
		n_ = ros::NodeHandle("~");
		posori_subscriber_ = n_.subscribe("/posori/Twist", 1, &navigation_plotter_node::plotOdom, this);
		ir_subscriber_ = n_.subscribe("/ir_reader/cdistance", 1, &navigation_plotter_node::plotIR, this);
		odometry_publisher_ = n_.advertise<nav_msgs::Odometry>("/odom", 50); //TODO
		marker_publisher_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
		ir_marker_publisher_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
		leftpoint_subscriber_ = n_.subscribe("/mapping/leftpoint", 1, &navigation_plotter_node::leftpointCallback, this);
		rightpoint_subscriber_ = n_.subscribe("/mapping/rightpoint", 1, &navigation_plotter_node::rightpointCallback, this);
	}
	
	~navigation_plotter_node() {}

	
	void construct_maze() {
	
		//ROS_INFO("Constructing maze...");
		
		visualization_msgs::Marker lines;
		lines.header.frame_id = "/map";
		lines.header.stamp = ros::Time::now();
		lines.ns = "maze lines";
		lines.action = visualization_msgs::Marker::ADD;
		lines.pose.orientation.w = 1.0;
		lines.id = 0;
		lines.type = visualization_msgs::Marker::LINE_LIST;

		// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		lines.scale.x = 0.02;

		// Lines are white
		lines.color.r = 1.0;
		lines.color.g = 1.0;
		lines.color.b = 1.0;
		lines.color.a = 1.0;
		
		const char* maze_lines = "0 0 0 2.4\n0 0 2.4 0\n0 2.4 2.4 2.4\n2.4 0 2.4 2.4\n0.36 2.15 1.167 2.15\n0.4 1.3 0.4 1.625\n0.8 1.625 0.8 1.3\n0.4 1.625 0.8 1.625\n0 0.4 2.025 0.4\n0.78 0.4 0.78 0.8\n1.6 0.4 1.6 2.025\n1.233 1.8 1.6 1.8\n1.25 1.4 1.25 0.96\n1.25 1.28 1.6 1.28\n1.6 1.32 2.0 1.32\n2.0 1.8 2.4 1.8\n2.0 0.81 2.4 0.81\n";
		geometry_msgs::Point tmp1, tmp2;
		int chread, offset = 0;
		while (sscanf(maze_lines+offset, "%lf%lf%lf%lf %n", &tmp1.x, &tmp1.y, &tmp2.x, &tmp2.y, &chread)) {
			lines.points.push_back(tmp1);
			lines.points.push_back(tmp2);
			offset += chread;
		}
		
		lines.lifetime = ros::Duration();
		
		//marker_publisher_.publish(lines);
	}
	
	void plotIR(const ir_reader::distance_readings::ConstPtr &msg) {
		
	}
	
	void plotOdom(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		current_time = ros::Time::now();
		
		//Read the current orientation and position
		double x, y, angle;
		x = msg->twist.linear.x;
		y = msg->twist.linear.y;
		angle = msg->twist.angular.z;
		
		ROS_INFO("x y and angle are: %lf %lf and %lf*pi", x, y, angle/PI);

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/map";
		odom_trans.child_frame_id = "/whyinthewholeworlddoineedthis";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "/map";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "/whyinthewholeworlddoineedthis";
		odom.twist.twist.linear.x = 1.0;
		odom.twist.twist.linear.y = 1.0;
		odom.twist.twist.angular.z = 1.0;

		//publish the message
		odometry_publisher_.publish(odom);

		last_time = current_time;
	}




};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_plotter_node");
	navigation_plotter_node npn;
	npn.last_time = ros::Time::now();
	ros::Rate loop_rate(CTRL_FREQ);
	
	//Print maze lines
	npn.construct_maze();
	
	while(npn.n_.ok())
	{
		ros::spinOnce();
		//npn.construct_maze();
		loop_rate.sleep();
		
	


		
	}
	return 0;
}





