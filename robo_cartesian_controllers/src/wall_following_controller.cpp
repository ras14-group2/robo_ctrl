#include "ros/ros.h"
#include <ir_reader/distance_readings.h>
#include <geometry_msgs/Twist.h>




class wall_following_controller_node
{
private:

	double alpha; // P gain {left, right}	//DON'T CHANGE THIS DURING RUNTIME
	geometry_msgs::Twist msg;

public:

	ros::NodeHandle n_;
	ros::Subscriber ir_reader_subscriber_;
	ros::Publisher twist_publisher_;
	double twist_[2]; // Linear and angular velocity topics for the robot
	double ir_[6]; // IR sensors
	double v; // linear velocity constant
	double w; // angular_vel


	wall_following_controller_node() : alpha(0.0275), v(0.15), w(0)
	{
		n_ = ros::NodeHandle("~");
		ir_reader_subscriber_ = n_.subscribe("/ir_reader_node/cdistance", 1, &wall_following_controller_node::irCallback, this);
		twist_publisher_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
	}

	void irCallback(const ir_reader::distance_readings::ConstPtr &msg)
	{
		ir_[0] = msg->front_left;
		ir_[1] = msg->back_left;
		ir_[2] = msg->front_right;
		ir_[3] = msg->back_right;
		ir_[4] = msg->front_center;
		ir_[5] = msg->back_center;

		ROS_INFO("IR front_left: [%lf]", ir_[0]);
		ROS_INFO("IR back_left: [%lf]", ir_[1]);
	}


	// Compute angular velocity
	void computeAngVel ()
	{
		w = alpha * (ir_[0] - ir_[1]); // angular_vel = alpha*( distance_sensor1 - distance_sensor2)
		ROS_INFO("w: [%lf]", w);

	}

	void publish()
	{
		computeAngVel();
		msg.linear.x = v; // [m/s]
		msg.angular.z = w; // [rad/s]
		twist_publisher_.publish(msg);
	}
};


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "wall_following_controller");
	wall_following_controller_node wfcnode;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
		wfcnode.publish();
    ros::spinOnce();
		loop_rate.sleep();
  }

  return 0;
}
