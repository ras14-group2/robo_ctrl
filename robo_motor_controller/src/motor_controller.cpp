/*

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <cmath>


#define CF 10.0
#define TPR 360.0

static const double PI = acos(-1.0);


class MotorControllerNode {

private:
	
	const double wheel_radius;	//Radius of the vehicle wheels
	const double base;	//Vehicle base
	const double alpha1;	//Adaptation speed coefficient LW
	const double alpha2;	//Adaptation speed coefficient RW
	double desLinVel;	//Desired linear velocity
	double desAngVel;	//Desired angular velocity
	double desLWVel;	//Desired left wheel angular velocity
	double desRWVel;	//Desired right wheel angular velocity
	double pwm1;	//Current pwm1 signal strength
	double pwm2;	//Current pwm2 signal strength
	

public:

	ros::NodeHandle n_;
	ros::Subscriber twist_subscriber_;
	ros::Subscriber encoders_subscriber_;
	ros::Publisher pwm_publisher_;
	

    MotorControllerNode() : wheel_radius(0.0352), base(0.23), alpha1(8.5), alpha2(10) {
         std::cerr << "Does it work?" << std::endl;
    }

	~MotorControllerNode() {}

	void init() {
		desLinVel = 0.0;
		desAngVel = 0.0;
		twist_subscriber_ = n_.subscribe("/motor_controller/twist", 1, &MotorControllerNode::updateDesiredVelocity, this);
        encoders_subscriber_ = n_.subscribe("/arduino/encoders", 1, &MotorControllerNode::adjustPWM, this);
        pwm_publisher_ = n_.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1);

        ras_arduino_msgs::PWM pm;
        pm.PWM1 = 0;
        pm.PWM2 = 0;
        pwm_publisher_.publish(pm);
	}
	
	void updateDesiredVelocity(const geometry_msgs::Twist::ConstPtr &msg) {
		
		desLinVel = msg->linear.x;
		desAngVel = msg->angular.z;
		
		desLWVel = (desLinVel-(base/2.0)*desAngVel)/wheel_radius;
		desRWVel = (desLinVel+(base/2.0)*desAngVel)/wheel_radius;

        std::cerr << "upd" << std::endl;
		
	}
	
	void adjustPWM(const ras_arduino_msgs::Encoders::ConstPtr &msg) {
		
        std::cerr << "adj" << std::endl;

		double estLWVel = (msg->delta_encoder1*CF*2.0*PI)/TPR;
		double estRWVel = (msg->delta_encoder2*CF*2.0*PI)/TPR;
		
		pwm1 += alpha1*(desLWVel-estLWVel);
		pwm2 += alpha2*(desRWVel-estRWVel);
		
        std::cerr << "difference for left wheel is " << desLWVel << "-" << estLWVel << std::endl;
        std::cerr << "difference for right wheel is " << desRWVel << "-" << estRWVel << std::endl;
		
		ras_arduino_msgs::PWM pm;
		pm.PWM1 = (int)pwm1;
		pm.PWM2 = (int)pwm2;
		pwm_publisher_.publish(pm);
		
		//std::cerr << "pm is:" << pm << std::endl;
		
	}

};


int main(int argc, char **argv) {
    //std::cerr << "Does it work   ?" << std::endl;
	ros::init(argc, argv, "motor_controller_node");

	MotorControllerNode motor_controller_node;

	motor_controller_node.init();

	ros::Rate loop_rate(CF);



	while(motor_controller_node.n_.ok())
	{
		ros::spinOnce();
        std::cerr << "5" << std::endl;
		loop_rate.sleep();
	}

	return 0;
}

*/




























#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>


class motor_controller
{
public:

    ros::NodeHandle n_;
    ros::Subscriber twist_subscriber_;
    ros::Subscriber encoders_subscriber_;
    ros::Publisher pwm_publisher_;
    std::vector<float> twist_;
    std::vector<float> encoder_;
    const static float control_frequency = 10.0; // Control @ 10 Hz

    motor_controller()
    {

        n_ = ros::NodeHandle("~");

        twist_= std::vector<float>(2, 0);
        encoder_= std::vector<float>(5,0);

        twist_subscriber_ = n_.subscribe("/motor_controller/twist", 1, &motor_controller::twistCallback, this);
        encoders_subscriber_ = n_.subscribe("/arduino/encoders", 1, &motor_controller::encodersCallback, this);
        pwm_publisher_ = n_.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    }

    void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        twist_[0] = msg->linear.x;
        twist_[1] = msg->angular.z;
    }

    void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
    {
        encoder_[0] = msg->encoder1;
        encoder_[1] = msg->encoder2;
        encoder_[2] = msg->delta_encoder1;
        encoder_[3] = msg->delta_encoder2;
        encoder_[4] = msg->timestamp;
    }

    // wheel angular velocity
    std::vector<float> wheels_ang_vel (std::vector<float> twist_)
    {
     float b = 0.206; // separation of the two central wheels in [m]
     float r = 0.097/2; //  wheel radius in [m]
     std::vector<float> wlr_;
     wlr_ = std::vector<float>(2,0);

     float v = twist_[0]; // robot linear velocity
     float w = twist_[1]; // robot angular velocity

     wlr_[0] = (v - (b/2) * w)/r; //left wheel angular velocity
     wlr_[1] = (v + (b/2) * w)/r; //right wheel angular velocity


/*
     ROS_INFO("Left Wheel: [%f]", wlr_[0]);// working
     ROS_INFO("Right angular: [%f]", wlr_[1]); //working
*/
     return wlr_;
    }

    // controller
    std::vector<int> controller (std::vector<float> wlr_, std::vector<int> actual_pwm)
    {

     std::vector<int> pwm_;
     pwm_ = std::vector<int>(2,0);

     std::vector<float> estimated_w_;
     estimated_w_ = std::vector<float>(2,0);

     float wl = wlr_[0]; // left wheel angular velocity
     float wr = wlr_[1]; // right wheel angular velocity
     float T = control_frequency; // Control @ 10 Hz
     int ticks = 360; //ticks_per_rev
     float pi = 3.14159265359;
     float alpha1 = 1.2; // P gain Left
     float alpha2 = 1.1; // P gain Right

     //motor_controller motor_controller_node;

     float delta_encoder1 = (float) motor_controller::encoder_[2];
     float delta_encoder2 = (float) motor_controller::encoder_[3];

     std::cerr << "delta_encoder1 = " << delta_encoder1 << std::endl;
     std::cerr << "delta_encoder2 = " << delta_encoder2 << std::endl;

     /*
       estimated_w = (delta_encoder*2*pi*control_frequency)/(ticks_per_rev)
       pwm = pwm + alpha*(desired_w - estimated_w)
     */

     estimated_w_[0] = (delta_encoder1*2*pi*T)/ticks;
     estimated_w_[1] = (delta_encoder2*2*pi*T)/ticks;

     std::cerr << "difference for left wheel is " << wl << "-" << estimated_w_[0] << std::endl;
     std::cerr << "difference for right wheel is " << wr << "-" << estimated_w_[1] << std::endl;

     std::cerr << "Left W: " << estimated_w_[0] << std::endl;
     std::cerr << "Right W: " << estimated_w_[1] << std::endl;
     std::cerr << "wl: " << wl << std::endl;
     std::cerr << "wr: " << wr << std::endl;

     pwm_[0] = (int) (actual_pwm[0] + alpha1 * (wl - estimated_w_[0])); //PWM1

    if (pwm_[0]>255)
     {
         pwm_[0] = 255;
     }

     if (pwm_[0]<-255)
     {
         pwm_[0] =-255;
     }

     pwm_[1] = (int) (actual_pwm[1] + alpha2 * (wr - estimated_w_[1])); //PWM2

     if (pwm_[1]>255)
     {
      pwm_[1] = 255;
     }

     if (pwm_[1]<-255)
     {
      pwm_[1] = -255;
     }

     return pwm_;
    }

private:
/*
    std::vector<float> twist_;
    std::vector<float> encoder_;
*/
};



int main(int argc, char **argv)
{

  std::vector<float> ww_;
  ww_ = std::vector<float>(2,0);

  std::vector<int> PWM_;
  PWM_ = std::vector<int>(2,0);
  PWM_[0] = 4; //reference (ref PWM_1)
  PWM_[1] = 4; //reference (ref PWM_2)

  ros::init(argc, argv, "motor_controller");

  motor_controller motor_controller_node;

  ros::Rate loop_rate(motor_controller::control_frequency);

  ras_arduino_msgs::PWM msg;



  while (ros::ok())
  {
    ww_ = motor_controller_node.wheels_ang_vel(motor_controller_node.twist_);

    std::cerr << "Left Wheel: " << ww_[0] << std::endl;
    std::cerr << "Right Wheel: " << ww_[1] << std::endl;

    PWM_ = motor_controller_node.controller(ww_,PWM_);

    std::cerr << "Left PWM: " << PWM_[0] << std::endl;
    std::cerr << "Right PWM: " << PWM_[1] << std::endl;

    msg.PWM1 = PWM_[0];
    msg.PWM2 = PWM_[1];

    motor_controller_node.pwm_publisher_.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
