#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace Eigen;

class DynamicModel
{
public:
	float integral_step;
	float Tstbd;
	float Tport;

	float delta_x;
	float delta_y;

	//Hydrodynamic parameters
	float Xu;
	float Yv;
	float Yr;
	float Nv;
	float Nr;
	static constexpr float X_u_dot = -2.25;
	static constexpr float Y_v_dot = -23.13;
	static constexpr float Y_r_dot = -1.31;
	static constexpr float N_v_dot = -16.41;
	static constexpr float N_r_dot = -2.79;
	float Xuu;
	static constexpr float Yvv = -99.99;
	static constexpr float Yvr = -5.49;
	static constexpr float Yrv = -5.49;
	static constexpr float Yrr = -8.8;
	static constexpr float Nvv = -5.49;
	static constexpr float Nvr = -8.8;
	static constexpr float Nrv = -8.8;
	static constexpr float Nrr = -3.49;

	//Intrinsic parameters
	static constexpr float m = 30; //mass
	static constexpr float Iz = 4.1; //moment of inertia
	static constexpr float B = 0.41; //centerline-to-centerline separation
	static constexpr float c = 0.78; //thruster correction factor

	tf2::Quaternion myQuaternion;

	Vector3f upsilon;
	Vector3f upsilon_dot_last;
	Vector3f upsilon_dot;
	Vector3f eta;
	Vector3f eta_dot_last;
	Vector3f eta_dot;
	Matrix3f M;
	Vector3f T;
	Matrix3f CRB;
	Matrix3f CA;
	Matrix3f C;
	Matrix3f Dl;
	Matrix3f Dn;
	Matrix3f D;
	Matrix3f J;
	Vector3f Delta;

	float x;
	float y;
	float etheta;
	float u;
	float v;
	float r;

	geometry_msgs::Pose2D dm_pose; //inertial navigation system pose [North East Yaw] or [x y psi]
	geometry_msgs::Vector3 dm_vel; //velocity vector [u v r]
	nav_msgs::Odometry odom;

	DynamicModel()
	{
		//ROS Publishers for each required simulated ins_2d data
		dm_pos_pub = n.advertise<geometry_msgs::Pose2D>("/vectornav/ins_2d/ins_pose", 1000);
		local_pos_pub = n.advertise<geometry_msgs::Pose2D>("/vectornav/ins_2d/NED_pose", 1000);
		dm_vel_pub = n.advertise<geometry_msgs::Vector3>("/vectornav/ins_2d/local_vel", 1000);
		ardumotors_flag_pub = n.advertise<std_msgs::UInt8>("/arduino_br/ardumotors/flag",1000);
		arduino_flag_pub = n.advertise<std_msgs::UInt8>("arduino",1000);
		boat_odom_pub = n.advertise<nav_msgs::Odometry>("/usv_control/dynamic_model_simulate/odom", 1000);

		right_thruster_sub = n.subscribe("/usv_control/controller/right_thruster", 1000, &DynamicModel::right_callback, this);
		left_thruster_sub = n.subscribe("/usv_control/controller/left_thruster", 1000, &DynamicModel::left_callback, this);
		disturbance_sub = n.subscribe("/usv_disturbance", 1000, &DynamicModel::dist_callback, this);

		upsilon << 0, 0, 0;
		upsilon_dot_last << 0, 0, 0;
		eta << 0, 0, 0;
		eta_dot_last << 0, 0, 0;

		//constant matrix M
		M << m - X_u_dot, 0, 0,
			0, m - Y_v_dot, 0 - Y_r_dot,
			0, 0 - N_v_dot, Iz - N_r_dot;
		
		J << cos(eta(2)), -sin(eta(2)), 0,
			sin(eta(2)), cos(eta(2)), 0,
			0, 0, 1;

	}

	void right_callback(const std_msgs::Float64::ConstPtr& right)
	{
		Tstbd = right->data; //right thruster input in Newtons
	}

	void left_callback(const std_msgs::Float64::ConstPtr& left)
	{
		Tport = left->data; //left thruster input in Newtons
	}

	void dist_callback(const geometry_msgs::Pose2D::ConstPtr& delta)
	{
		delta_x = delta->x; //North disturbance in Newtons
		delta_y = delta->y; //East disturbance in Newtons
	}

	void time_step()
	{
		//Hydrodynamic equations and parameter conditions
		Xu = -25;
		Xuu = 0;
		if (abs(upsilon(0)) > 1.2){
			Xu = 64.55;
			Xuu = -70.92;
		}

		Yv = 0.5*(-40*1000*abs(upsilon(1)))*(1.1+0.0045*(1.01/0.09)-0.1*(0.27/0.09)+0.016*(pow((0.27/0.09),2)));
		Yr = 6*(-3.141592*1000)*sqrt(pow(upsilon(0),2)+pow(upsilon(1),2))*0.09*0.09*1.01;
		Nv = 0.06*(-3.141592*1000)*sqrt(pow(upsilon(0),2)+pow(upsilon(1),2))*0.09*0.09*1.01;
		Nr = 0.02*(-3.141592*1000)*sqrt(pow(upsilon(0),2)+pow(upsilon(1),2))*0.09*0.09*1.01*1.01;

		//Vector of NED disturbances
		Delta << delta_x, delta_y, 0;

		//Vector of body disturbances
		Delta = J.inverse()*Delta;

		//Vector tau of torques
		T << Tport + c*Tstbd, 0, 0.5*B*(Tport - c*Tstbd);

		//Coriolis matrix - rigid body
		CRB << 0, 0, 0 - m * upsilon(1),
			0, 0, m * upsilon(0),
			m * upsilon(1), 0 - m * upsilon(0), 0;

		//Coriolis matrix - added mass
		CA << 0, 0, 2 * ((Y_v_dot*upsilon(1)) + ((Y_r_dot + N_v_dot)/2) * upsilon(2)),
			0, 0, 0 - X_u_dot * m * upsilon(0),
			2*(((0 - Y_v_dot) * upsilon(1)) - ((Y_r_dot+N_v_dot)/2) * upsilon(2)), X_u_dot * m * upsilon(0), 0;

		//Coriolis matrix
		C = CRB + CA;

		//Drag matrix - linear
		Dl << 0-Xu, 0, 0,
			0, 0-Yv, 0-Yr,
			0, 0-Nv, 0-Nr;

		//Drag matrix - nonlinear
		Dn << Xuu * abs(upsilon(0)), 0, 0,
			0, Yvv * abs(upsilon(1)) + Yvr * abs(upsilon(2)), Yrv * abs(upsilon(1)) + Yrr * abs(upsilon(2)),
			0, Nvv * abs(upsilon(1)) + Nvr * abs(upsilon(2)), Nrv * abs(upsilon(1)) + Nrr * abs(upsilon(2));

		//Drag matrix
		D = Dl - Dn;

		ROS_INFO("Beginning dummy action test");
		ROS_INFO_STREAM(D);

		upsilon_dot =  M.inverse()*(T - (C * upsilon) - (D * upsilon) + Delta); //acceleration vector [u' v' r']
		upsilon = integral_step * (upsilon_dot + upsilon_dot_last)/2 + upsilon; //integral [u v r]
		upsilon_dot_last = upsilon_dot;

		//Transformation matrix
		J << cos(eta(2)), -sin(eta(2)), 0,
			sin(eta(2)), cos(eta(2)), 0,
			0, 0, 1;

		eta_dot = J*upsilon; //transformation into local reference frame [x' y' psi']
		eta = integral_step*(eta_dot+eta_dot_last)/2 + eta; //integral [x y psi]
		eta_dot_last = eta_dot;

		x = eta(0); //position in x
		y = eta(1); //position in y
		etheta = eta(2); //orientatio psi
		//Wrap to [-pi pi]
		if (abs(etheta) > 3.141592){
			etheta = (etheta/abs(etheta))*(abs(etheta)-2*3.141592);
			}
		dm_pose.x = x;
		dm_pose.y = y;
		dm_pose.theta = etheta;
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0;

		myQuaternion.setRPY(0,0,etheta);

		odom.pose.pose.orientation.x = myQuaternion[0];
		odom.pose.pose.orientation.y = myQuaternion[1];
		odom.pose.pose.orientation.z = myQuaternion[2];
		odom.pose.pose.orientation.w = myQuaternion[3];

		u = upsilon(0); //surge velocity
		v = upsilon(1); //sway velocity
		r = upsilon(2); //yaw rate
		dm_vel.x = u;
		dm_vel.y = v;
		dm_vel.z = r;
		odom.twist.twist.linear.x = u;
		odom.twist.twist.linear.y = v;
		odom.twist.twist.linear.z = 0;

		odom.twist.twist.angular.x = 0;
		odom.twist.twist.angular.y = 0;
		odom.twist.twist.angular.z = r;

		//Data publishing
		dm_pos_pub.publish(dm_pose);
		dm_vel_pub.publish(dm_vel);
		local_pos_pub.publish(dm_pose);
		boat_odom_pub.publish(odom);

		std_msgs::UInt8 flag;
		flag.data = 1;
		arduino_flag_pub.publish(flag);
		ardumotors_flag_pub.publish(flag);
	}

private:
	ros::NodeHandle n;

	ros::Publisher dm_pos_pub;
	ros::Publisher local_pos_pub;
	ros::Publisher dm_vel_pub;
	ros::Publisher ardumotors_flag_pub;
	ros::Publisher arduino_flag_pub;
	ros::Publisher boat_odom_pub;

	ros::Subscriber right_thruster_sub;
	ros::Subscriber left_thruster_sub;
	ros::Subscriber disturbance_sub;

};

//Main
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dynamic_model_simulate");
	DynamicModel dynamicModel;
	dynamicModel.integral_step = 0.01;
	int rate = 100;
	ros::Rate loop_rate(rate);

  while (ros::ok())
  {
	dynamicModel.time_step();
	ros::spinOnce();
	loop_rate.sleep();
  }

	return 0;
}
