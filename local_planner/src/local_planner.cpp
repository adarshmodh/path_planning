#include "ros/ros.h"	
#include "ros/message_operations.h"	
#include "ros/message.h"						
#include "geometry_msgs/Twist.h"			
#include "geometry_msgs/Vector3.h"			
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"	
#include "geometry_msgs/Transform.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "ros/callback_queue.h"


#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#define PI 3.14159265


#define max_robot_curvature 0.4            								// curvature=1/turning_radius*****
#define max_robot_velocity 1               								// robot=husky
#define road_width 11.0

#define robot_length 1
#define robot_width 0.7
#define robot_radius 0.61
#define max_deviation ((road_width/2)-robot_width)                    	// according to road_width and robot_width, the max deviation from the center line possible for the robot

#define cl_curve_scaling 4
#define cl_straight_scale 200	
#define cl_dt 0.01								  						// for t=0 to 1 in bezier curve we want 100 points per section (200=2bezier curve)
#define cl_way_pt_no 3                                       			// change this when changing no of waypoints
#define	cl_traj_pt ((cl_way_pt_no-2)*2/(cl_dt*cl_curve_scaling) + (cl_way_pt_no -1)*cl_straight_scale - 1)

#define vs1_var 10 
#define d1_var 30          												// change for number of trajectories

#define traj_time_div 200												// each traj divided in 200 small time scaled divisions	


#define set_loop_rate 50												// ros loop_rate
#define replanning_freq 2

////tuning 
#define safety_clearance 0.3 
#define traj_s1 10
#define traj_t1 10



using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace Eigen;
using namespace std;



///////////////////ros variables///////////////////
ros::Publisher velocity_pub;
ros::Publisher vis_pub;
sensor_msgs::Imu accn_msg;
nav_msgs::Odometry pose_msg;
nav_msgs::Odometry temp_pose_msg;
geometry_msgs::Twist velocity_msg;
sensor_msgs::LaserScan obs_msg;


/////////////////obstacle variables////////////////////
int obs_no;
float obstacle_local_distance[100], obstacle_local_angle[100],obstacle_radius[100];


/////////////traj_parameter global variables//////////////////////
double t0=0,t1, s0=0,s1,vs0=0,vs1,as0=0,as1, d0=0,d1,vd0=0,vd1,ad0=0,ad1;              ////starting states for every replanning stage   


double robot_current_x,robot_current_y,robot_current_yaw;      			 /////from odometry
double robot_current_s,robot_current_d;
	


////////////////center_line global variables/////////////////
Vector2d cl_way_pt[cl_way_pt_no]; 													/// center line waypoints		
Vector2d cl_way_traj[(int)cl_traj_pt];		/// center line trajectory (x,y)	
double theta_r[(int)(cl_traj_pt)], arc_length[(int)(cl_traj_pt)], kappa[(int)(cl_traj_pt)];



//////////////////final_optimum_traj global variables/////////////// 
int optimum_traj_no;
double global_velocity[(int)cl_traj_pt], global_omega[(int)cl_traj_pt], global_theta_x[(int)cl_traj_pt], global_x[(int)cl_traj_pt], global_y[(int)cl_traj_pt]; 
double exec_t_step;

int prev_cl_pt=0,cl_pt_indices=50;
bool stop_replanning=0, last_plan=0;
long int planning_instant=0;
double prev_global_theta_x=0;




class obstacle
{
	//// obs_center_s, obs_center_d, clearing_radius
	
	public:
		static int obs_counter;
		
		double transformed_x,transformed_y,center_s,center_d,clearing_radius,velocity,orientation,vs,vd;
		
		obstacle(){
			obs_counter++;
		}
}; 
int obstacle::obs_counter = 0;                                      ////////////////use (obs_no) not this
obstacle obs[100];

class sd_traj
{
	//// matrix st_mat,ds_mat,a,b,cs_mat,cd_mat, &  collision_status, cost, max_curvature_status,
	public:
		static int traj_counter;
		
		double t0, t1;  //s0,vs0,as0,  d0,vd0,ad0,  t1,  s1,vs1,as1,  d1,vd1,ad1;
		MatrixXd a_mat, b_mat, c_a_mat, c_b_mat  ;
		
		double cost,max_curvature_value; 
		bool max_curvature_status,collision_status;

		double st[200], ds[200];
		
		sd_traj() : a_mat(6,1), b_mat(6,1), c_a_mat(6,1), c_b_mat(6,1) {						///constructor
			traj_counter++;
		}              
		
};
int sd_traj::traj_counter = 0;
sd_traj traj[vs1_var*d1_var];									//////define objects for trajectories





void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	accn_msg.header = imu_msg->header; 												/////header.seq
	accn_msg.angular_velocity = imu_msg->angular_velocity;							/////angular_velocity.x
	accn_msg.linear_acceleration = imu_msg->linear_acceleration;						/////linear_acceleration.x	
	//ROS_INFO("imu_updated");
}


void odom_scan_callback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	tf::Transform baselink2odom, obs2odom, point2;

	/////////////////////////////////////////////////odom////////////////////////////////////////////////////////

	pose_msg.header = odom_msg->header;													/////header.seq
	pose_msg.pose.pose.position = odom_msg->pose.pose.position;							/////pose.pose.position.x
	pose_msg.pose.pose.orientation = odom_msg->pose.pose.orientation;					/////pose.pose.orientation.x   /////quaternion	
	pose_msg.twist.twist.linear = odom_msg->twist.twist.linear;							/////twist.twist.linear.x	
	pose_msg.twist.twist.angular = odom_msg->twist.twist.angular;						/////twist.twist.angular.x
	
	tf::poseMsgToTF(odom_msg->pose.pose, baselink2odom);
 	
 	// ROS_INFO("Seq: [%d]", odom_msg->header.seq);
 	// ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom_msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
 	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom_msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
 	// ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom_msg->twist.twist.linear.x,msg->twist.twist.angular.z);


	///////////////////////////////////////////scan//////////////////////////////////////////////////////////////
	
	
	obs_msg.header = scan_msg->header;
	obs_msg.angle_min = scan_msg->angle_min;
	obs_msg.angle_max = scan_msg->angle_max;
	obs_msg.angle_increment = scan_msg->angle_increment;
	obs_msg.range_min = scan_msg->range_min;
	obs_msg.range_max = scan_msg->range_max;	
	obs_msg.ranges = scan_msg->ranges;

	obs_no=-1;
	float i=obs_msg.angle_min,temp_distance=0,temp_angle=0,delta_angle=0,prev_angle=0;
	int j=0,prev_j=0,count=0;

	while(i<obs_msg.angle_max)
	{
		if((obs_msg.ranges[j] >= obs_msg.range_min) && (obs_msg.ranges[j] <= obs_msg.range_max))
		{
			
			if(abs(j-prev_j) > 90)
			{

				obs_no++;
				//ROS_INFO("obs_no = %d",obs_no);
				prev_j = j;

				if(obs_no>0)
				{
					//ROS_INFO("current-angle %lf",i);	
					obstacle_local_distance[obs_no-1] = temp_distance/count; 
					obstacle_local_angle[obs_no-1] = (-temp_angle)/count;
					obstacle_radius[obs_no-1] = (obstacle_local_distance[obs_no-1]*(delta_angle)/2);
					//ROS_INFO("obs(%d) local_distance = %lf local_angle = %lf radius = %lf", obs_no-1,obstacle_local_distance[obs_no-1], obstacle_local_angle[obs_no-1], obstacle_radius[obs_no-1]);

				}

				temp_distance=0;
				temp_angle=0;
				count=0;
				prev_angle = i;
				//ROS_INFO("initial angle %lf",delta_angle);
			}	

			temp_distance += obs_msg.ranges[j];
			temp_angle += i;
			count++;
			delta_angle = i-prev_angle;
		}		

		j++;
		i+=obs_msg.angle_increment;
	}

	obstacle_local_distance[obs_no] = temp_distance/count; 
	obstacle_local_angle[obs_no] = (-temp_angle)/count;
	obstacle_radius[obs_no] = (obstacle_local_distance[obs_no]*(delta_angle)/2);
	//ROS_INFO("obs(%d) local_distance = %lf local_angle = %lf radius = %lf", obs_no,obstacle_local_distance[obs_no], obstacle_local_angle[obs_no],obstacle_radius[obs_no]);
	//ROS_INFO("end");

	
	for(int i=0; i<=obs_no; i++)
	{
		tf::Vector3 V(obstacle_local_distance[i]*cos(obstacle_local_angle[i]), obstacle_local_distance[i]*sin(obstacle_local_angle[i]),0);
		point2.setOrigin(V);
		obs2odom= baselink2odom * point2;		
		
		obs[i].transformed_x = obs2odom.getOrigin().x(); 
		obs[i].transformed_y = obs2odom.getOrigin().y();

		///////////////get velocity and orientation for dynamic obstacle  //////////////instead of point_tf use pose_tf for getting obstacle orientation and position

		obs[i].velocity = 0;        ////////////convert velocity and orientation to vs and vd
		obs[i].orientation = 0;

		//obs[i].vs = obs[i].vd = 0
		// obs[i].vs = (sqrt(pow(pose_msg.twist.twist.linear.x,2) + pow(pose_msg.twist.twist.linear.y,2))) * cos(theta_r[prev_cl_pt] - obs[j].orientation);  
		// obs[i].vd = -(sqrt(pow(pose_msg.twist.twist.linear.x,2) + pow(pose_msg.twist.twist.linear.y,2))) * sin(theta_r[prev_cl_pt] - obs[j].orientation); 

	} 
	
}



void publish_vw(int x)
{	

	//cout<<x<<endl;
	//if((x < (sizeof(global_velocity)/sizeof(global_velocity[0]))) && (!stop_replanning))
	if(x<0)
	{
		velocity_msg.linear.x = (double)max_robot_velocity;
		velocity_msg.linear.y = 0;
		velocity_msg.linear.z = 0;
				
		velocity_msg.angular.x = 0;
		velocity_msg.angular.y = 0;
		velocity_msg.angular.z = 0;
		printf("velocity = %lf \n",velocity_msg.linear.x);
	}	
	
	else
	{	
		if(!stop_replanning)
		{	
		
			// double query_time = x/(double)set_loop_rate;			
			// double query_index = query_time/exec_t_step;                   /////////// (x/loop_rate) will be between 2 timesteps of index*exec_t_step
			// double index_time_1 = floor(query_index)*exec_t_step;
			// double index_time_2 = ceil(query_index)*exec_t_step;

			// cout<<"query_time="<<query_time<<" query_index="<<query_index<<" index_time1="<<index_time_1<<" index_time2="<<index_time_2<<endl;
			// cout<<"global_velocity1="<<global_velocity[(int)floor(query_index)]<<" global_velocity2="<<global_velocity[(int)ceil(query_index)]<<endl;
			// cout<<"global_omega1="<<global_omega[(int)floor(query_index)]<<" global_omega2="<<global_omega[(int)ceil(query_index)]<<endl;
			

			// double final_velocity = ((index_time_2 - query_time)*global_velocity[(int)floor(query_index)] + (query_time-index_time_1)*global_velocity[(int)ceil(query_index)])/(index_time_2 - index_time_1) ;
			// double final_omega = ((index_time_2 - query_time)*global_omega[(int)floor(query_index)] + (query_time-index_time_1)*global_omega[(int)ceil(query_index)])/(index_time_2 - index_time_1) ;

			
			double query_index = (x/(double)set_loop_rate)/exec_t_step;                   /////////// (x/loop_rate) will be between 2 timesteps of index*exec_t_step
			
			double final_velocity = ((ceil(query_index)-query_index)*global_velocity[(int)floor(query_index)] + (query_index-floor(query_index))*global_velocity[(int)ceil(query_index)])/(ceil(query_index) - floor(query_index)) ;
			double final_omega = ((ceil(query_index)-query_index)*global_omega[(int)floor(query_index)] + (query_index-floor(query_index))*global_omega[(int)ceil(query_index)])/(ceil(query_index) - floor(query_index)) ;
			


			velocity_msg.linear.x = final_velocity;
			velocity_msg.linear.y = 0;
			velocity_msg.linear.z = 0;
				
			velocity_msg.angular.x = 0;
			velocity_msg.angular.y = 0;
			velocity_msg.angular.z = final_omega;

			//prev_global_theta_x = global_theta_x[x];

			//printf("z = %d velocity = %lf , omega = %lf, global_theta_x = %lf \n", x,velocity_msg.linear.x, velocity_msg.angular.z, global_theta_x[x]);
			velocity_pub.publish(velocity_msg);	

			if(ceil(query_index)>=cl_pt_indices)
			{	
				stop_replanning=1;
				cout<<"stopped planning"<<endl;
			}	

		}	
		
		else
		{
			velocity_msg.linear.x = 0;
			velocity_msg.linear.y = 0;
			velocity_msg.linear.z = 0;

			velocity_msg.angular.x = 0;
			velocity_msg.angular.y = 0;
			velocity_msg.angular.z = 0;

			velocity_pub.publish(velocity_msg);	

			if(stop_replanning || last_plan)
				ros::shutdown();
		}	
	}
}



void get_global_waypoints()
{	
	cl_way_pt[0] <<  0, 0;
	cl_way_pt[1] <<  42, 0;												//////0,0; 36,0; 36,-45; 
	cl_way_pt[2] <<  42, -45;
	// cl_way_pt[3] <<  10, 30;
	// cl_way_pt[4] <<  40, 30;
	// cl_way_pt[5] <<  60, 10;

	
}

void get_center_line_test()
{

}


void get_center_line()
{
	//// 2 bezier curve (for continous dkappa/ds) with linear interpolation in between the sets of curves to get the complete center_line trajectory

	cout<<"no of cl_pt = "<<cl_traj_pt<<endl;

	// ofstream myfile;
 //  	myfile.open ("center_line_coordinates.txt"); 			

	Vector2d W1, W2, W3, U1, U2, Ud, B0, B1, B2, B3, E0, E1, E2, E3;	

	double c1=7.2364, c2=(sqrt(6)-1)*2/5, c3, kmax=max_robot_curvature;  
	double beta, gammaa, dk, hb, he, gb, ge, kb, ke ;

	int ways=cl_way_pt_no,traj_pt=0;

	c3 = (c2+4)/(c1+6);

	int i=0;
	double temp_slope1,temp_slope2;
	double t=0;

	cl_way_traj[traj_pt] << 0,0;
	Vector2d temp_pos = cl_way_traj[traj_pt], temp_pos1;
	traj_pt++;
		
	

	while(i < (ways-2))
	{	


		// W[0][0]= way_x[i]; W[0][1]= way_y[i]; 
		// W[1][0]= way_x[i+1]; W[1][1]= way_y[i+1];
		// W[2][0]= way_x[i+2]; W[2][1]= way_y[i+2];	
		
		// temp_slope1 = atan((cl_way_pt[i+1](1)-cl_way_pt[i](1))/(cl_way_pt[i+1](0)-cl_way_pt[i](0))); 	
		// temp_slope2 = atan((cl_way_pt[i+2](1)-cl_way_pt[i+1](1))/(cl_way_pt[i+2](0)-cl_way_pt[i+1](0))); 	


		W1 = cl_way_pt[i];
		W2 = cl_way_pt[i+1];
		W3 = cl_way_pt[i+2];

		// W1 = cl_way_pt[1];
		// W2 = cl_way_pt[2];
		// W3 = cl_way_pt[3];
		// //cout<<" W1 " << W1 <<" W2 "<<W2 <<" W3 "<< W3<<endl; 

		temp_slope1 = atan((W2(1) - W1(1))/(W2(0) - W1(0)));	
		temp_slope2 = atan((W3(1) - W2(1))/(W3(0) - W2(0)));
		//cout << temp_slope1 <<" "<< temp_slope1<<endl;	

		if (temp_slope1<0)
			temp_slope1=temp_slope1+PI;

		if (temp_slope2<0)
			temp_slope2=temp_slope2+PI;
		
		
		
		gammaa = fabs(temp_slope2 - temp_slope1);
		beta = gammaa/2;
		dk = (pow(c2+4,2)*sin(beta))/(54*c3*kmax*pow(cos(beta),2));
		hb=he=c3*dk; 
		gb=ge=c2*c3*dk;
		kb=ke=(6*c3*cos(beta)*dk)/(c2+4);	

		//U1 = (W[1]-W[2])/(W[1].norm() * W[2].norm());
		//U2 = (W[2]-W[3])/(W[2].norm() * W[3].norm());

		U1 = (W1-W2)/((W1-W2).norm());
		U2 = (W3-W2)/((W3-W2).norm());
		
		B0 = W2 + dk*U1;
		B1 = B0 - gb*U1;
		B2 = B1 - hb*U1;
	
		E0 = W2 + dk*U2;
		E1 = E0 - ge*U2;
		E2 = E1 - he*U2;

		Ud = (E2-B2)/((E2-B2).norm());

		B3 = B2 + kb*Ud;
		E3 = E2 - ke*Ud;


		// cout << "Here is the vector B0:\n" << B[0] << endl;	
		// cout << "Here is the vector B1:\n" << B[1] << endl;	
		// cout << "Here is the vector B2:\n" << B[2] << endl;



		////interpolation code 

		temp_pos1 = (pow(1-t,3)*B0 + 3*pow(1-t,2)*t*B1 + 3*(1-t)*pow(t,2)*B2 + pow(t,3)*B3);
		//cout<<"temp_pos1 "<<temp_pos1<<endl;

		//if(traj_pt==1)
		{	
		
			if(( temp_pos1 - temp_pos ).norm() > 0.2)
			{
				Vector2d temp_step = (temp_pos1 - temp_pos)/cl_straight_scale;
				for(int x=0; x<cl_straight_scale-1; x++)															///////////199 so that no point are exactly same, ds should be greater than zero always
				{
					cl_way_traj[traj_pt] = cl_way_traj[traj_pt-1] + temp_step;
					//cout <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl;
					//myfile <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl; 
					traj_pt++;	
				}

			}
		}
		////////////////////
			
		while(t<=1)
		{	
			cl_way_traj[traj_pt] = (pow(1-t,3)*B0 + 3*pow(1-t,2)*t*B1 + 3*(1-t)*pow(t,2)*B2 + pow(t,3)*B3); 	
			//cout <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl;
			//myfile <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl;			
			traj_pt++;
			t += cl_dt*cl_curve_scaling;
		}	
		
		while(t>=0)
		{	
			cl_way_traj[traj_pt] = (pow(1-t,3)*E0 + 3*pow(1-t,2)*t*E1 + 3*(1-t)*pow(t,2)*E2 + pow(t,3)*E3); 	
			//cout <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl;
			//myfile <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl;
			traj_pt++;
			t -= cl_dt*cl_curve_scaling;
		}


		temp_pos =  cl_way_traj[traj_pt-1];
		//cout << "temp_pos " <<temp_pos<<endl;
		i++;
	}


		////interpolation code 

		temp_pos1 = cl_way_pt[cl_way_pt_no-1];
		//cout<<"temp_pos1 "<<temp_pos1<<endl;

		//if(traj_pt==1)
		{	
		
			if(( temp_pos1 - temp_pos ).norm() > 0.2)
			{
				Vector2d temp_step = (temp_pos1 - temp_pos)/cl_straight_scale;
				for(int x=0; x<cl_straight_scale; x++)
				{
					cl_way_traj[traj_pt] = cl_way_traj[traj_pt-1] + temp_step;
					//cout <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl;
					//myfile <<cl_way_traj[traj_pt](0)<<" "<<cl_way_traj[traj_pt](1)<< endl; 
					traj_pt++;	
				}

			}
		}
		////////////////////

	//////////////////////calculation for theta and kappa (curvature)	

	theta_r[0] = atan((cl_way_traj[1](1) - cl_way_traj[0](1))/(cl_way_traj[1](0) - cl_way_traj[0](0)));
	arc_length[0] = (cl_way_traj[1] - cl_way_traj[0]).norm();
	
	for(int j=1; j<cl_traj_pt-1; j++)
	{
		theta_r[j] = atan((cl_way_traj[j+1](1) - cl_way_traj[j](1))/(cl_way_traj[j+1](0) - cl_way_traj[j](0)));
		arc_length[j] = arc_length[j-1] + (cl_way_traj[j+1] - cl_way_traj[j]).norm();
		kappa[j-1] = (theta_r[j] - theta_r[j-1])/(arc_length[j] - arc_length[j-1]);			
		
		// cout<<"theta_r(" <<(j) << ") = " << theta_r[j]<<endl;
		// cout<<"arc_length(" <<(j) << ") = " << arc_length[j]<<endl;
		// cout<<"kappa(" <<(j-1) << ") = " << kappa[j-1]<< "\n"<<endl; 
	
	}
	
	theta_r[(int)cl_traj_pt-1] = theta_r[(int)cl_traj_pt-2];
	arc_length[(int)cl_traj_pt-1] = arc_length[(int)cl_traj_pt-2];
	kappa[(int)cl_traj_pt-1] = kappa[(int)cl_traj_pt-2] = kappa[(int)cl_traj_pt-3];
	
	// cout<<"theta_r(" <<(int)cl_traj_pt-1 << ") = " << theta_r[(int)cl_traj_pt-1]<<endl;
	// cout<<"arc_length(" <<(int)cl_traj_pt-1 << ") = " << arc_length[(int)cl_traj_pt-1]<<endl;
	// cout<<"kappa(" <<(int)cl_traj_pt-1 << ") = " << kappa[(int)cl_traj_pt-1]<< "\n"<<endl; 			

	/////////////////////////////////////
			
	//myfile.close();

}


void get_current_statedata()
{
/// collect data from ROS; output: t0,  s0,vs0,as0,   d0,vd0,ad0	
	
	//extern obstacle obs[obs_no];
	static long int count=0;
	count++;

	double s_dot,s_dot_dot, d_dot, d_dot_dot;    /////from IMU
	double d_dash, d_dash_dash;					
//	double robot_current_s,robot_current_d;

	tf::Quaternion q(pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	//std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
	robot_current_yaw = yaw;


	robot_current_x = pose_msg.pose.pose.position.x;
	robot_current_y = pose_msg.pose.pose.position.y;
	

	//prev_cl_pt = 0;
	Vector2d robot_current_pos;
	robot_current_pos << robot_current_x,robot_current_y;  

	double temp_distance, lowest_distance = 1000, arc_length=0;
 	int closest_pt_index;

	for(int i=0; i<cl_traj_pt; i++)
	{
		
		temp_distance = (robot_current_pos - cl_way_traj[i]).norm();
		if(temp_distance < lowest_distance)
		{
			lowest_distance = temp_distance;
			closest_pt_index = i;
		}
			
	}
	
	for(int j=1; j<closest_pt_index+1; j++)
	{
		arc_length = arc_length + (cl_way_traj[j] - cl_way_traj[j-1]).norm();

	}

	prev_cl_pt = closest_pt_index;
	robot_current_s = arc_length;
	
	//global_delta_theta = prev_global_theta_x - theta_r[prev_cl_pt] ;
	
	///////////////////////////////////////
	Vector3d cl_vector, cl2pose_vector, cross_vector;
	
	cl_vector << (cl_way_traj[prev_cl_pt+1](0) - cl_way_traj[prev_cl_pt-1](0)),(cl_way_traj[prev_cl_pt+1](1) - cl_way_traj[prev_cl_pt-1](1)), 0;
	cl2pose_vector << (robot_current_x-cl_way_traj[prev_cl_pt](0)), (robot_current_y-cl_way_traj[prev_cl_pt](1)), 0;
	
	cross_vector = cl_vector.cross(cl2pose_vector);
	/////////////////////////////////////////

	if(cross_vector(2) >= 0)
		robot_current_d = lowest_distance;
	else
		robot_current_d = -lowest_distance;


	if(prev_cl_pt==cl_traj_pt-2)
	{
		stop_replanning = 1;
		cout<<"stopped planning"<<endl;
	}

	//printf("\nrobot current pose (x=%lf, y=%lf); (s=%lf, d=%lf) and previous center_line pt = %d \n",robot_current_x, robot_current_y, robot_current_s,robot_current_d,prev_cl_pt);

	s_dot = (sqrt(pow(pose_msg.twist.twist.linear.x,2) + pow(pose_msg.twist.twist.linear.y,2))) * cos(theta_r[prev_cl_pt] - robot_current_yaw);  
	d_dot = -(sqrt(pow(pose_msg.twist.twist.linear.x,2) + pow(pose_msg.twist.twist.linear.y,2))) * sin(theta_r[prev_cl_pt] - robot_current_yaw); 

	s_dot_dot = (sqrt(pow(accn_msg.linear_acceleration.x,2) + pow(accn_msg.linear_acceleration.y,2))) * cos(theta_r[prev_cl_pt] - robot_current_yaw); 
	d_dot_dot = -(sqrt(pow(accn_msg.linear_acceleration.x,2) + pow(accn_msg.linear_acceleration.y,2))) * sin(theta_r[prev_cl_pt] - robot_current_yaw); 

	//printf("d_dot=%lf  d_dot_dot=%lf \n",d_dot,d_dot_dot);

	d_dash = s_dot/(d_dot);
	d_dash_dash = ((d_dot_dot) - d_dash*s_dot_dot)/(pow(s_dot,2));

	//printf("d_dash=%lf  d_dash_dash=%lf \n",d_dash,d_dash_dash);


	if(count==1)
	{	
		//ROS_INFO("jabddbakdsakjd");
		t0 = 0;					//////////////get system_time /clock
		s0 = 0;
		vs0 = 0;
		as0 = 0;	
		d0 = 0;
		vd0 = 0; 
		ad0 = 0;
	}
	
	else if(count < 240)
	{	
		///for replanning update all of them with latest values
		//ROS_INFO("bababababba");
		t0 = 0;					//////////////get system_time /clock
		s0 = 0;
		vs0 = s_dot;
		as0 = s_dot_dot;	
		d0 = robot_current_d;
		vd0 = d_dot;//d_dash; 
		ad0 = d_dot_dot;//d_dash_dash;
		//printf("current state data: vs0=%lf as0=%lf d0=%lf vd0=%lf ad0=%lf \n",vs0,as0,d0,vd0,ad0);
	}

	else
	{
		///for replanning update all of them with latest values
		t0 = 0;					//////////////get system_time /clock
		s0 = 0;
		vs0 = s_dot;
		as0 = s_dot_dot;	
		d0 = robot_current_d;
		vd0 = d_dot;//d_dash; 
		ad0 = d_dot_dot;//d_dash_dash;
		//printf("current state data: vs0=%lf as0=%lf d0=%lf vd0=%lf ad0=%lf \n",vs0,as0,d0,vd0,ad0);

	}

}


void get_traj()
{
// input: current_statedata, max_deviation, max_velocity varying sets of d1,vs1   & t1,s1(optional for more sets & optimality) 
//output: sets of sd_traj 
	
	//ofstream myfile1;
	//myfile1.open ("frenet_frame_traj_1.txt");
 	
 	MatrixXd st_mat(6,6),c_st_mat(6,1),  ds_mat(6,6),c_ds_mat(6,1);
 	int traj_no = 0; 			
			

	
	as1 = 0;
	vd1 = 0;
	ad1 = 0;	
	t1 = t0 + traj_t1;
	s1 = s0 + traj_s1;
	
	st_mat << pow(t0,5),    pow(t0,4),    pow(t0,3),   pow(t0,2),   pow(t0,1), 1, 
			  pow(t1,5),    pow(t1,4),    pow(t1,3),   pow(t1,2),   pow(t1,1), 1,				
			  5*pow(t0,4),  4*pow(t0,3),  3*pow(t0,2), 2*pow(t0,1), 1,         0,
			  5*pow(t1,4),  4*pow(t1,3),  3*pow(t1,2), 2*pow(t1,1), 1,         0,
			  20*pow(t0,3), 12*pow(t0,2), 6*pow(t0,1), 2          , 0,         0,
			  20*pow(t1,3), 12*pow(t1,2), 6*pow(t1,1), 2          , 0,         0;

	ds_mat << pow(s0,5),    pow(s0,4),    pow(s0,3),   pow(s0,2),   pow(s0,1), 1, 
			  pow(s1,5),    pow(s1,4),    pow(s1,3),   pow(s1,2),   pow(s1,1), 1,				
			  5*pow(s0,4),  4*pow(s0,3),  3*pow(s0,2), 2*pow(s0,1), 1,         0,
			  5*pow(s1,4),  4*pow(s1,3),  3*pow(s1,2), 2*pow(s1,1), 1,         0,
			  20*pow(s0,3), 12*pow(s0,2), 6*pow(s0,1), 2          , 0,         0,
			  20*pow(s1,3), 12*pow(s1,2), 6*pow(s1,1), 2          , 0,         0;

	// int vs1_var=10, d1_var=10;          												//////change for number of trajectories
	// int traj_no=vs1_var*d1_var;
	// sd_traj traj[traj_no];

	double temp_vs1=0, temp_d1=(double)max_deviation, temp_t1, temp_s1;		  
	double vs1_step = (double)max_robot_velocity/vs1_var;
	double d1_step = (double)max_deviation/(d1_var/2);		

	//cout<<"deviation_step " <<d1_step<<endl;

	for(int i=0; i<vs1_var; i++)
	{
		temp_vs1 =  temp_vs1 + vs1_step;

		for(int j=0; j<d1_var; j++)
		{	

			temp_d1 = temp_d1 - d1_step;

			if((fabs(temp_d1))<0.05)
				temp_d1=0;

			c_st_mat << s0,
						s1,  
						vs0,
						temp_vs1,
						as0,
						as1;
			
			c_ds_mat << d0,
						temp_d1,  
						vd0,
						vd1,
						ad0,
						ad1;

									
			traj[traj_no].a_mat = st_mat.fullPivHouseholderQr().solve(c_st_mat);			//colPivHouseholderQr()  llt()
			traj[traj_no].b_mat = ds_mat.fullPivHouseholderQr().solve(c_ds_mat);						
			traj[traj_no].c_a_mat = c_st_mat;
			traj[traj_no].c_b_mat = c_ds_mat;
			traj[traj_no].t0 = t0;
			traj[traj_no].t1 = t1;

			//cout << "\nst:\n" << st_mat << endl;
			//cout << "\ncs:\n" << c_st_mat << endl;
			//cout << "\na:\n" << traj[traj_no].a_mat.transpose() << endl;
			//cout << "\nds:\n" << ds_mat << endl;
			//cout << "\ncd:\n" << c_ds_mat << endl;
			//cout << "\nb:\n" << traj[traj_no].b_mat.transpose() << endl;						
			
			//// cout << traj[traj_no].a_mat(0,0) <<	endl;
			//cout <<traj_no<<" "<<traj[traj_no].c_b_mat(1,0) << endl;

			//myfile1 <<"\n"<< traj[traj_no].a_mat.transpose()<<endl ;
 			//myfile1 <<"\n"<< traj[traj_no].c_a_mat.transpose()<<endl ;
			//myfile1 <<"\n"<< traj[traj_no].b_mat.transpose()<<endl ;
			//myfile1 <<"\n"<< traj[traj_no].c_b_mat.transpose()<<endl ;
			

			traj_no++;
		}

		temp_d1=(double)max_deviation;	
		//break;
	}
	
	//myfile1.close();
	//cout<<sd_traj::traj_counter<<endl;
}


void get_obstacle_sd()
{
//input: laser_data output: get obstacles from local laser frame to sd frame and their radius
//sample laser_data for non_zero values and get length and width of that obstacle
//then get center of obs in sdframe and clearing_radius (hypotenuse of lenth and width is diameter + husky_radius(0.6+0.2mclearance)) 
//closest point from center line(x,y) is s(t) and the deviation d(s) is perpendicular distance from that point 
//take only obstacles inside 10m distance 


	Vector2d obs_global_center[obs_no];
	
	for(int x=0; x<obs_no+1; x++)
	{
	
	obs_global_center[x] << obs[x].transformed_x, obs[x].transformed_y;	
	ROS_INFO("obsno%d x= %lf y= %lf", x, obs_global_center[x](0), obs_global_center[x](1));


	double temp_distance, lowest_distance = 1000, arc_length=0;
 	int closest_pt_index;

	for(int i=prev_cl_pt; i<cl_traj_pt; i++)
	{
		
		temp_distance = (obs_global_center[x] - cl_way_traj[i]).norm();
		if(temp_distance < lowest_distance)
		{
			lowest_distance = temp_distance;
			closest_pt_index = i;
		}
			
	}
	
	for(int j=prev_cl_pt+1; j<closest_pt_index+1; j++)
	{
		arc_length = arc_length + (cl_way_traj[j] - cl_way_traj[j-1]).norm();

	}
	
	///////////////////////////////////////
	Vector3d cl_vector, cl2pose_vector, cross_vector;
	
	cl_vector << (cl_way_traj[closest_pt_index+1](0) - cl_way_traj[closest_pt_index-1](0)),(cl_way_traj[closest_pt_index+1](1) - cl_way_traj[closest_pt_index-1](1)), 0;
	cl2pose_vector << (obs_global_center[x](0)-cl_way_traj[closest_pt_index](0)), (obs_global_center[x](1)-cl_way_traj[closest_pt_index](1)), 0;
	
	cross_vector = cl_vector.cross(cl2pose_vector);
	/////////////////////////////////////////


	if(cross_vector(2)>=0)
		obs[x].center_d = lowest_distance;
	else
		obs[x].center_d = -lowest_distance;
	
	obs[x].center_s = arc_length;
	obs[x].clearing_radius = obstacle_radius[x] + (double)robot_radius + safety_clearance; 
	 
	cout << "obstacle("<<x+1<<") s,d = " << obs[x].center_s <<"," <<obs[x].center_d	<<" radius = " << obs[x].clearing_radius<< " and the closest point on center line = "<< cl_way_traj[closest_pt_index](0) << ","<< cl_way_traj[closest_pt_index](1)<<endl; 
	
	obs[x].vs = obs[x].vd = 0;
	// obs[x].vs = (sqrt(pow(pose_msg.twist.twist.linear.x,2) + pow(pose_msg.twist.twist.linear.y,2))) * cos(theta_r[closest_pt_index] - obs[x].orientation);  
	// obs[x].vd = -(sqrt(pow(pose_msg.twist.twist.linear.x,2) + pow(pose_msg.twist.twist.linear.y,2))) * sin(theta_r[closest_pt_index] - obs[x].orientation); 


	}


}

void publish_marker_loop()
{
	long int temp=0;
		
	for(int i=0; i<sd_traj::traj_counter; i++)
	{	
		for(int id=0; id<cl_pt_indices; id++)
		{
			double x = traj[i].st[id];
			double y = traj[i].ds[id];	

			visualization_msgs::Marker marker;
			marker.header.frame_id = "odom";
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			marker.id = temp;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			temp++;
			//only if using a MESH_RESOURCE marker type:
			//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
			vis_pub.publish( marker );
			int temp1=20000;
			while(temp1--);


		}	
	}	
}

void publish_marker(double x , double y)
{
	static long int id=0;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	id++;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub.publish( marker );
}

void check_collision()
{
/// one by one take all points on the sd_traj and compute distances from all available objects & simultaneously check if it is more than clearing_radius 
/// give collision_status 0(no collision) & 1(collision_detected)

	// ofstream myfile2;
	// myfile2.open ("traj_x_y_1.txt");

	double t_div = cl_pt_indices;
	int temp_collision=0;


	for(int i=0; i< sd_traj::traj_counter ;i++)
	{
		double temp_T=0, temp_D=0;
		double obs_t_step = (traj[i].t1 - traj[i].t0)/t_div;

		double temp_st,	temp_ds;

		traj[i].collision_status = 0;					



		for(int i1=0; i1<t_div; i1++)
		{
			
			temp_st = traj[i].a_mat(0,0)*pow(temp_T,5) + traj[i].a_mat(1,0)*pow(temp_T,4) + traj[i].a_mat(2,0)*pow(temp_T,3) + traj[i].a_mat(3,0)*pow(temp_T,2) + 
								traj[i].a_mat(4,0)*temp_T + traj[i].a_mat(5,0);
			///// s(T)
								
			temp_ds = traj[i].b_mat(0,0)*pow(temp_st,5) + traj[i].b_mat(1,0)*pow(temp_st,4) + traj[i].b_mat(2,0)*pow(temp_st,3) + traj[i].b_mat(3,0)*pow(temp_st,2) + 
								traj[i].b_mat(4,0)*temp_st + traj[i].b_mat(5,0);

			
			traj[i].st[i1] = temp_st;
			traj[i].ds[i1] = temp_ds;

			//if(i1%10==0)
			//publish_marker(temp_st,temp_ds);

			////// d(s(T))

			//if(i==0)					
			// {
			//myfile2 <<""<< temp_st<<" "<< temp_ds<<" "<<i<<" "<<planning_instant<<endl ;
 		// 	}

			
			for(int j=0; j<obs_no+1; j++)
			{

				// obs[j].transformed_x = obs[j].transformed_x + obs[j].velocity*cos(obs[j].orientation)*obs_t_step;
				// obs[j].transformed_y = obs[j].transformed_y + obs[j].velocity*sin(obs[j].orientation)*obs_t_step;
				// get_obstacle_sd();

				// obs[j].center_s = obs[j].center_s + obs[j].vs*obs_t_step;
				// obs[j].center_d = obs[j].center_d + obs[j].vd*obs_t_step;

				temp_D = pow(fabs(obs[j].center_s - temp_st) ,2) + pow(fabs(obs[j].center_d - temp_ds) ,2);      //// D^2 > rc^2 for every obstacle on the trajpoint selected   

				//cout<<"distance = "<<sqrt(temp_D) << endl;
				//cout<< obs[j].clearing_radius <<" radius"<<	endl;

				//double temp_adu = sqrt(temp_D) - obs[j].clearing_radius ;
				//static int adu_counter=0;
				//if((traj[i].c_b_mat(1,0)==0))
				//cout<<temp_adu<<"    "<<i<<"  "<<temp_ds<<"  "<<temp_st <<endl;

			

				if((sqrt(temp_D)) - (obs[j].clearing_radius)<=0)
				{
					//adu_counter++;	
					traj[i].collision_status = 1;
					//cout <<"traj "<<i<< " has collided and its deviation = " << traj[i].c_b_mat(1,0) <<endl;
					temp_collision++;
					
				}
				// else
				// {
				// 	traj[i].collision_status = 0;
				// 	//cout<<"not collided "<< i <<endl;
					
				// }

				if(traj[i].collision_status == 1)
				break;	


			}
			
			if(traj[i].collision_status == 1)
				break;	

			temp_T = temp_T + obs_t_step;

		}	

	}

	if(temp_collision >= vs1_var*d1_var)
		{	
			//stop_replanning=1; 
			cout<<"stopped planning-obs "<<temp_collision<<endl;
		}

	///////////print collision_status of all trajectory
	// for(int i=0; i<sd_traj::traj_counter; i++)
	// {cout << "collision of traj(" <<i<< ") = " << traj[i].collision_status << " and its deviation = " << traj[i].c_b_mat(1,0)<< endl;}

	//myfile2.close();

}



bool sort_low_cost (sd_traj& traj_a, sd_traj& traj_b)
{
	return (traj_a.cost < traj_b.cost);
}


void compute_cost()
{
///calculate cost for collision_status-0 and simultaneously sort sd_traj ascendingly 
	
	
	//double kj_lon=0.001, kt_lon=1, ks_lon=0, kj_lat=0.0001, ks_lat=0.0001, kd_lat=20, k_lon=1, k_lat=1;							// tune heuristics
	double kj_lon=2, kt_lon=0, ks_lon=0, kj_lat=2, ks_lat=0.0000, kd_lat=10,kv_lon=-10; 
	double k_lon=1, k_lat=1;																									// tune heuristics



	for(int i=0; i< sd_traj::traj_counter ;i++)
	{
		
		double cost_lon=0, cost_lat=0;

		if((traj[i].collision_status == 0) || obs_no<0)
		{	


			double del_t = traj[i].t1 - traj[i].t0 , del_s = traj[i].c_a_mat(1,0)-s0, v = traj[i].c_a_mat(3,0);	    				//////del_t=t1-t0, del_s=s1-s0 of that particular traj

			double cost_jerklon = 720*(pow(traj[i].a_mat(0,0),2)*pow(del_t,5)) + (720*(traj[i].a_mat(0,0)*traj[i].a_mat(1,0))*pow(del_t,4)) + 
								(240*(traj[i].a_mat(0,0)*traj[i].a_mat(2,0)) + 192*(pow(traj[i].a_mat(1,0),2)))*pow(del_t,3) + (144*(traj[i].a_mat(1,0)*traj[i].a_mat(2,0))*pow(del_t,2))+ + (36*(pow(traj[i].a_mat(2,0),2)*del_t));

			cost_lon = kj_lon*cost_jerklon + kt_lon*del_t + ks_lon*pow(del_s,2) + kv_lon*v;

			double cost_jerklat = 720*(pow(traj[i].b_mat(0,0),2)*pow(del_s,5)) + (720*(traj[i].b_mat(0,0)*traj[i].b_mat(1,0))*pow(del_s,4)) + 
								(240*(traj[i].b_mat(0,0)*traj[i].b_mat(2,0)) + 192*(pow(traj[i].b_mat(1,0),2)))*pow(del_s,3) + (144*(traj[i].b_mat(1,0)*traj[i].b_mat(2,0))*pow(del_s,2))+ + (36*(pow(traj[i].b_mat(2,0),2)*del_s));

			cost_lat = kj_lat*cost_jerklat + ks_lat*del_s + kd_lat*pow((traj[i].c_b_mat(1,0)),2);
			
			traj[i].cost = k_lat* cost_lat + k_lon* cost_lon;
			
			//cout << "cost of traj(" <<i<< ") = " << traj[i].cost << " and its deviation = " << traj[i].c_b_mat(1,0) << endl;  
		}

		else
		{
			traj[i].cost = 1000000;
			
		}

	}

	//std::sort(traj, traj+sd_traj::traj_counter, [] (sd_traj& traj_a, sd_traj& traj_b) -> bool { return (traj_a.cost < traj_b.cost); });    

	std::sort(traj, traj+sd_traj::traj_counter, sort_low_cost);									////sort all trajectories according to minimum cost

	///////////print low_cost_sorted trajectory
	// for(int i=0; i<sd_traj::traj_counter; i++)
	// {cout << "cost of traj(" <<i<< ") = " << traj[i].cost << " and its deviation = " << traj[i].c_b_mat(1,0) << endl;  }

}




void check_curvature_convert2vw()
{
////start from lowest-cost traj and calculate Kmax and check till condition is satisfied
	
	//ofstream myfile3;
	//myfile3.open ("final_traj.txt");

	
	///////////////////////kappa_max and velocity omega

	if(!stop_replanning)
	{	

	for(int i=0; i< sd_traj::traj_counter ;i++)
	{	
		cl_pt_indices=0;
		double temp_arclength = traj[i].c_a_mat(0,0) ; 
		while(temp_arclength < traj[i].c_a_mat(1,0))
		{
			temp_arclength = temp_arclength + (arc_length[prev_cl_pt + cl_pt_indices+1]-arc_length[prev_cl_pt + cl_pt_indices]);
			cl_pt_indices++;
		}
		
		//cout << "no of center_line points included " << cl_pt_indices <<" and total arc_length is " << temp_arclength << endl;
		
		if((cl_pt_indices+prev_cl_pt)>=cl_traj_pt)
		{	
			last_plan=1; //stop_replanning=1;
			cout<<"lastplanning"<<endl;
			return;
		}
 
		double temp_T=0;
		exec_t_step = (traj[i].t1 - traj[i].t0)/cl_pt_indices;
		//cout << "time_step = "<<exec_t_step <<endl;
		double s[cl_pt_indices], d[cl_pt_indices], s_dot[cl_pt_indices], d_dash[cl_pt_indices], d_dot[cl_pt_indices], s_dot_dot[cl_pt_indices], d_dash_dash[cl_pt_indices], d_dot_dot[cl_pt_indices];
		double del_theta[cl_pt_indices],final_kappa[cl_pt_indices-1];
		double temp_a,temp_b;

		traj[i].max_curvature_status = 0;	
			
		for (int k = 0; k < cl_pt_indices; k++)
		{
			s[k] = traj[i].a_mat(0,0)*pow(temp_T,5) + traj[i].a_mat(1,0)*pow(temp_T,4) + traj[i].a_mat(2,0)*pow(temp_T,3) + traj[i].a_mat(3,0)*pow(temp_T,2) + 
					traj[i].a_mat(4,0)*temp_T + traj[i].a_mat(5,0); 					
			d[k] = traj[i].b_mat(0,0)*pow(s[k],5) + traj[i].b_mat(1,0)*pow(s[k],4) + traj[i].b_mat(2,0)*pow(s[k],3) + traj[i].b_mat(3,0)*pow(s[k],2) + 
					traj[i].b_mat(4,0)*s[k] + traj[i].b_mat(5,0); 					 		
			
			s_dot[k] = 5*traj[i].a_mat(0,0)*pow(temp_T,4) + 4*traj[i].a_mat(1,0)*pow(temp_T,3) + 3*traj[i].a_mat(2,0)*pow(temp_T,2) + 2*traj[i].a_mat(3,0)*temp_T + 
						traj[i].a_mat(4,0);
			d_dash[k] = 5*traj[i].b_mat(0,0)*pow(s[k],4) + 4*traj[i].b_mat(1,0)*pow(s[k],3) + 3*traj[i].b_mat(2,0)*pow(s[k],2) + 2*traj[i].b_mat(3,0)*s[k] + 
						traj[i].b_mat(4,0);
			
			d_dot[k] = s_dot[k]*d_dash[k];
			
			s_dot_dot[k] = 20*traj[i].a_mat(0,0)*pow(temp_T,3) + 12*traj[i].a_mat(1,0)*pow(temp_T,2) + 6*traj[i].a_mat(2,0)*temp_T + 2*traj[i].a_mat(3,0);
			d_dash_dash[k] = 20*traj[i].b_mat(0,0)*pow(s[k],3) + 12*traj[i].b_mat(1,0)*pow(s[k],2) + 6*traj[i].b_mat(2,0)*s[k] + 2*traj[i].b_mat(3,0);

			d_dot_dot[k] = d_dash_dash[k]*pow(s_dot[k],2) + d_dash[k]*s_dot_dot[k];			

			global_theta_x[k] = theta_r[prev_cl_pt+k] + atan(d_dash[k]/(1 - kappa[prev_cl_pt+k]*d[k]));					/////////////////// omega
			del_theta[k] = global_theta_x[k] - theta_r[prev_cl_pt+k];

			global_velocity[k] = (1 - kappa[prev_cl_pt+k]*d[k])*s_dot[k]/cos(del_theta[k]);				/////////////////// velocity

			//cout << "v = "<<global_velocity[k] << " theta = "<< global_theta_x[k] <<" and traj "<<i << " and point "<< k<<endl;

			//cout<<"haha "<<k<<endl;

			if(k>0)
			{		
				//////////omega calculation////////////////
				global_omega[k-1] = (global_theta_x[k] - global_theta_x[k-1])/exec_t_step; 	

				////////////kappa_max calculation////////////				
				temp_a = (kappa[prev_cl_pt+k]*d[k] - kappa[prev_cl_pt+k-1]*d[k-1])/(s[k] - s[k-1]);
				temp_b = ((d_dash_dash[k] + temp_a*tan(del_theta[k])) * (pow(cos(del_theta[k]),2)/(1 - kappa[prev_cl_pt+k]*d[k]))) + kappa[prev_cl_pt+k];
				final_kappa[k] = temp_b * (cos(del_theta[k])/(1 - kappa[prev_cl_pt+k]*d[k]));
				
				//cout<<"haha "<<k<<endl;
				//cout<< "final_kappa = "<<final_kappa[k] <<endl;

				//if ((final_kappa[k] < -max_robot_curvature) || (final_kappa[k] > max_robot_curvature))
				if (fabs(final_kappa[k]) > (max_robot_curvature+0.02))
				{	
					traj[i].max_curvature_status = 1;     
					cout << "curvature exceeded for traj:" << i << " and kappa_max = "<< fabs(final_kappa[k])<< endl;
				
					if(i==(vs1_var*d1_var-1))
					{	
						stop_replanning=1;
						cout<<"stopped planning - curv"<<endl;
					}

					break;					

				}
				else
					traj[i].max_curvature_status = 0;	
			
				

				// if(traj[i].max_curvature_status == 1)
				// {
				// 	cout << "curvature exceeded for traj:" << i << " and kappa_max = "<< fabs(final_kappa[k])<< endl;
				
				// 	if(i==(vs1_var*d1_var-1))
				// 	stop_replanning=1;

				// 	break;
				// }

			}	

			temp_T = temp_T + exec_t_step;			
		}

		if(traj[i].max_curvature_status == 0)
		{
			optimum_traj_no = i;
			cout << "optimum_traj= " <<optimum_traj_no <<", deviation= "<<traj[optimum_traj_no].c_b_mat(1,0) <<", velocity= "<<traj[optimum_traj_no].c_a_mat(3,0) <<"\n"<<endl;

			//////////////storing velocity and theta values////////////
			// for (int l = 0; l < cl_pt_indices; ++l)
			// {
			// 		//myfile3 << global_velocity[l] <<" " << global_theta_x[l] << endl;
			// }

			break;
		}

		temp_T = 0;

	}	

	// global_x[0] = robot_current_x; global_y[0] = robot_current_y;
	// double exec_t_step = (traj[optimum_traj_no].t1 - traj[optimum_traj_no].t0)/cl_pt_indices;
	
	// for (int m = 1; m < cl_pt_indices; m++)
	// {
	// 	global_x[m] = global_x[m-1] + global_velocity[m-1]*cos(global_theta_x[m-1])*exec_t_step;
	// 	global_y[m] = global_y[m-1] + global_velocity[m-1]*sin(global_theta_x[m-1])*exec_t_step;			
	// 	//myfile3 << global_x[m] <<" " << global_y[m] << endl;
	// }

	//myfile3.close();
	}
}

// void convert2worldxy()
// {
// 	global_x[0] = global_y[0] = 0;
// 	double exec_t_step = (traj[optimum_traj_no].t1 - traj[optimum_traj_no].t0)/cl_traj_pt;
	
// 	for (int m = 1; m < cl_traj_pt; ++m)
// 	{
// 		global_x[m] = global_x[m-1] + global_velocity[m-1]*cos(global_theta_x[m-1])*exec_t_step;
// 		global_y[m] = global_y[m-1] + global_velocity[m-1]*sin(global_theta_x[m-1])*exec_t_step;			
// 	}

// }



int main(int argc, char **argv)
{
  
  	ros::init(argc, argv, "local_planner");

  	ros::NodeHandle n;

	//ros::Subscriber odom_sub = n.subscribe("/odometry/filtered", 1, odom_callback);
	//ros::Subscriber scan_sub = n.subscribe("/scan", 1, scan_callback );
	ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, imu_callback);
	
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "/odometry/filtered", 1);
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(n, "/scan", 1);
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, scan_sub);
	sync.registerCallback(boost::bind(&odom_scan_callback, _1, _2));

  	velocity_pub = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 0);
  	vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 2000000);

  	ros::Rate loop_rate(set_loop_rate);

 	///////////////////wait for initial data 10000000 loop
 
 	get_global_waypoints();
	get_center_line();
	
	for(int w=0; w<10000000; w++)
	{	
		ros::spinOnce();
		if((w%1000000)==0 )
		publish_vw(-1);
	}
	
	//clock_t tStart = clock();
	get_current_statedata();
	get_traj();
	if(obs_no>=0)
	{	
		get_obstacle_sd();
		check_collision();
	}
	compute_cost();
	check_curvature_convert2vw();
	int z=0;
	//printf("Exec Time: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

	//publish_marker_loop();

  	while (ros::ok())
  	{
    	
  		get_current_statedata();

  		if((z==(int)(set_loop_rate/replanning_freq)))
  		{	

  			printf("\nrobot current pose (x=%lf, y=%lf); (s=%lf, d=%lf) and previous center_line pt = %d \n",robot_current_x, robot_current_y, robot_current_s,robot_current_d,prev_cl_pt);
  			printf("current state data: vs0=%lf as0=%lf d0=%lf vd0=%lf ad0=%lf \n",vs0,as0,d0,vd0,ad0);
  			
  			//int y=0;
    		//clock_t tStart = clock();
			//get_current_statedata();
			//y++; ROS_INFO("y = %d",y);
			get_traj();
			//y++; ROS_INFO("y = %d",y);
			
			if(obs_no>=0)
			{	
				get_obstacle_sd();
				//y++; ROS_INFO("y = %d",y);
				check_collision();
				//y++; ROS_INFO("y = %d",y);	
			}
			
			compute_cost();
			//y++; ROS_INFO("y = %d",y);
			check_curvature_convert2vw();
			//y++; ROS_INFO("y = %d",y);
			if(!last_plan)
			z=0;
			//printf("Exec Time: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
			planning_instant++;
  		}

 		publish_vw(z+1);
  		z++;
  		//ROS_INFO("z = %d",z);

  		//if(!stop_replanning)
  			//ros::shutdown();
  		
    	ros::spinOnce();
    	loop_rate.sleep();
    }


  	return 0;

}
