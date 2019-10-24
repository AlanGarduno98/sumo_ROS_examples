#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sumo_pet/BoundingBoxes.h>
#include <eigen3/Eigen/Eigen>
#include <sumo_pet/BoundingBox.h>
#include <ros/ros.h>
#include <stdlib.h>     
#include <time.h>
#include <iostream>
#include <vector>

using namespace std;

std_msgs::String string_msg;
ros::Publisher posture_pub, _cmd_vel_pub, high_jump_pub, sharp_turn_pub, anim_pub, rossumo_publisher;
ros::Subscriber anim_yolo, vicon_subscriber;
geometry_msgs::Twist v;

int counter=0;
std::vector<darknet_ros_msgs::BoundingBox> bb;
double v_x = 0;
double v_y = 0;
double v_z = 0;
double q_x = 0;
double q_y = 0;
double q_z = 0;
double q_w = 0;
double roll=0,pitch=0,yaw=0;
int i,j,k;
Eigen::Affine3d eig;
Eigen::Quaterniond norm;
Eigen::Affine3d q1_eig;
Eigen::Affine3d q0_eig;
Eigen::Matrix4d r0_eig;
Eigen::Affine3d p1_eig;
Eigen::Affine3d p0_eig;
Eigen::Matrix4d t0_eig;


double rand_float( double low, double high ) {
    return ( ( double )rand() * ( high - low ) ) / ( double )RAND_MAX + low;
}

void randomMovement(int r1, int r2, int r3, int r4, int r5)
{
	double f1=(double)r1;
	f1=f1/100;
	double f2=(double)r2;
	f2=f2/75;
	double f3=(double)r3;
	f3=f3/83;
	double f4=(double)r4;
	f4=f4/5;
	v.linear.x = f1;
	if(f3 < 0.5){f2 = f2*-1;}
	v.angular.z = f2;
	if(f4 > 0.95)
	{
		v.linear.x = 0;
		v.angular.z = 1*0.5;
	}
	if(f4 <= 0.05)
	{
		v.linear.x = 0;
		v.angular.z = -1*0.5;
	}
	if(f4 <= 0.40 && f4 >= 0.35)
	{
		v.linear.x = 1*0.5;
		v.angular.z = 0;
	}
	if(r5 <= 20)
	{
		string_msg.data = "slalom";
		anim_pub.publish(string_msg);
	}
	if(r5 >= 980)
	{
		string_msg.data = "metronome";
		anim_pub.publish(string_msg);
	}
	for(int i=0;i<80;i++)
	{
		//v.linear.x = v.linear.x/2;
		//v.angular.z = v.angular.z/2;
		_cmd_vel_pub.publish(v);
		ros::Duration(0.01).sleep();
	}
	
	//v.linear.x = v.linear.x/2;
	//v.angular.z = v.angular.z/2;

}

void transformTFToEigen(Eigen::Affine3d &eig, geometry_msgs::TransformStamped::ConstPtr& msg)
{
        double q_x,q_y,q_z,q_w;
        q_x = msg->transform.rotation.x;
        q_y = msg->transform.rotation.y;
        q_z = msg->transform.rotation.z;
        q_w = msg->transform.rotation.w;

        double x,y,z;
        x = msg->transform.translation.x;
        y = msg->transform.translation.y;
        z = msg->transform.translation.z;

        Eigen::Quaterniond q(q_x,q_y,q_z,q_w);
        Eigen::Vector3d pos(x,y,z);

        Eigen::Matrix3d Re(q);
        eig.translation() = pos;
        eig.linear() = Re;
std::cout<<eig.translation()<<std::endl;
std::cout<<eig.linear()<<std::endl;
        return;
}

void messageCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	bb = msg -> bounding_boxes;
	for(int i=0; i < bb.size(); i++)
	{
		//if(bb[i].Class == "sports ball")
		//if(bb[i].Class == "cell phone")
		if(bb[i].Class == "person")
		{
			counter++;
			string_msg.data = "tap";
			if(counter >= 15)
			{
				for(int k=0;k<3;k++)
				{
					anim_pub.publish(string_msg);
					ros::Duration(0.01).sleep();
				}
				std::cout<<"I can see myself !!!"<<std::endl;
				v.linear.x = 0;
				v.angular.z = 1;
				_cmd_vel_pub.publish(v);
				ros::Duration(0.01).sleep();
				counter	= 0;
			}
		}
	}
	
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "sumo_pet_simple");
	ros::NodeHandle nh_public, nh_private("~");

	//Subscribers
	anim_yolo = nh_public.subscribe("/darknet_ros/bounding_boxes",30,messageCallback);
	vicon_subscriber = nh_public.subscribe("/vicon/SUMO_11_5_2018/SUMO_11_5_2018",10,subscribecall);
	// publishers
  	anim_pub = nh_public.advertise<std_msgs::String>("/rossumo1/anim", 1);
  	_cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("/rossumo1/cmd_vel_norm", 1);

	ros::Rate r(100);
	srand((int)time(0));

	while(nh_public.ok())
	{
		int r1 = (rand() % 100) + 1;
		int r2 = (rand() % 75) + 1;
		int r3 = (rand() % 83) + 1;
		int r4 = (rand() % 5) + 1;
		int r5 = (rand() % 1000) + 1;
		//ROS_INFO("r1: %d | r2: %d",r1,r2);
		randomMovement(r1, r2, r3, r4, r5);
		_cmd_vel_pub.publish(v);
		ros::spinOnce();
		r.sleep(); 
	} 
	
	Eigen::Matrix4d r0_eig;
        r0_eig << 1,0,0,0,
               0,1,0,0,
               0,0,0,0,
               0,0,0,1;
        Eigen::Matrix4d q0_eig;
        q0_eig= eig.linear();
//d1 is affine data type 
        Eigen::Affine3d d1;

//transform affined3d datatype d1 to a matrix
q1_eig.matrix() = r0_eig*q0_eig.matrix();
        d1.matrix() = q1_eig.matrix() - q0_eig.matrix();
//transform q1_eig and q0_eig into a matrix and then subtract the vectors 
        float norm;
//store the normalization of the d1 matrix into a floating number to later compare in the while loop
        norm = d1.matrix().norm();

        while(n.ok()&& norm > 0)
	{
	        d1.matrix() = q1_eig.matrix()-q0_eig.matrix();
                norm = d1.matrix().norm();
                v.linear.x = 0;
                v.angular.z = 0.35;
                rossumo_publisher.publish(v);
                ros::spinOnce();
                rate.sleep();
        }

        while(n.ok()&& norm !=0)

        {
                d1.matrix() = q1_eig.matrix() - q0_eig.matrix();
                norm = d1.matrix().norm();
                v.linear.x = 0.3;
                v.angular.z = 0;
                rossumo_publisher.publish(v);
                ros::spinOnce();
                rate.sleep();
        }


        while(n.ok()&& norm !=0)
        {
                d1.matrix() = q1_eig.matrix() - q0_eig.matrix();
                norm = d1.matrix().norm();
                v.linear.x = 0;
                v.angular.z = 0.35;
                rossumo_publisher.publish(v);
                ros::spinOnce();
                rate.sleep();
        }


	v.linear.x = 0;
	v.angular.z = 0;
	_cmd_vel_pub.publish(v);
  	return 0;
}
