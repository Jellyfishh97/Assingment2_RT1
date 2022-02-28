// IMPORTING LIBRARIES 

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "secondAssignment/Accelerate.h"
#include "std_srvs/Empty.h"

///////////////////////////

// The Laser of the Robot returns an array of 721 elements,

#define ARRAY_SIZE 721

// Initializing ros::Publisher object to publish the velocity of the robot.

ros::Publisher pub; 

// Initializing a std_srvs::Empty object
// in order to call the standard service /reset_positions

std_srvs::Empty Reset;

// FUNCTIONS DEF

bool Server(secondAssignment::Accelerate::Request &req, secondAssignment::Accelerate::Response &res);

float Wall_Detection(int range_min , int range_max , float laser_array[]);

void RotCallback(const sensor_msgs::LaserScan::ConstPtr& msg);



geometry_msgs::Twist reaccelerate(geometry_msgs::Twist my_vel);


// sumCurve: variable used to gradually increase the robot velocity after a curve.
float sumCurve = 0.5;

// aRate: variable that estimates the degree of acceleration of the robot .
float aRate = 1.0;

// state: boolean variable to understand whether the robot has turned right after or not.
bool state = true;

//MAIN

int main(int argc , char **argv){
	
	// Initializing the Controller_Node.
	ros::init(argc ,argv, "carcontroller_node");

	// Setupping the NodeHandle.
	ros::NodeHandle n;

	// Setting up the publisher for the topic /cmd_vel.
	pub = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1000); 
	
	// Defining the subscriber to the topic /base_scan
	ros::Subscriber sub = n.subscribe("/base_scan", 1000,RotCallback); 

	// Defining the service providing its name "/accelerate" and the callback "Server".
	ros::ServiceServer service =  n.advertiseService("/accelarate", Server); 

	//Loop and call the Callback functions.
	ros::spin();

	return 0;

}


float Wall_Detection(int range_min , int range_max , float laser_array[]){

	/*
		Function to detect the minimum distance between the robot and the wall (180 deg f.o.w.)

		Returns: lowest_value (float) = dist between the walland the robot. 
	*/

	float lowest_value = 9.0;
	
	// loop for detecting the lowest value
	for( int i = range_min ; i < range_max ; i++ ){
				
		if(laser_array[i] <= lowest_value )
		{ 
			lowest_value = laser_array[i];
		}	
	}
				
	return lowest_value;	
				
}

void RotCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

	//Callback function that subscribes to the topic /base_scan, it publishes the velocity of the robot

	// Initializing a geometry_msgs::Twist object that contains both the linear and the angular velocity of the robot.
	geometry_msgs::Twist my_vel;

	// Initializing an array that will contain all the robot's laser returns.
	float Laser_Array[ARRAY_SIZE];
	
	// Itializing float variables for the distances between the robot and the wall.
	float dist_right, dist_left , dist_front;
	
	// Putting in the Laser array, datas coming from the topic /base_scan.
	for(int i = 0; i < ARRAY_SIZE ; i++ ){ Laser_Array[i] = msg->ranges[i]; }

	// Detecting walls on the robot's surrounding
	dist_front = Wall_Detection(290,430,Laser_Array);
	dist_right = Wall_Detection(0,100,Laser_Array);
	dist_left = Wall_Detection(620,720,Laser_Array);
	
	
	// MOVEMENT 

	// Checking if there's a wall in front of the robot.
	if ( dist_front < 1.5 )
	
	{

		state = false; // setting state to false: the robot is going to turn.
	
		 
		if ( dist_right < dist_left ){
			my_vel.angular.z = 3.0;
			my_vel.linear.x = 0.3;
		}
	
		else if ( dist_right > dist_left ){ 
			my_vel.linear.x = 0.3;
			my_vel.angular.z = -3.0;
		}
	}
	else{

		// no wall in front 
		switch(state)
		{

			// The robot hasn't just turned 
			case true:

				my_vel.linear.x = 2.0 * aRate;
				my_vel.angular.z = 0.0;

			break;

			// The robot has just turned 
			case false:

				my_vel = reaccelerate(my_vel);

			break;
		}	
	}

	std::cout<< "  VELOCITY: " << my_vel.linear.x << std::endl;

	fflush(stdout);

	// publish the linear and angular velocity 
	pub.publish(my_vel);
}

bool Server(secondAssignment::Accelerate::Request &req , secondAssignment::Accelerate::Response &res ){
	
	/*
		The proper Server:
		this function handle the input of the user aquired in the UI_Node.
		press [a],  aRate is increased.
		press [d],  aRate is decreased.
		press [r], the service /reset_position is called, 
		if the user press a wrong key an error will be printed.
	*/

	switch(req.input){
		case 'a':
			aRate += 0.4;
			res.value = aRate;
			
		break;
		case 'd':
			aRate -= 0.4;
			res.value = aRate;
			
		break;
		case 'r':
			ros::service::call("/reset_positions", Reset);
			aRate = 1.0;
			res.value = aRate;
			
		break;
		
		default:
			res.value = -1.0; // handling the missclick of the user
		break;
	}
	
	return true;
}


geometry_msgs::Twist reaccelerate(geometry_msgs::Twist my_vel){

	
	//	Function to increse gradually  the velocity of the robot after a curve (just like a true car!)
		
	//	returns: my_vel: a geometry_msgs::Twist object, velocity of the Robo.
	

	if((aRate/2) <= sumCurve){
		
		state = true; // when the robot has achived an certain velocity change the state var
		sumCurve = 0.5;													
	}
	else{
		my_vel.angular.z = 0.0;
		my_vel.linear.x = 1.0 * sumCurve;
		sumCurve += 0.1;						
	}

	return my_vel;

}

