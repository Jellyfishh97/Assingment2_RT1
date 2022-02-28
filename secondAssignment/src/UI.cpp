#include "ros/ros.h"
#include "secondAssignment/Accelerate.h"

// Initializing a ros::ServiceClient object that's gonna call the service.

ros::ServiceClient client;

char Input(){

	
//Function to get the user's input from the terminal console
//Returns: the character of the user
	char input;
	std::cout << "   INPUT :  ";
	std::cin >> input;
	std::cout<< "\n" ;
	return input;
}

int main (int argc, char **argv)
{
	
	// Printing on the screen the available commands
	std::cout << " In order to  accelarate press: "<<  "[a]\n";
	std::cout << " In order to  decelerate press: "<<  "[d]\n"; 
	std::cout << " In order to  reset the position press: "<<  "[r]\n\n"; 
	
	// Initializing the UI_node.
	ros::init(argc, argv, "UI_node");

	// Setting the NodeHandle 
	ros::NodeHandle n;
	
	// Creating the client 
	client = n.serviceClient<secondAssignment::Accelerate>("/accelarate");
	
	//Declaring a secondAssignment::Accelerate object
	secondAssignment::Accelerate acc;

	// if Everything goes right is an infinite loop.
	while(ros::ok())
	{
		// Input() call (declared before in the code).
		char input = Input();

		// Assigning the input to the request member of the object acc
		acc.request.input = input;
		
		// Waitingf for the service 
		client.waitForExistence();
		
		if(client.call(acc)){
		
			if(acc.response.value == -1.000000){
				std::cout << " This command does'nt exists \n";
			}
			else{

				ROS_INFO(" ACCELERATOR: %f\n ",acc.response.value);
			}
		}
		else{

			ROS_ERROR("Acceleration not succesful");

		}
	}
	return 0;
}
