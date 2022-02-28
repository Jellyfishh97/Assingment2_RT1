# ROS Monza Carcontroller 
## Second Assignment of the course Research_Track_1, Robotics Engineering UNIGE.

-----------------------

This project makes use of a set of software libraries [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) to build a robot application that makes a robot run autonomously in the Monza circuit.

I created two nodes: 
* One to control autonomously the movement of the robot.
* The second one to create a really basic basic Uman Interface(UI) to increase or decrease run-time the velocity run-time, and also to reset the robot position to the starting one.

Installing and running  
-----------------------

In order to run more than one node at the same time i created a `.launch` file named `toRun.launch`.

__`toRun.launch`__ : 
```xml
<launch>	
	<node name="stageros" pkg="stage_ros" type="stageros" required ="true" args = "$(find secondAssignment)/world/my_world.world"/>
	<node name="carcontroller_node" pkg="secondAssignment" type="carcontroller_node" output="screen"  required="true"/>
	<node name="UI_node" pkg="secondAssignment" type="UI_node" output= "screen" required="true"/>
	
</launch>
```

__Command to launch the project__:

```bash
	roslaunch secondAssignment toRun.launch
```
where `secondAssignment` is the name of the ros package I created for this assignment.

StageRos_Node
-------------
The stageros node simulates a world as defined in the `my_world.world` file in the folder world. This file tells Stage everything about the environment which in our case is a 2D representation of the Monza Formula 1 circuit.

Stageros Node subscribes to the topic `cmd_vel` from `geometry_msgs` package which provides a [`Twist`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) type message to express the velocity of a robot (linear and angular components).

The Stageros Node also publishes on the `base_scan` topic from `sensor_msgs` package which provides a [`LaserScan`](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) type message. 

Eventually i also used a standard service called `reset_position` from the `std_srvs` package. 

Controller_Node 
---------------

This is the most important node of the package and it contains the logic for the robot's movement and for the handling of the inputs coming from the UI_Node. 

The Node is implemented in controller.cpp in the folder src.
There are 4 main functions:

 * __Wall_detection(range_min, range_max , Laser_Array)__

	 This function is needed to compute the minimum distance between between the robot and the walls 
	 It eanble the sensor to detect an obstacle in a cone between the range_min and the range_max, not expressed in degrees but in the number of elements of an array of 721 elements. The array is read from the `base_scan` topic and it returns the distances between the robot and the circuit contour. The robot in this case in non holonomic, therefore the fow of the sensor are the 180 degrees in front of him

   	`Arguments` :

   	* range_max (`int`) : the top end of the array Laser_Scan (721th value)

	* range_min (`int`) : the lower end of the array Laser_Scan (1st value)

   	* Laser_Scan (`float`) : the array of 721 elements representing the 180° field of view in front of the robot.

   	`Returns` :

   	* lowest_value (`float`) : the minimum value of the array (the minimum distance between the robot and the walls in the interval considered).

* __RotCallback__

This Callback function will be called whenever a message is posted on the `base_scan` topic.

Thanks to the function Wall_Detection(), the robot can detect the shortest distance to the walls on its right, left and front :

``` C

	dist_front = Wall_Detection(310,410,Laser_Array); // shortest distance to wall on its front

	dist_right = Wall_Detection(0,100,Laser_Array); // shortest distance to wall on its right 
	
	dist_left = Wall_Detection(620,720,Laser_Array); //  shortest distance to wall on its left 

	// the 100 range is an empiric value


```

The Logic implemented to move the robot:

At first the robot checks for the presence of a wall in front of it at a distance of less than 1.5, if there isn't it will move straight ahead, it publishes a linear velocity on the "cmd_vel" topic which is by default equal to 2. (otherwise it will detect walls to its left and right)
If the wall is closer to the right, it will turn left and, if the wall is closer to the left, it will turn right.

* __Server__

The actual server to receive the client request from the UI_Node.

Here is where the user's keyboard input is received. A switch handles the different client requests. (__[a]__, __[d]__), __[r]__).

Acceleration and deceleration features are coded using a ,global variable that in the case of the former is incremented and in the case of the latter decremented.
The reset resets also the velocity of the robot. (bringing it bak to 2)

Is created with this function also the server's response to the client's request;
The "response" is the float containing the degree of acceleration (the value of the global variable mentioned before), that is gonna get printed to screen.

Moreover if  by any chance the user inputs a wrong key a message will appear.

* __Reaccelerate__

Of course the faster the robot the more probabilities there is for it to crush in the wall, just like cars in real life actually! To overcome this issue i tried to make the robot decelerate before a curve and reaccelerate just after it (exactly how a normal car would do!). With the help of a boolean variable indeed the program knows if the robot is just approaching a curve or if it just ended one. After it completes a curve the robot will gradually aceelerate back to the UI imposed velocity.


UI_Node 
-------

This node manages the UI of the project. 
The user can control the speed of the robot by increasing and decreasing its acceleration. 
He can also reset the robot's position to its initial state.
The UI node receives input from a terminal window.

[__[a]__]   To Accelerate

[__[d]__]  To Decelerate

[__[r]__]   To Reset the position


------

As soon as an input arrives from outside, it is transmitted to the Controller_Node, which answers back sending the "robot's acceleration".
This is a client-server architecture and has been implemented by creating a custom service called AccDec.srv . 
Structure of the service:
``` xml
     char input
     ---
     float32 value
```
Where:
* __char__ input is the character typed by the user on the keyboard: __[a]__ , __[d]__ or __[r]__.

* __float32__ value is "acceleration of the robot". 


Flowchart
----------------------

The flow chart of the project: 
![Flowchart](https://user-images.githubusercontent.com/91626281/155983707-c7c9f24f-6264-47e2-8e85-4478951a45ac.png)


Improvements ?
----------------------
- Maybe could be useful to implement a "Starting the curve" function, in order to make the robot path smoother and to make it goes faster.
For the pupose of speed, we could also maybe try to devide the f.o.w. of the robot in more slices (not only 3) in order to retrive more data, with the aim of a more precise control of the robot during curves.



