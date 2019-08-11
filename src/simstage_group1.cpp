// Wall follower robot for continuous maps 

#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// Define states of the wall-follower inside the map
#define BEGIN 1
#define LEFT 2
#define RIGHT 3
#define STRAIGHT 4
#define CHECK_DISTANCE 0.7
#define LINEAR_SPEED 1.2
#define ANGULAR_SPEED 0.7

class ReactiveController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

	//Create distance variables and set initial state to BEGIN
    double obstacle_distance_front = 999;
	double obstacle_distance_right;
	double obstacle_distance_left;	
	int state = BEGIN;


    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();


		// Use swith to alter cases according to momentary state of the robot 
    	switch(state)
		{
			case BEGIN: 
				return Begin();
				break; 
			case LEFT: 
				return GoLeft();
				break;
			case RIGHT:
				return GoRight();
				break;
			case STRAIGHT:
				return GoStraight();
				break;
	}
		
        
        return msg;
    }


	// Define initial state, move forward until detecting an object then turn right
	geometry_msgs::Twist Begin()
    {
		auto msg = geometry_msgs::Twist();

		if(obstacle_distance_front < CHECK_DISTANCE){
			state = RIGHT;
			return msg;
		}
		msg.linear.x = LINEAR_SPEED;
		return msg;
	}


	// Go straight if no frontal object exists and the right-side wall is no further the defined distance 
	geometry_msgs::Twist GoStraight()
    {
		auto msg = geometry_msgs::Twist();

		if(obstacle_distance_front < CHECK_DISTANCE - 0.2){  // Extract a value to avoid looping of state changes 
			state = RIGHT;			
			return msg;
		}

		if(obstacle_distance_left > CHECK_DISTANCE){
			state = LEFT;
			return msg;
		}

		msg.linear.x = LINEAR_SPEED;
		return msg;
	}


	// Turn left if there exists no wall on the left, else change the state to STRAIGHT 
	geometry_msgs::Twist GoLeft()
    {
		auto msg = geometry_msgs::Twist();


		if(obstacle_distance_left < CHECK_DISTANCE){
			state = STRAIGHT;
			return msg;
		}	

		msg.angular.z = ANGULAR_SPEED;
		return msg;
	}


	// Turn right if there exists a wall on the front, else change the state to STRAIGHT 
	geometry_msgs::Twist GoRight()
    {
		auto msg = geometry_msgs::Twist();

		if(obstacle_distance_front > CHECK_DISTANCE){
			state = STRAIGHT;
			return msg;
		}
		
		msg.angular.z = -ANGULAR_SPEED;
		return msg;
	}



    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
		// Use sensor samples and range to measure distance between robot and the object 
		// Use ROS_INFO to print the distance values and state of the robot
        obstacle_distance_right = *std::min_element(&msg->ranges[0], &msg->ranges[59]);
        ROS_INFO("Min distance to obstacle right: %f", obstacle_distance_right);

		obstacle_distance_front = *std::min_element(&msg->ranges[60], &msg->ranges[179]);
        ROS_INFO("Min distance to obstacle front: %f", obstacle_distance_front);

		obstacle_distance_left = *std::min_element(&msg->ranges[180], &msg->ranges[239]);
        ROS_INFO("Min distance to obstacle left: %f", obstacle_distance_left);

		ROS_INFO("State: %d", state);
    }


public:
    ReactiveController()
	{
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a subscriber for laser scans 
        this->laser_sub = n.subscribe("base_scan", 10, &ReactiveController::laserCallback, this);

    }

    void run()
	{
        // Send messages in a loop
        ros::Rate loop_rate(10);

        while (ros::ok())
        {

            ros::spinOnce();
						
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simstage_group1");

    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();

    // And make good on our promise
    return 0;
}
