#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


#define BEGIN 1
#define LEFT 2
#define RIGHT 3
#define STRAIGHT 4
#define UNSTUCK 5
#define CHECK_DISTANCE 0.5
#define LINEAR_SPEED 1
#define ANGULAR_SPEED 0.8

class ReactiveController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

    double obstacle_distance_front = 999;
	double obstacle_distance_right;
	double obstacle_distance_left;



	
    bool robot_stopped;
	int state = BEGIN;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();

/* 		if(obstacle_distance_front > CHECK_DISTANCE && obstacle_distance_right > CHECK_DISTANCE){
			state = UNSTUCK;
			//return msg;
		}*/

      
        switch(state){
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
			case UNSTUCK:
				return GetFree();
				break;
		}
		
        
        return msg;
    }

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


	geometry_msgs::Twist GoStraight()
    {
		auto msg = geometry_msgs::Twist();

		if(obstacle_distance_front < CHECK_DISTANCE){
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



	geometry_msgs::Twist GoLeft()
    {
		auto msg = geometry_msgs::Twist();

		if(obstacle_distance_front > CHECK_DISTANCE && obstacle_distance_left > CHECK_DISTANCE){
			state = UNSTUCK;
			//return msg;
		}

		if(obstacle_distance_left < CHECK_DISTANCE){
			state = STRAIGHT;
			return msg;
		}

		msg.angular.z = ANGULAR_SPEED;
		return msg;
	}


	geometry_msgs::Twist GoRight()
    {
		auto msg = geometry_msgs::Twist();

		if(obstacle_distance_front > CHECK_DISTANCE){
			state = STRAIGHT;
			return msg;
		}
		

		msg.angular.z = -ANGULAR_SPEED;

		}

	geometry_msgs::Twist GetFree()
    {
		auto msg = geometry_msgs::Twist();

		msg.linear.x = LINEAR_SPEED;

		if(obstacle_distance_front < CHECK_DISTANCE){
			state = RIGHT;
			return msg;	
		}

		if(obstacle_distance_left < CHECK_DISTANCE){
			state = STRAIGHT;
			return msg;
		}

		return msg;
	}



    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        obstacle_distance_right = *std::min_element(&msg->ranges[0], &msg->ranges[59]);
        ROS_INFO("Min distance to obstacle right: %f", obstacle_distance_right);

		obstacle_distance_front = *std::min_element(&msg->ranges[50], &msg->ranges[189]);
        ROS_INFO("Min distance to obstacle front: %f", obstacle_distance_front);

		obstacle_distance_left = *std::min_element(&msg->ranges[180], &msg->ranges[239]);
        ROS_INFO("Min distance to obstacle left: %f", obstacle_distance_left);

		ROS_INFO("state: %d", state);
    }


public:
    ReactiveController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a subscriber for laser scans 
        this->laser_sub = n.subscribe("base_scan", 10, &ReactiveController::laserCallback, this);

    }

    void run(){
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


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "simstage_group1");

    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();

    // And make good on our promise
    return 0;
}
