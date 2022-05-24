#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <std_srvs/Empty.h>
#include "turtlesim/TeleportAbsolute.h"

class TurtleLawnmower{

ros::NodeHandle nh_;

ros::Subscriber sub_;

ros::Publisher pub_;

public:
	TurtleLawnmower();
	~TurtleLawnmower();
	
	void turtleCallback(const turtlesim::Pose::ConstPtr& msg);};

TurtleLawnmower::TurtleLawnmower(){


	sub_ = nh_.subscribe("turtle1/pose",1, &TurtleLawnmower::turtleCallback, this);
	pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	
	ros::service::waitForService("turtle1/teleport_absolute");
	ros::ServiceClient spawnClient = nh_.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	turtlesim::TeleportAbsolute::Request spawn_point;
	turtlesim::TeleportAbsolute::Response response;
	spawn_point.x = 0.55;
	spawn_point.y = 0.5;
	spawnClient.call(spawn_point,response);
	
	ros::Duration(1,0).sleep();
	
	ros::service::waitForService("clear");
  	ros::ServiceClient clearClient = nh_.serviceClient<std_srvs::Empty>("/clear");
  	std_srvs::Empty srv;
	clearClient.call(srv);
}

TurtleLawnmower::~TurtleLawnmower(){}

void TurtleLawnmower::turtleCallback(const turtlesim::Pose::ConstPtr& msg){

	ROS_INFO("Turtle lawnmower@[%f, %f, %f]",msg->x, msg->y, msg->theta);
	geometry_msgs::Twist turtle_cmd_vel;
	turtle_cmd_vel.linear.x = 1;
	
	if(msg->x>=10.5){
	
		turtle_cmd_vel.angular.z = 4;

	}
	
	
	if(msg->x<=0.5){
	
	
		turtle_cmd_vel.angular.z = -4;
	}
	
	pub_.publish(turtle_cmd_vel);
	
}

int main(int argc, char **argv){

	ros::service::waitForService("/turtle1/teleport_absolute");
	

	ros::init(argc,argv,"turtle_lawnmower_node");
	
	TurtleLawnmower TMower;
	
	ros::spin();
	
	return 0;

}
