#include <pluginlib/class_list_macros.h>
#include <pluginlib_turtle_move/plugin_base.h>
#include <pluginlib_turtle_move/twist.h>
#include <pluginlib_turtle_move/follow.h>

//PLUGINLIB_EXPORT_CLASS(move_plugins::FolowListener, move_base::BaseListener)
//PLUGINLIB_EXPORT_CLASS(move_plugins::TwistTimerListener, move_base::BaseListener)

namespace move_base {
	// BaseListener::BaseListener(/*TurtleSine *p,*/ double dur, ros::NodeHandle &n) : 
	// 	nh(n),
	// 	//parent(p), 
	// 	duration(dur), 
	// 	timer(nh.createTimer(ros::Duration(dur), &BaseListener::timerCallback, this)),
	// 	pubsine(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000)){}

	void BaseListener::start(){
		timer = nh.createTimer(ros::Duration(duration), &BaseListener::timerCallback, this);
		pubsine = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	}
}

