#include <pluginlib/class_list_macros.h>
#include <pluginlib_turtle_move/plugin_base.h>
//#include <pluginlib_turtle_move/twist.h>
//#include <pluginlib_turtle_move/follow.h>

namespace move_base {

	void BaseListener::start(){
		timer = nh.createTimer(ros::Duration(duration), &BaseListener::timerCallback, this);
		pubsine = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	}
}

