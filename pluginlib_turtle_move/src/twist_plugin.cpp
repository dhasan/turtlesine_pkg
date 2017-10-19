#include <pluginlib/class_list_macros.h>
#include <pluginlib_turtle_move/plugin_base.h>
#include <pluginlib_turtle_move/twist.h>

PLUGINLIB_EXPORT_CLASS(move_plugins::TwistTimerListener, move_base::BaseListener)


namespace move_plugins {
	
	TwistTimerListener::TwistTimerListener():BaseListener(){}

	void TwistTimerListener::initialize(double dur, ros::NodeHandle &n){
		nh = n;
		this->duration = dur;
	}


	void TwistTimerListener::timerCallback(const ros::TimerEvent& e)
	{
		geometry_msgs::Twist twist;
		lastevent = e;
		double as,ls;
		nh.getParam("a_speed", as);
		nh.getParam("l_speed", ls);

		ros::TimerEvent odomeevent;
		
		//odomeevent.current_real = e.current_real;
		//odomeevent.last_real = parent->odomtimerlistener->lastevent.last_real;
		//parent->odomtimerlistener->timerCallback(odomeevent);

		twist.linear.x = as;
		twist.linear.y = 0;
		twist.linear.z = 0;
	
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = ls;
		
		if ((count % 3) == 0){
			twist.angular.z *= -1;
		}

		pubsine.publish(twist);
		//parent->odomtimerlistener->latesttwist = twist;
		count++;
	  	
	}

}