#ifndef PLUGINLIB_TURTLE_MOVE__PLUGIN_BASE_H_
#define PLUGINLIB_TURTLE_MOVE__PLUGIN_BASE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
namespace move_base {
	class BaseListener
	{
			
		public:
				
			BaseListener() = default;
				 
			virtual ~BaseListener() = default;
			BaseListener(const BaseListener&) = delete;
			BaseListener& operator= (const BaseListener&) = delete; //copy assignment operator
			BaseListener(BaseListener&&) =delete; //move constructor
			BaseListener& operator=(BaseListener&&) = delete; //move assignment operator
				
			double getDuration() { return duration;}
			virtual void timerCallback(const ros::TimerEvent& e) = 0;	
			virtual void initialize(double dur, ros::NodeHandle &n) = 0;
			void start();

			ros::TimerEvent lastevent;
		
		protected:
			ros::NodeHandle nh;
			double duration;
			ros::Publisher pubsine;
			
				

		private:
			//ROS timer
			ros::Timer timer;
			

		};
};
#endif
