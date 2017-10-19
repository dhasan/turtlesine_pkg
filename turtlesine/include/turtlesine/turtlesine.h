#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nodelet/nodelet.h>
#include <turtlesim/Pose.h>
#include <tf/transform_listener.h>
#include <cmath>

#include <thread>             
#include <mutex>              
#include <condition_variable> 

#define		POSE_X		(0)
#define		POSE_Y		(1)
#define		POSE_THETA	(2)

#define 	TIME_DT_TWIST 	(0.9)
#define 	TIME_DT_ODOM 	(0.01)
#define 	TIME_DT_FOLOW	(0.1)

#undef 		SIMULATE_GPS

namespace task1_pkg {
	class TurtleSine : public nodelet::Nodelet
	{


	public:
		
		static const std::string node_name;

		virtual void onInit();

		void poseCallback(const turtlesim::PoseConstPtr& msg);

		TurtleSine(ros::NodeHandle &n);

		//Nodelet is using this constructor, so keeping it non-default.....
		TurtleSine();
		virtual ~TurtleSine();
		TurtleSine(const TurtleSine&) = delete;
		TurtleSine& operator= (const TurtleSine&) = delete; //copy assignment operator
		TurtleSine(TurtleSine&&) = delete; //move constructor
		TurtleSine& operator=(TurtleSine&&) = delete; //move assignment operator

	private:

		class BaseListener
		{
			
			public:
				
				BaseListener() = delete;
				 
				virtual ~BaseListener() = default;
				BaseListener(const BaseListener&) = delete;
				BaseListener& operator= (const BaseListener&) = delete; //copy assignment operator
				BaseListener(BaseListener&&) =delete; //move constructor
				BaseListener& operator=(BaseListener&&) = delete; //move assignment operator
				double getDuration() { return duration;}
				virtual void timerCallback(const ros::TimerEvent& e) = 0;	

				ros::TimerEvent lastevent; //TODO use setter getter
			protected:
				BaseListener(TurtleSine *p, double dur, ros::NodeHandle &n);
				ros::NodeHandle &nh;	

				double duration;
				TurtleSine *parent;
				

			private:
				//ROS timer
				ros::Timer timer;
			

		};

		class TwistTimerListener : public BaseListener{
		
			public:
				TwistTimerListener(TurtleSine *p, double dur, ros::NodeHandle &nh);
				TwistTimerListener() = delete;
				 
				virtual ~TwistTimerListener() = default;
				TwistTimerListener(const TwistTimerListener&) = delete;
				TwistTimerListener& operator= (const TwistTimerListener&) = delete; //copy assignment operator
				TwistTimerListener(TwistTimerListener&&) =delete; //move constructor
				TwistTimerListener& operator=(TwistTimerListener&&) = delete; //move assignment operator

				virtual void timerCallback(const ros::TimerEvent& e);
			
			private:
				//counter of twist commands, 
				int count;
				
	
		};

		class FolowListener : public BaseListener{
		
			public:
				FolowListener(TurtleSine *p, double dur, ros::NodeHandle &nh, std::string topicname);
				FolowListener() = delete;
				 
				virtual ~FolowListener() = default;
				FolowListener(const FolowListener&) = delete;
				FolowListener& operator= (const FolowListener&) = delete; //copy assignment operator
				FolowListener(FolowListener&&) =delete; //move constructor
				FolowListener& operator=(FolowListener&&) = delete; //move assignment operator

				virtual void timerCallback(const ros::TimerEvent& e);

				void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

			private:
				ros::Subscriber folowposesub;
				std::string follow_frame;

			
		};

		class OdomTimerListener : public BaseListener{
		
			public:
				OdomTimerListener(TurtleSine *p, double dur, ros::NodeHandle &nh);
				OdomTimerListener() = delete;
				 
				virtual ~OdomTimerListener() = default;
				OdomTimerListener(const OdomTimerListener&) = delete;
				OdomTimerListener& operator= (const OdomTimerListener&) = delete; //copy assignment operator
				OdomTimerListener(OdomTimerListener&&) =delete; //move constructor
				OdomTimerListener& operator=(OdomTimerListener&&) = delete; //move assignment operator

				virtual void timerCallback(const ros::TimerEvent& e);
				void setLastPose(double x, double y, double theta){lastpose.at(0) = x, lastpose.at(1) = y, lastpose.at(2) = theta;}
				void odominittransform();
				int getCount() {return count;}
				
				geometry_msgs::Twist latesttwist; //TODO: setter gettor
			private:
 				//Odometry vector storage
				std::vector<double> lastpose;

				//counter of odometry calculations
				int count;

			
		};

		//Conditional variable callback 
		static void simWait(TurtleSine *obj);

		void toPolarP(const geometry_msgs::Point &in, double &alpha, double &r) const;

		//ROS node handle
		ros::NodeHandle& nh;
		
		//Velocities command publisher and turtlesim pose 
		ros::Publisher pubsine;
		ros::Publisher posepub;

		tf::Transform gpstransform;

		//Turtle teleportation service client
		ros::ServiceClient clienttelep;

		//Turtle spawn service client
		ros::ServiceClient spawn;

		//Turtle main timer for sending twists
		BaseListener *movelistener;
		
		//Odometry integration timer
		OdomTimerListener *odomtimerlistener;

		//Turtle odometry publisher
		ros::Publisher odompub;

		//Turtle kill service client
		ros::ServiceClient kill;

		//Pose subscriber from turtlesim
		ros::Subscriber posesub;
	
		//Name of the turtle
		std::string turtlename;

		std::mutex mtx;
		std::condition_variable cv;
		std::thread thread;

		tf::TransformListener tflistener;

	};


}

#endif