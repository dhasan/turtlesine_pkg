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

#define 	TIME_DT 	(0.9)

#define 	TIME_DTODOM 	(1.0/100)

//#define M_PI 3.14159265358979323846

namespace task1_pkg {
	class TurtleSine : public nodelet::Nodelet
	{


	public:
		virtual void onInit();

		static const std::string node_name;
		TurtleSine(ros::NodeHandle &n);

		//Nodelet is using this constructor, so keeping it non-default.....
		TurtleSine();
		virtual ~TurtleSine();
  		TurtleSine(const TurtleSine&) = delete;
		TurtleSine& operator= (const TurtleSine&) = delete; //copy assignment operator
		TurtleSine(TurtleSine&&) = delete; //move constructor
		TurtleSine& operator=(TurtleSine&&) = delete; //move assignment operator

	private:

		class PoseListener
		{
			public:
  				void poseCallback(const turtlesim::PoseConstPtr& msg);

  				PoseListener(TurtleSine *p);
  				PoseListener() = default;
  				 
  				virtual ~PoseListener() = default;
  				PoseListener(const PoseListener&) = delete;
				PoseListener& operator= (const PoseListener&) = delete; //copy assignment operator
				PoseListener(PoseListener&&) =delete; //move constructor
				PoseListener& operator=(PoseListener&&) = delete; //move assignment operator

			private:
				TurtleSine *parent;
			

		};

		class TimerBaseListener
		{
			
  			public:
  				
  				TimerBaseListener() = default;
  				 
  				virtual ~TimerBaseListener() = default;
  				TimerBaseListener(const TimerBaseListener&) = delete;
				TimerBaseListener& operator= (const TimerBaseListener&) = delete; //copy assignment operator
				TimerBaseListener(TimerBaseListener&&) =delete; //move constructor
				TimerBaseListener& operator=(TimerBaseListener&&) = delete; //move assignment operator

			protected:
				TimerBaseListener(TurtleSine *p);
  				virtual void timerCallback(const ros::TimerEvent& e) = 0;
				TurtleSine *parent;
				ros::TimerEvent lastevent;
			

		};

		class TwistTimerListener : protected TimerBaseListener{
		
			public:
				TwistTimerListener(TurtleSine *p);
				TwistTimerListener() = default;
  				 
  				virtual ~TwistTimerListener() = default;
  				TwistTimerListener(const TwistTimerListener&) = delete;
				TwistTimerListener& operator= (const TwistTimerListener&) = delete; //copy assignment operator
				TwistTimerListener(TwistTimerListener&&) =delete; //move constructor
				TwistTimerListener& operator=(TwistTimerListener&&) = delete; //move assignment operator

				virtual void timerCallback(const ros::TimerEvent& e);

			
		};

		class OdomTimerListener : protected TimerBaseListener{
		
			public:
				OdomTimerListener(TurtleSine *p);
				OdomTimerListener() = default;
  				 
  				virtual ~OdomTimerListener() = default;
  				OdomTimerListener(const OdomTimerListener&) = delete;
				OdomTimerListener& operator= (const OdomTimerListener&) = delete; //copy assignment operator
				OdomTimerListener(OdomTimerListener&&) =delete; //move constructor
				OdomTimerListener& operator=(OdomTimerListener&&) = delete; //move assignment operator

				virtual void timerCallback(const ros::TimerEvent& e);

			
		};

		static void odomCallback(TurtleSine *obj);

		//Conditional variable callback 
		static void simWait(TurtleSine *obj);

		void toPolar(sensor_msgs::PointCloud &in, std::vector<double> &alpha, std::vector<double> &r) const;

		//Time discrete of the turtle
		double dt;

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

		pthread_mutex_t var=PTHREAD_MUTEX_INITIALIZER;

		//Turtle main timer for sending twists
		TwistTimerListener twisttimerlistener;
		ros::Timer twisttimer;
		

		//Odometry integration timer
		OdomTimerListener odomtimerlistener;
		ros::Timer odomtimer;

		//Letest twist
		geometry_msgs::Twist latesttwist;

		//Turtle odometry publisher
		ros::Publisher odompub;

		//Turtle kill service client
		ros::ServiceClient kill;

		//Depricated
		ros::Subscriber posesub;
		
		//Odometry vector storage
		std::vector<double> lastpose;

		//Name of the turtle
		std::string turtlename;

		std::mutex mtx;
		std::condition_variable cv;
		std::thread thread;

		//Pose listener
		PoseListener poselistener;

		tf::TransformListener tflistener;

		int count,odomcount;
	
	};


}

#endif