#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nodelet/nodelet.h>
#include <turtlesim/Pose.h>

#include <thread>             
#include <mutex>              
#include <condition_variable> 

#define		POSE_X		(0)
#define		POSE_Y		(1)
#define		POSE_THETA	(2)

#define		INITIAL_X	(5.544445)
#define		INITIAL_Y	(5.544445)

#define 	RETRYS 		(50)

#define 	TIME_DT 	(1.0/1.3)
namespace task1_pkg {
	class TurtleSine : public nodelet::Nodelet
	{


	private:

		class PoseListener
		{
			private:
				TurtleSine *parent;
			public:
  				void poseCallback(const turtlesim::PoseConstPtr& msg);
  				PoseListener(TurtleSine *p);
  				PoseListener() = default;

  				PoseListener(const PoseListener &obj) = delete;
				PoseListener& operator=(PoseListener pl) = delete;
				PoseListener(PoseListener&&) = delete;

		};

		ros::NodeHandle& nh;
	
		ros::Publisher pubsine;
		ros::ServiceClient clienttelep;
		ros::ServiceClient spawn;
		ros::Timer timer;
		ros::Publisher odompub;
		ros::ServiceClient kill;

		ros::Subscriber posesub;
		
		std::vector<float> lastpose;
		std::string turtlename;

		std::mutex mtx;
		std::condition_variable cv;
		std::thread thread;

		PoseListener poselistener;
		

		static void timerCallback(TurtleSine *obj,double l, double a);
		static void simWait(TurtleSine *obj);

		void poseCalculate(const geometry_msgs::Twist &twist);
		//void poseCallback(const turtlesim::PoseConstPtr& msg);



	public:
		virtual void onInit();

		static const std::string node_name;
		TurtleSine(ros::NodeHandle &n);

		//Nodelet is using this constructor, so keeping it non-default.....
		TurtleSine();



		TurtleSine(const TurtleSine &obj) = delete;
		TurtleSine& operator=(TurtleSine ts) = delete;
		TurtleSine(TurtleSine&&) = delete;

		virtual ~TurtleSine();
	
	};


}

#endif