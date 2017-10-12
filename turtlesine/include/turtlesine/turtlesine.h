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

#define 	TIME_DT 	(1.0/1.0)
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
  				 
  				virtual ~PoseListener() = default;
  				PoseListener(const PoseListener&) = delete;
				PoseListener& operator= (const PoseListener&) = delete; //copy assignment operator
				PoseListener(PoseListener&&) =delete; //move constructor
				PoseListener& operator=(PoseListener&&) = delete; //move assignment operator

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

		int count;
		double dt;
		

		static void timerCallback(TurtleSine *obj);
		static void simWait(TurtleSine *obj);

		void poseCalculate(const geometry_msgs::Twist &twist);
		//void poseCallback(const turtlesim::PoseConstPtr& msg);



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
	
	};


}

#endif