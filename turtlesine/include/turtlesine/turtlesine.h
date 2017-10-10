#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nodelet/nodelet.h>

#include <thread>             // std::thread
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

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
		ros::NodeHandle& nh;
	
		ros::Publisher pubsine;
		ros::ServiceClient clienttelep;
		ros::ServiceClient spawn;
		ros::Timer timer;
		ros::Publisher odompub;
		ros::ServiceClient kill;
		
		std::vector<float> lastpose;
		std::string turtlename;

		std::mutex mtx;
		std::condition_variable cv;
		std::thread thread;
		

		static void timerCallback(TurtleSine *obj,double l, double a);
		static void simWait(TurtleSine *obj);

		void poseCalculate(const geometry_msgs::Twist &twist, unsigned int count);



	public:
		virtual void onInit();

		static const std::string node_name;
		TurtleSine(ros::NodeHandle &n);

		//Nodelet is using this constructor, so keeping it non-default.....
		TurtleSine(); 


		TurtleSine(const TurtleSine &obj) = delete;
		virtual ~TurtleSine();
	
	};


}

#endif