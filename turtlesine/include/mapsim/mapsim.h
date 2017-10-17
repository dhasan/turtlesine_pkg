#ifndef _MAPSIM_H_
#define _MAPSIM_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>

#define 	TIME_DT 	(1.0/30.0)


/* namespace task1_pkg {*/

	class Mapsim : public nodelet::Nodelet
	{


	public:
		virtual void onInit();

		static void timerCallback(Mapsim *obj);

		static const std::string node_name;
		Mapsim(ros::NodeHandle &n);

		//Nodelet is using this constructor, so keeping it non-default.....
		Mapsim();
		virtual ~Mapsim() = default;
  		Mapsim(const Mapsim&) = delete;
		Mapsim& operator= (const Mapsim&) = delete; //copy assignment operator
		Mapsim(Mapsim&&) = delete; //move constructor
		Mapsim& operator=(Mapsim&&) = delete; //move assignment operator

	private:
		ros::NodeHandle& nh;

		//point cloud publishter
		ros::Publisher pcpub;

		sensor_msgs::PointCloud pc;

		ros::Timer timer;

	

	};


/*}*/

#endif