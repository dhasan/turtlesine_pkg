#ifndef _MAPSIM_H_
#define _MAPSIM_H_

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>
#include <cmath>

/* namespace task1_pkg {*/

	class Lasersim : public nodelet::Nodelet
	{


	public:
		class WallsListener
		{
			public:
  				void wallsCallback(const sensor_msgs::PointCloudConstPtr &msg);

  				WallsListener(Lasersim *p);
  				WallsListener() = default;
  				 
  				virtual ~WallsListener() = default;
  				WallsListener(const WallsListener&) = delete;
				WallsListener& operator= (const WallsListener&) = delete; //copy assignment operator
				WallsListener(WallsListener&&) =delete; //move constructor
				WallsListener& operator=(WallsListener&&) = delete; //move assignment operator

			private:
				Lasersim *parent;
			

		};

		class TurtleListener
		{
			public:
  				void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

  				TurtleListener(Lasersim *p);
  				TurtleListener() = default;
  				 
  				virtual ~TurtleListener() = default;
  				TurtleListener& operator= (const TurtleListener&) = delete; //copy assignment operator

			private:
				Lasersim *parent;
			

		};

		virtual void onInit();

		static void timerCallback(Lasersim *obj);

		//Transformation from cadestrian coordinates to polar
		void toPolarPC(const sensor_msgs::PointCloud &in, std::vector<double> &alpha, std::vector<double> &r) const;
		void toPolarP(const geometry_msgs::Point &in, double &alpha, double &r) const;


		static const std::string node_name;
		Lasersim(ros::NodeHandle &n);

		//Nodelet is using this constructor, so keeping it non-default.....
		Lasersim();
		virtual ~Lasersim() = default;
  		Lasersim(const Lasersim&) = delete;
		Lasersim& operator= (const Lasersim&) = delete; //copy assignment operator
		Lasersim(Lasersim&&) = delete; //move constructor
		Lasersim& operator=(Lasersim&&) = delete; //move assignment operator

	private:
		void resetlaser();

		ros::NodeHandle& nh;

		//Name of the turtle
		std::string turtlename;

		//Walls subscriber
		ros::Subscriber wallssub;

		//walls listeners
		WallsListener wallslistener;
		
		tf::TransformListener tflistener;

		//LaserScan publisher
		ros::Publisher laserscan;

		//Laser scan
		sensor_msgs::LaserScan ls;

		//Time discrete
		double dt;

		//TODO: use list for subs and listeners!!!
		ros::Subscriber turtlessub[10];
		TurtleListener turtlelisteners[10];

		//laser message measurements 
		int measurments;
		unsigned int flags;
		int turtles_cnt;

	};


/*}*/

#endif