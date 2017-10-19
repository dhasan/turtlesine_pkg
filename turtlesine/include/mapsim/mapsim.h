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

		class TimerBaseListener
		{
			
			public:
				
				TimerBaseListener() = default;
				 
				virtual ~TimerBaseListener() = default;
				TimerBaseListener(const TimerBaseListener&) = delete;
				TimerBaseListener& operator= (const TimerBaseListener&) = delete; //copy assignment operator
				TimerBaseListener(TimerBaseListener&&) =delete; //move constructor
				TimerBaseListener& operator=(TimerBaseListener&&) = delete; //move assignment operator
				double getDuration() { return duration;}
				virtual void timerCallback(const ros::TimerEvent& e) = 0;	

			protected:
				TimerBaseListener(Mapsim *p, double dur, ros::NodeHandle &n);
				ros::NodeHandle &nh;
				double duration;
				Mapsim *parent;
			
			private:
				ros::Timer timer;
				
			

		};

		class MapTimerListener : public TimerBaseListener{
		
			public:
				MapTimerListener(Mapsim *p, double dur, ros::NodeHandle &n);
				MapTimerListener() = default;
				
				virtual ~MapTimerListener() = default;
				MapTimerListener(const MapTimerListener&) = delete;
				MapTimerListener& operator= (const MapTimerListener&) = delete; //copy assignment operator
				MapTimerListener(MapTimerListener&&) =delete; //move constructor
				MapTimerListener& operator=(MapTimerListener&&) = delete; //move assignment operator

				virtual void timerCallback(const ros::TimerEvent& e);


			
		};


		ros::NodeHandle& nh;

		//point cloud publishter
		ros::Publisher pcpub;

		sensor_msgs::PointCloud pc;

		TimerBaseListener *timerlistener;
		

	

	};


/*}*/

#endif