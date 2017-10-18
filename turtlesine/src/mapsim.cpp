#include "mapsim.h"
#include "nodelet/loader.h"

#include <geometry_msgs/Point32.h>

/*PLUGINLIB_EXPORT_CLASS(task1_pkg::Mapsim, nodelet::Nodelet)

/* namespace task1_pkg {*/

	const std::string Mapsim::node_name = "mapsim";

	Mapsim::MapTimerListener::MapTimerListener(Mapsim *p) : TimerBaseListener(p){}
	Mapsim::TimerBaseListener::TimerBaseListener(Mapsim *p) : parent(p){}

	Mapsim::Mapsim(ros::NodeHandle &n): nh(n), 
		timerlistener(this), timer(nh.createTimer(ros::Duration(TIME_DT), &MapTimerListener::timerCallback, &timerlistener)),
		pcpub(nh.advertise<sensor_msgs::PointCloud>("walls", 1000)){

		

	}

	Mapsim::Mapsim(): nh(getPrivateNodeHandle()){

	}

	void Mapsim::onInit(){

	}

	void Mapsim::MapTimerListener::timerCallback(const ros::TimerEvent& e){

		geometry_msgs::Point32 point;

		parent->pc.header.frame_id = "map";
		
		
  		for (int i = 0; i<100; i++){
  			
  			
  			point.x = i * 11.08/100;
  			point.y = 0;
  			point.z = 0;
  			parent->pc.points.push_back(point);
  			point.x = i * 11.08/100;
  			point.y = 11.08;
  			point.z = 0;
  			parent->pc.points.push_back(point);
  			point.x = 0;
  			point.y = i * 11.08/100;
  			point.z = 0;
  			parent->pc.points.push_back(point);
  			point.x = 11.08;
  			point.y = i * 11.08/100;
  			point.z = 0;
  			parent->pc.points.push_back(point);
  		}

  		parent->pcpub.publish(parent->pc);

  		parent->pc.points.resize(0);
	}
	

/* } */


int main(int argc, char **argv)
{
	ros::init(argc, argv, /* task1_pkg::*/Mapsim::node_name);
#if 1
	
	ros::NodeHandle n("~");
	/*task1_pkg::*/Mapsim ts(n);

#else
	nodelet::Loader nodelet;
  	nodelet::M_string remap(ros::names::getRemappings());
  	nodelet::V_string nargv;
  	std::string nodelet_name = ros::this_node::getName();
  	nodelet.load(nodelet_name, "task1_pkg/Mapsim", remap, nargv);
#endif

	ros::spin();

	return 0;
}
