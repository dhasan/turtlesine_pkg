
#include "turtlesine.h"
#include <sstream>
#include <boost/tokenizer.hpp>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Spawn.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include "nodelet/loader.h"
#include <turtlesim/Kill.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

PLUGINLIB_EXPORT_CLASS(task1_pkg::TurtleSine, nodelet::Nodelet)

namespace task1_pkg {

	const std::string TurtleSine::node_name = "turtlesine";

	void TurtleSine::simWait(TurtleSine *obj){
		

		std::unique_lock<std::mutex> lck(obj->mtx);
		while (!ros::service::exists("/sim/spawn", true))
		{
			
			ROS_WARN("Wait for turtlesim_node.");
			ros::service::waitForService("/sim/spawn", 15);
			
		}
		
		obj->cv.notify_all();
	}

	TurtleSine::~TurtleSine(){
		turtlesim::Kill kill_turtle;

		kill_turtle.request.name = turtlename;
		kill.call(kill_turtle);
		ROS_ERROR("KILLING THE TURTLE");

	}

	TurtleSine::PoseListener::PoseListener(TurtleSine *p) : parent(p){}

	void TurtleSine::PoseListener::poseCallback(const turtlesim::PoseConstPtr& msg){
		//TODO remove this

		static tf::TransformBroadcaster br;
 		tf::Transform transform;
  		transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  		tf::Quaternion q;
  		q.setRPY(0, 0, msg->theta);
  		transform.setRotation(q);
  		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", parent->turtlename+std::string("_pose")));

	}
	

	TurtleSine::TurtleSine() : nh(getPrivateNodeHandle()){}
	
	TurtleSine::TurtleSine(ros::NodeHandle &n) : nh(n) ,dt(ros::Time().toSec()), pubsine(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000)),
		clienttelep(nh.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute")), 
		spawn(nh.serviceClient<turtlesim::Spawn>("/sim/spawn")),
		timer(nh.createTimer(ros::Duration(TIME_DT), boost::bind(&TurtleSine::timerCallback, this))),
		odompub(nh.advertise<nav_msgs::Odometry>("odometry", 1000)),
		kill(nh.serviceClient<turtlesim::Kill>("/sim/kill")),
		poselistener(this),
		posesub(nh.subscribe("pose", 10, &PoseListener::poseCallback, &poselistener)),
		lastpose(3, .0)
		
	{
		turtlesim::TeleportAbsolute telep;
		

		
		/*
			Apparently params can't be float, only (str|int|double|bool|yaml), so use temporary vars, the other option is yaml with single vector parameter...
		*/
		double tx,ty,ttheta;
		nh.getParam("initial_x", tx);
		nh.getParam("initial_y", ty);
		nh.getParam("initial_theta", ttheta);

		auto ns = ros::this_node::getNamespace();
		boost::tokenizer<boost::char_separator<char>> tokens(ns, boost::char_separator<char>("/"));
		std::vector<std::string> result(tokens.begin(), tokens.end());

    	//TODO: empty vector check
		turtlename = result.at(result.size() - 1);
		std::cout << "NAMESPACE : " <<ros::this_node::getNamespace()<< " "<<turtlename<<std::endl;
	
		/*
			Since both nodes are starting at the same from launcher sometimes turtlesine node starts before
			turtlesim_node, so we need to wait until turtlesim_node appear to use spawn service
		*/
		thread = std::thread(TurtleSine::simWait, this);
		std::unique_lock<std::mutex> lck(mtx);
  		cv.wait(lck);

  		telep.request.x = tx;
		telep.request.y = ty;
		telep.request.theta = ttheta;

		ros::service::waitForService("teleport_absolute", 150);
  		if (turtlename.compare("turtle1")){
			//Check if we have the turtle, if not create it first before teleport (in all other case except turtle1)
			if (!ros::service::exists("teleport_absolute", true)){
				turtlesim::Spawn spawn_turtle;
			
				spawn_turtle.request.x = tx;
				spawn_turtle.request.y = ty;
				spawn_turtle.request.theta = ttheta;
				spawn_turtle.request.name = turtlename;
				spawn.call(spawn_turtle);
				std::cout << "spawn NAMESPACE: "<< ns<<" NAME: " <<turtlename<<std::endl;
			}
		}else
			clienttelep.call(telep);

		

	}
	
	
	void TurtleSine::timerCallback(TurtleSine *obj)
	{
		geometry_msgs::Twist twist;
		

		double as,ls;
		obj->nh.getParam("a_speed", as);
		obj->nh.getParam("l_speed", ls);

		twist.linear.x = as;
		twist.linear.y = 0;
		twist.linear.z = 0;
	
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = ls;
	
		if ((obj->count % 80) > 40){
	  		twist.angular.z *= -1;
	  	}
	
	  	obj->pubsine.publish(twist);
		
		obj->poseCalculate(twist);
	
	  
		obj->count++;
	}
	
	void TurtleSine::poseCalculate(const geometry_msgs::Twist &twist){
	
		/* Odometry formula*/
		nav_msgs::Odometry odom;

		static tf::TransformBroadcaster br;
  		tf::Transform transform_bs, transform;
  		ros::Time time_now = ros::Time::now();
  		if (dt==0){
  			dt = time_now.toSec();
  			return;
  		}
		
		dt = time_now.toSec() - dt;
		//dt = TIME_DT;
		std::cout << "TIME: " << dt << std::endl;

		double vx = twist.linear.x;
		double vy = twist.linear.y; 
		double th = twist.angular.z;
	
		double thi = lastpose.at(POSE_THETA); //initial angle
	
		double delta_x = (vx * cos(thi) - vy * sin(thi)) * dt;
		double delta_y = (vx * sin(thi) + vy * cos(thi)) * dt;
		double delta_th = th * dt;
	
		lastpose.at(POSE_X) += delta_x;
		lastpose.at(POSE_Y) += delta_y;
		lastpose.at(POSE_THETA) += delta_th;

		double tx,ty,ttheta;
		nh.getParam("initial_x", tx);
		nh.getParam("initial_y", ty);
		nh.getParam("initial_theta", ttheta);
	
		ROS_INFO("Calculated pose x y: %f %f", lastpose.at(0), lastpose.at(1));
	
		//q.setEuler(lastpose.at(POSE_THETA), 0, 0);
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, lastpose.at(POSE_THETA));

		odom.header.stamp = time_now;
		
		odom.header.frame_id = turtlename + std::string("_odometry");
		odom.child_frame_id = turtlename + std::string("_base_link");

	
		odom.pose.pose.position.x = lastpose.at(POSE_X);
		odom.pose.pose.position.y = lastpose.at(POSE_Y);
		odom.pose.pose.position.z = 0;

		odom.pose.pose.orientation.x = q[0];
		odom.pose.pose.orientation.y = q[1];
		odom.pose.pose.orientation.z = q[2];
		odom.pose.pose.orientation.w = q[3];

		odom.twist.twist = twist;
		

  		transform_bs.setOrigin( tf::Vector3(lastpose.at(POSE_X), lastpose.at(POSE_Y), 0.0) );
  		transform_bs.setRotation(q);  		


  		br.sendTransform(tf::StampedTransform(transform_bs, time_now, turtlename + std::string("_odometry"), turtlename + std::string("_base_link")));

  		//Initial coordinates
  		transform.setOrigin( tf::Vector3(tx, ty, 0.0) );
  		tf::Quaternion q3;
  		q3.setRPY(0, 0, ttheta);
  		transform.setRotation(q3);
  		br.sendTransform(tf::StampedTransform(transform, time_now, "world", turtlename + std::string("_odometry")));


  		odompub.publish(odom);
  		dt = time_now.toSec();
  		
	}
	
	void TurtleSine::onInit()
	{
		NODELET_DEBUG("Initializing nodelet...");
		pubsine = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
		clienttelep = nh.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute"); 
		spawn = nh.serviceClient<turtlesim::Spawn>("spawn"); 
		timer = nh.createTimer(ros::Duration(TIME_DT), boost::bind(&TurtleSine::timerCallback, this));
		odompub = nh.advertise<nav_msgs::Odometry>("odometry", 1000);
		kill = nh.serviceClient<turtlesim::Kill>("/task1/kill");
		lastpose.resize(3);
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, task1_pkg::TurtleSine::node_name);
#if 1
	
	ros::NodeHandle n("~");
	task1_pkg::TurtleSine ts(n);

#else
	nodelet::Loader nodelet;
  	nodelet::M_string remap(ros::names::getRemappings());
  	nodelet::V_string nargv;
  	std::string nodelet_name = ros::this_node::getName();
  	nodelet.load(nodelet_name, "task1_pkg/TurtleSine", remap, nargv);
#endif

	ros::spin();

	return 0;
}
