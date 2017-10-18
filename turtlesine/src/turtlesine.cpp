
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

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <math.h>

#include <pthread.h>

PLUGINLIB_EXPORT_CLASS(task1_pkg::TurtleSine, nodelet::Nodelet)

namespace task1_pkg {

	const std::string TurtleSine::node_name = "turtlesine";

	void TurtleSine::simWait(TurtleSine *obj){

		while (!ros::service::exists("~spawn", true))
		{
			
			ROS_WARN("Wait for turtlesim_node.");
			ros::service::waitForService("~spawn", 15);
			
		}
		
		obj->cv.notify_all();
	}

	TurtleSine::~TurtleSine(){
		turtlesim::Kill kill_turtle;

		kill_turtle.request.name = turtlename;
		kill.call(kill_turtle);
		ROS_ERROR("KILLING THE TURTLE");

	}

	TurtleSine::TwistTimerListener::TwistTimerListener(TurtleSine *p) : TimerBaseListener(p){}
	TurtleSine::OdomTimerListener::OdomTimerListener(TurtleSine *p) : TimerBaseListener(p){}
	TurtleSine::TimerBaseListener::TimerBaseListener(TurtleSine *p) : parent(p){}

	TurtleSine::PoseListener::PoseListener(TurtleSine *p) : parent(p){}

	void TurtleSine::PoseListener::poseCallback(const turtlesim::PoseConstPtr& msg){
		
		static tf::TransformBroadcaster br;
#if 1
 		if (parent->odomcount % 50 != 0)
			return;
#endif
 		pthread_mutex_lock(&parent->var);

  		parent->gpstransform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  		tf::Quaternion q;
  		q.setRPY(0, 0, msg->theta);
  		parent->gpstransform.setRotation(q);
  		ros::Time time_now = ros::Time::now();
  		
  		br.sendTransform(tf::StampedTransform(parent->gpstransform, time_now, "map", parent->turtlename+std::string("_odom")));


  		/*
			Reset the odometry, with gps values, to discard the accumulated error
			The idea is to replace pose values from turtlesim with real GPS data that comes on every 1 second (very rear)
  		*/
  		
 		parent->lastpose.at(POSE_X) = 0;
  		parent->lastpose.at(POSE_Y) = 0;
  		parent->lastpose.at(POSE_THETA) = 0;

  		br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), time_now, parent->turtlename+std::string("_odom"), parent->turtlename+std::string("_base_link")));
  		

  		

  		pthread_mutex_unlock(&parent->var);

	}



	TurtleSine::TurtleSine() : nh(getPrivateNodeHandle()){}
	
	TurtleSine::TurtleSine(ros::NodeHandle &n) : nh(n) , dt(ros::Time().toSec()), pubsine(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000)),
		clienttelep(nh.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute")), 
		spawn(nh.serviceClient<turtlesim::Spawn>("spawn")),
		//twisttimerlistener( new TwistTimerListener(this)), twisttimer(nh.createTimer(ros::Duration(TIME_DT), &TwistTimerListener::timerCallback, twisttimerlistener)),
		twisttimerlistener(this), twisttimer(nh.createTimer(ros::Duration(TIME_DT), &TwistTimerListener::timerCallback, &twisttimerlistener)),
		odomtimerlistener(this), odomtimer(nh.createTimer(ros::Duration(TIME_DTODOM), &OdomTimerListener::timerCallback, &odomtimerlistener)),
		odompub(nh.advertise<nav_msgs::Odometry>("odometry", 1000)),
		posepub(nh.advertise<geometry_msgs::PoseStamped>("stampedpose", 1000)),
		kill(nh.serviceClient<turtlesim::Kill>("kill")),
		poselistener(this),
		posesub(nh.subscribe("pose", 10, &PoseListener::poseCallback, &poselistener)),
		lastpose(3, .0)
		
	{
		turtlesim::TeleportAbsolute telep;
		static tf::TransformBroadcaster br;
  		//tf::Transform transform;
		
		double tx,ty,ttheta;
		nh.getParam("initial_x", tx);
		nh.getParam("initial_y", ty);
		nh.getParam("initial_theta", ttheta);

		auto ns = nh.getNamespace();
		auto pos = ns.find_last_of('/');
  		turtlename = ns.substr(pos + 1);

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

		ros::service::waitForService("~teleport_absolute", 150);
  		if (turtlename.compare("turtle1")){
			//Check if we have the turtle, if not create it first before teleport (in all other case except turtle1)
			if (!ros::service::exists("~teleport_absolute", true)){
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
	
		gpstransform.setOrigin( tf::Vector3(tx, ty, 0.0) );
  		tf::Quaternion q3;
  		q3.setRPY(0, 0, ttheta);
  		gpstransform.setRotation(q3);
  		br.sendTransform(tf::StampedTransform(gpstransform, ros::Time::now(), "map", turtlename + std::string("_odom")));

	}
	
	
	void TurtleSine::TwistTimerListener::timerCallback(const ros::TimerEvent& e)
	{
		geometry_msgs::Twist twist;

		double as,ls;
		parent->nh.getParam("a_speed", as);
		parent->nh.getParam("l_speed", ls);

		//pthread_mutex_lock(&parent->var);
		//parent->poseCalculate(parent->latesttwist);

		twist.linear.x = as;
		twist.linear.y = 0;
		twist.linear.z = 0;
	
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = ls;
	
		if ((parent->count % 3) == 0){
	  		twist.angular.z *= -1;
	  	}
	

	  	parent->pubsine.publish(twist);

	  	parent->latesttwist = twist;
	
	  	parent->count++;

	  //	pthread_mutex_unlock(&parent->var);
	  	
	}

	void TurtleSine::OdomTimerListener::timerCallback(const ros::TimerEvent& e){
		//pthread_mutex_lock(&obj->var);

		//obj->poseCalculate(parent->latesttwist);
		
	//	pthread_mutex_unlock(&obj->var);

		/* Odometry formula*/
		nav_msgs::Odometry odom;

		geometry_msgs::PoseStamped ps;
		

		static tf::TransformBroadcaster br;
  		tf::Transform transform_bs;
  		tf::StampedTransform maptransform;
  		ros::Time time_now = ros::Time::now();
  		if (parent->dt==0){
  			parent->dt = time_now.toSec();
  			return;
  		}
		
		parent->dt = time_now.toSec() - parent->dt;

		//std::cout << "TIME: " << dt << std::endl;

		double vx = parent->latesttwist.linear.x;
		double vy = parent->latesttwist.linear.y; 
		double th = parent->latesttwist.angular.z;
	
		double thi = parent->lastpose.at(POSE_THETA); //initial angle
	
		double delta_x = (vx * cos(thi) - vy * sin(thi)) * parent->dt;
		double delta_y = (vx * sin(thi) + vy * cos(thi)) * parent->dt;
		double delta_th = th * parent->dt;
	
		parent->lastpose.at(POSE_X) += delta_x;
		parent->lastpose.at(POSE_Y) += delta_y;
		parent->lastpose.at(POSE_THETA) += delta_th;

		double tx,ty,ttheta;
		parent->nh.getParam("initial_x", tx);
		parent->nh.getParam("initial_y", ty);
		parent->nh.getParam("initial_theta", ttheta);
	
		//ROS_INFO("Calculated pose x y: %f %f", lastpose.at(0), lastpose.at(1));

		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, parent->lastpose.at(POSE_THETA));

		odom.header.stamp = time_now;
		
		odom.header.frame_id = parent->turtlename + std::string("_odom");
		odom.child_frame_id = parent->turtlename + std::string("_base_link");

	
		odom.pose.pose.position.x = parent->lastpose.at(POSE_X);
		odom.pose.pose.position.y = parent->lastpose.at(POSE_Y);
		odom.pose.pose.position.z = 0;
		ps.pose.position = odom.pose.pose.position;

		odom.pose.pose.orientation.x = q[0];
		odom.pose.pose.orientation.y = q[1];
		odom.pose.pose.orientation.z = q[2];
		odom.pose.pose.orientation.w = q[3];
		ps.pose.orientation = odom.pose.pose.orientation;

		odom.twist.twist = parent->latesttwist;
		parent->odompub.publish(odom);

		ps.header.stamp = time_now;
		ps.header.frame_id = parent->turtlename + std::string("_base_link");

		parent->posepub.publish(ps);


		/*
			After we create separate thread we may uncomment this lines to send the transformation of odometry
		*/
		br.sendTransform(tf::StampedTransform(parent->gpstransform, time_now, "map", parent->turtlename+std::string("_odom")));
#if 1
  		transform_bs.setOrigin( tf::Vector3(parent->lastpose.at(POSE_X), parent->lastpose.at(POSE_Y), 0.0) );
  		transform_bs.setRotation(q);  		
  		br.sendTransform(tf::StampedTransform(transform_bs, time_now, parent->turtlename + std::string("_odom"), parent->turtlename + std::string("_base_link")));
#endif
  		parent->odomcount++;
  		parent->dt = time_now.toSec();
	}

	void TurtleSine::toPolar(sensor_msgs::PointCloud &in, std::vector<double> &alpha, std::vector<double> &r) const {

		for (int i=0;i<in.points.size();i++){
			r[i] = (sqrt((in.points[i].x * in.points[i].x)  + (in.points[i].y * in.points[i].y)));
			alpha[i] = (atan2(in.points[i].y, in.points[i].x));
		}

	}

	void TurtleSine::onInit()
	{

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
