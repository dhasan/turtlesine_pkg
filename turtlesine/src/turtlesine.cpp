
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

		delete odomtimerlistener;
		delete movelistener;

	}

	TurtleSine::TwistTimerListener::TwistTimerListener(TurtleSine *p, double dur, ros::NodeHandle &nh) : BaseListener(p, dur, nh){}

	TurtleSine::FolowListener::FolowListener(TurtleSine *p, double dur, ros::NodeHandle &nh, std::string topicn) : BaseListener(p, dur, nh), follow_frame(topicn),
		folowposesub(nh.subscribe("/demo/turtletarget", 10, &FolowListener::poseCallback, this)){} //TODO: use relative path

	TurtleSine::OdomTimerListener::OdomTimerListener(TurtleSine *p, double dur, ros::NodeHandle &nh) : BaseListener(p, dur, nh), lastpose(3, .0){

	}

	void TurtleSine::OdomTimerListener::odominittransform(){
		static tf::TransformBroadcaster br;

		br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), parent->turtlename+std::string("_odom"), parent->turtlename+std::string("_base_link")));
	}
	TurtleSine::BaseListener::BaseListener(TurtleSine *p, double dur, ros::NodeHandle &n) : 
		nh(n),
		parent(p), 
		duration(dur), 
		timer(nh.createTimer(ros::Duration(dur), &BaseListener::timerCallback, this)){}

	void TurtleSine::poseCallback(const turtlesim::PoseConstPtr& msg){
		
		static tf::TransformBroadcaster br;

#ifdef SIMULATE_GPS
 		if (odomtimerlistener->getCount() % 50 != 0)
			return;
#endif

		gpstransform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, msg->theta);
		gpstransform.setRotation(q);
		ros::Time time_now = ros::Time::now();
		
		br.sendTransform(tf::StampedTransform(gpstransform, time_now, "map", turtlename+std::string("_odom")));


		/*
			Reset the odometry, with gps values, to discard the accumulated error
			The idea is to replace pose values from turtlesim with real GPS data that comes on every 1 second (very rear)
		*/
		odomtimerlistener->setLastPose(0,0,0);
		br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), time_now, turtlename+std::string("_odom"), turtlename+std::string("_base_link")));

	}

	TurtleSine::TurtleSine() : nh(getPrivateNodeHandle()){}
	
	TurtleSine::TurtleSine(ros::NodeHandle &n) : nh(n) ,
		pubsine(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000)),
		clienttelep(nh.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute")), 
		spawn(nh.serviceClient<turtlesim::Spawn>("spawn")),
		odomtimerlistener(new OdomTimerListener(this, TIME_DT_ODOM, nh)), 
		odompub(nh.advertise<nav_msgs::Odometry>("odometry", 1000)),
		posepub(nh.advertise<geometry_msgs::PoseStamped>("stampedpose", 1000)),
		kill(nh.serviceClient<turtlesim::Kill>("kill")),
		posesub(nh.subscribe("pose", 10, &TurtleSine::poseCallback, this))

		
	{
		turtlesim::TeleportAbsolute telep;
		static tf::TransformBroadcaster br;
		
		std::string follow_frame;

		double tx,ty,ttheta;
		nh.getParam("initial_x", tx);
		nh.getParam("initial_y", ty);
		nh.getParam("initial_theta", ttheta);
		nh.getParam("follow_frame", follow_frame);

		if (follow_frame.size()>0){
			movelistener = new FolowListener(this, TIME_DT_FOLOW, nh, follow_frame);
		}else{
			movelistener = new TwistTimerListener(this, TIME_DT_TWIST, nh);
		}

		auto ns = nh.getNamespace();
		auto pos = ns.find_last_of('/');
		turtlename = ns.substr(pos + 1);

		odomtimerlistener->odominittransform();

		/*
			Since both nodes are starting at the same from launcher sometimes turtlesine node starts before
			turtlesim_node, so we need to wait until turtlesim_node appear to use spawn service
		*/
		thread = std::thread(TurtleSine::simWait, this);
		std::unique_lock<std::mutex> lck(mtx);
		cv.wait(lck);

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
			}
		}else{
			telep.request.x = tx;
			telep.request.y = ty;
			telep.request.theta = ttheta;

			clienttelep.call(telep);
		}
	
		gpstransform.setOrigin( tf::Vector3(tx, ty, 0.0) );
		tf::Quaternion q3;
		q3.setRPY(0, 0, ttheta);
		gpstransform.setRotation(q3);
		br.sendTransform(tf::StampedTransform(gpstransform, ros::Time::now(), "map", turtlename + std::string("_odom")));

	}

	void TurtleSine::toPolarP(const geometry_msgs::Point &in, double &alpha, double &r) const {
        r= sqrt((in.x * in.x)  + (in.y * in.y));
        alpha = atan2(in.y, in.x);
    }

	void TurtleSine::FolowListener::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){

		geometry_msgs::PoseStamped transformedpose;
		double angle, range;
		geometry_msgs::Twist twist;

		ros::Time time_now = ros::Time::now();

		if (follow_frame.compare(msg->header.frame_id)){
			return;
		}

		try{
             parent->tflistener.waitForTransform(parent->turtlename+std::string("_base_link"), msg->header.frame_id, time_now, ros::Duration(0.01));
        }catch(tf::TransformException& ex){
            ROS_ERROR("Wait an exception trying to transform a point from \"%s\" to \"%s\": %s",  "map", std::string(parent->turtlename+std::string("_base_link")).c_str(), ex.what());
        }

        try{
            parent->tflistener.transformPose(parent->turtlename+std::string("_base_link"), *msg, transformedpose);
        }catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", msg->header.frame_id.c_str(), std::string(parent->turtlename+std::string("_base_link")).c_str(), ex.what());
        }

        parent->toPolarP(transformedpose.pose.position, angle, range);

        twist.linear.x = range;
		twist.linear.y = 0;
		twist.linear.z = 0;
	
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = angle;
		
		parent->pubsine.publish(twist);
		parent->odomtimerlistener->latesttwist = twist;

	}
	
	void TurtleSine::FolowListener::timerCallback(const ros::TimerEvent& e)
	{
		lastevent = e;
	}
	
	void TurtleSine::TwistTimerListener::timerCallback(const ros::TimerEvent& e)
	{
		geometry_msgs::Twist twist;
		lastevent = e;
		double as,ls;
		parent->nh.getParam("a_speed", as);
		parent->nh.getParam("l_speed", ls);

		ros::TimerEvent odomeevent;
		odomeevent.current_real = e.current_real;
		odomeevent.last_real = parent->odomtimerlistener->lastevent.last_real;
		parent->odomtimerlistener->timerCallback(odomeevent);

		twist.linear.x = as;
		twist.linear.y = 0;
		twist.linear.z = 0;
	
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = ls;
		
		if ((count % 3) == 0){
			twist.angular.z *= -1;
		}

		parent->pubsine.publish(twist);
		parent->odomtimerlistener->latesttwist = twist;
		count++;
	  	
	}

	void TurtleSine::OdomTimerListener::timerCallback(const ros::TimerEvent& e)
	{
		lastevent = e;
		/* Odometry formula*/
		nav_msgs::Odometry odom;

		geometry_msgs::PoseStamped ps;

		double dt = e.current_real.toSec() - e.last_real.toSec();
		
		static tf::TransformBroadcaster br;
		tf::Transform transform_bs;
		tf::StampedTransform maptransform;
		ros::Time time_now = ros::Time::now();

		double vx = latesttwist.linear.x;
		double vy = latesttwist.linear.y; 
		double th = latesttwist.angular.z;
	
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

		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, lastpose.at(POSE_THETA));

		odom.header.stamp = time_now;
		
		odom.header.frame_id = parent->turtlename + std::string("_odom");
		odom.child_frame_id = parent->turtlename + std::string("_base_link");

	
		odom.pose.pose.position.x = lastpose.at(POSE_X);
		odom.pose.pose.position.y = lastpose.at(POSE_Y);
		odom.pose.pose.position.z = 0;
		ps.pose.position = odom.pose.pose.position;

		odom.pose.pose.orientation.x = q[0];
		odom.pose.pose.orientation.y = q[1];
		odom.pose.pose.orientation.z = q[2];
		odom.pose.pose.orientation.w = q[3];
		ps.pose.orientation = odom.pose.pose.orientation;

		odom.twist.twist = latesttwist;
		parent->odompub.publish(odom);

		ps.header.stamp = time_now;
		ps.header.frame_id = parent->turtlename + std::string("_base_link");

		parent->posepub.publish(ps);


		/*
			After we create separate thread we may uncomment this lines to send the transformation of odometry
		*/
		br.sendTransform(tf::StampedTransform(parent->gpstransform, time_now, "map", parent->turtlename+std::string("_odom")));

#ifdef SIMULATE_GPS

		transform_bs.setOrigin( tf::Vector3(lastpose.at(POSE_X), lastpose.at(POSE_Y), 0.0) );
		transform_bs.setRotation(q);  		
		br.sendTransform(tf::StampedTransform(transform_bs, time_now, parent->turtlename + std::string("_odom"), parent->turtlename + std::string("_base_link")));
#endif
		count++;

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
