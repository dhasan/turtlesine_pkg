#include <pluginlib/class_list_macros.h>
#include <pluginlib_turtle_move/plugin_base.h>
//#include <pluginlib_turtle_move/twist.h>
#include <pluginlib_turtle_move/follow.h>

PLUGINLIB_EXPORT_CLASS(move_plugins::FolowListener, move_base::BaseListener)

namespace move_plugins {
    void FolowListener::toPolarP(const geometry_msgs::Point &in, double &alpha, double &r) const {
        r= sqrt((in.x * in.x)  + (in.y * in.y));
        alpha = atan2(in.y, in.x);
    }


    FolowListener::FolowListener(): BaseListener() {}
	void FolowListener::initialize(double dur, ros::NodeHandle &n){
		nh = n;
		this->duration = dur;
		folowposesub = nh.subscribe("/demo/turtletarget", 10, &FolowListener::poseCallback, this);
	}

	void FolowListener::setnames(std::string topicname, std::string tn){
		this->turtlename = tn;
		this->follow_frame = topicname;
	}

	void FolowListener::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){

		geometry_msgs::PoseStamped transformedpose;
		double angle, range;
		geometry_msgs::Twist twist;

		auto time_now = ros::Time::now();

		if (follow_frame.compare(msg->header.frame_id)){
			return;
		}

		try{
             tflistener.waitForTransform(turtlename+std::string("_base_link"), msg->header.frame_id, time_now, ros::Duration(0.01));
        }catch(tf::TransformException& ex){
            ROS_ERROR("Wait an exception trying to transform a point from \"%s\" to \"%s\": %s",  "map", std::string(turtlename+std::string("_base_link")).c_str(), ex.what());
        }

        try{
            tflistener.transformPose(turtlename+std::string("_base_link"), *msg, transformedpose);
        }catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", msg->header.frame_id.c_str(), std::string(turtlename+std::string("_base_link")).c_str(), ex.what());
        }

        toPolarP(transformedpose.pose.position, angle, range);

        twist.linear.x = range;
		twist.linear.y = 0;
		twist.linear.z = 0;
	
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = angle;
		
		pubsine.publish(twist);
		//parent->odomtimerlistener->latesttwist = twist;

	}
	
	void FolowListener::timerCallback(const ros::TimerEvent& e)
	{
		lastevent = e;
	}
}