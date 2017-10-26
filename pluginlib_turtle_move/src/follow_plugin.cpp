#include <pluginlib/class_list_macros.h>
#include <pluginlib_turtle_move/plugin_base.h>

#include <pluginlib_turtle_move/follow.h>

PLUGINLIB_EXPORT_CLASS(move_plugins::FolowListener, move_base::BaseListener)

namespace move_plugins {
    void FolowListener::toPolarP(const geometry_msgs::Point &in, geometry_msgs::Pose2D &out) const { 
        out.x = sqrt((in.x * in.x)  + (in.y * in.y));
        out.theta = atan2(in.y, in.x);
    }

    FolowListener::FolowListener(): BaseListener() {}

    void FolowListener::initialize(double dur, ros::NodeHandle &n){
        nh = n;
        duration = dur;
        std::string turtletarget;
        nh.param<std::string>("turtletarget", turtletarget, "/demo/turtletarget");
        folowposesub = nh.subscribe(turtletarget, 10, &FolowListener::poseCallback, this); 
    }

    void FolowListener::setnames(std::string topicname, std::string tn){
        turtlename = tn;
        follow_frame = topicname;
    }

    void FolowListener::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){

        geometry_msgs::PoseStamped transformedpose;
        double angle, range;
        geometry_msgs::Twist twist;
        geometry_msgs::Pose2D outpoint;

        const auto time_now = msg->header.stamp;
        dt = time_now.toSec();

        if (follow_frame.compare(msg->header.frame_id)){
            return;
        }

        try{
            tflistener.waitForTransform(turtlename+std::string("_base_link"), msg->header.frame_id, time_now, ros::Duration(0.1));
        }catch(tf::TransformException& ex){
            ROS_WARN("WaitForTransform exception \"%s\" to \"%s\": %s",  
                "map", 
                std::string(turtlename+std::string("_base_link")).c_str(), 
                ex.what());
        }

        try{
            tflistener.transformPose(turtlename+std::string("_base_link"), *msg, transformedpose);
        }catch(tf::TransformException& ex){
            ROS_WARN("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", 
                msg->header.frame_id.c_str(), 
                std::string(turtlename+std::string("_base_link")).c_str(), 
                ex.what());
        }

        toPolarP(transformedpose.pose.position, outpoint);

        //If turtle is far compensate faster - log
        twist.linear.x = outpoint.x * (1.0 + outpoint.x/2.0);
        twist.linear.y = 0.;
        twist.linear.z = 0.;
    
        twist.angular.x = 0.;
        twist.angular.y = 0.;
         //If running turtle is behind following turtle it will turn faster
        twist.angular.z = outpoint.theta * (1.0 +  2.0*abs(outpoint.theta));
        
        pubsine.publish(twist);
    }
    
    void FolowListener::timerCallback(const ros::TimerEvent& e) {}
}