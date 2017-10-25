
#ifndef PLUGINLIB_TURTLE_MOVE__PLUGIN_FOLLOW_H_
#define PLUGINLIB_TURTLE_MOVE__PLUGIN_FOLLOW_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include "plugin_base.h"

namespace move_plugins {
    class FolowListener : public move_base::BaseListener{
        
        public:

            FolowListener();
            virtual ~FolowListener() = default;
            FolowListener(const FolowListener&) = delete;
            FolowListener& operator= (const FolowListener&) = delete; //copy assignment operator
            FolowListener(FolowListener&&) =delete; //move constructor
            FolowListener& operator=(FolowListener&&) = delete; //move assignment operator

            virtual void timerCallback(const ros::TimerEvent& e);
            virtual void initialize(double dur, ros::NodeHandle &n);

            void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
            void setnames(std::string topicname, std::string tn);

        private:
            void toPolarP(const geometry_msgs::Point &in, geometry_msgs::Pose2D &out) const;
            ros::Subscriber folowposesub;
            std::string follow_frame;
            tf::TransformListener tflistener;
            std::string turtlename;
            double dt;
    };
};
#endif