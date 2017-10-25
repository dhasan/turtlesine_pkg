#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"

#include <thread>             
#include <mutex>              
#include <condition_variable> 

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>

#include <cmath>

#include <pluginlib_turtle_move/plugin_base.h>
#include <pluginlib_turtle_move/twist.h>
#include <pluginlib_turtle_move/follow.h>

#define     POSE_X        (0)
#define     POSE_Y        (1)
#define     POSE_THETA    (2)

#define     TIME_DT_TWIST    (0.9)
#define     TIME_DT_ODOM     (0.01)
#define     TIME_DT_FOLOW    (0.1)

class TurtleSine 
{

public:    
    static const std::string node_name;
    static void sigHandler (int signum);
    void poseCallback(const turtlesim::PoseConstPtr& msg);
    TurtleSine(ros::NodeHandle &n);

    TurtleSine() = delete;
    virtual ~TurtleSine();
    TurtleSine(const TurtleSine&) = delete;
    TurtleSine& operator= (const TurtleSine&) = delete; //copy assignment operator
    TurtleSine(TurtleSine&&) = delete; //move constructor
    TurtleSine& operator=(TurtleSine&&) = delete; //move assignment operator
private:
    class BaseListener
    {
        
        public:
            BaseListener() = delete;
             
            virtual ~BaseListener() = default;
            BaseListener(const BaseListener&) = delete;
            BaseListener& operator= (const BaseListener&) = delete; //copy assignment operator
            BaseListener(BaseListener&&) =delete; //move constructor
            BaseListener& operator=(BaseListener&&) = delete; //move assignment operator
            double getDuration() const { return duration;}
            virtual void timerCallback(const ros::TimerEvent& e) = 0;    
            ros::TimerEvent lastevent;
        protected:
            BaseListener(TurtleSine &p, double dur, ros::NodeHandle &n);
            ros::NodeHandle &nh;    
            double duration;
            TurtleSine &parent;
            
        private:
            //ROS timer
            ros::Publisher pubsine;
            ros::Timer timer;
        
    };
    class OdomTimerListener : public BaseListener{
    
        public:
            OdomTimerListener(TurtleSine &p, double dur, ros::NodeHandle &nh);
            OdomTimerListener() = delete;
             
            virtual ~OdomTimerListener() = default;
            OdomTimerListener(const OdomTimerListener&) = delete;
            OdomTimerListener& operator= (const OdomTimerListener&) = delete; //copy assignment operator
            OdomTimerListener(OdomTimerListener&&) =delete; //move constructor
            OdomTimerListener& operator=(OdomTimerListener&&) = delete; //move assignment operator
            virtual void timerCallback(const ros::TimerEvent& e);
            void setLastPose(double x, double y, double theta){lastpose.x = x, lastpose.y = y, lastpose.theta = theta;}
            void odominittransform();
            int getCount() const {return count;}
            geometry_msgs::Twist& getLatestTwist() {return latesttwist;} 
            void setLatestTwist(const geometry_msgs::Twist &twist){latesttwist = twist;}
            
        private:
            //Odometry storage
            geometry_msgs::Pose2D lastpose;

            //store the last twist so, odometry can accumulated 
            geometry_msgs::Twist latesttwist; 

            //counter of odometry calculations
            int count;
        
    };
    //Conditional variable callback 
    static void simWait(TurtleSine *obj);
  
    //ROS node handle
    ros::NodeHandle& nh;
    
    //Velocities command publisher and turtlesim pose 
    ros::Publisher pubsine;
    ros::Publisher posepub;

    tf::Transform gpstransform;
    //Turtle teleportation service client
    ros::ServiceClient clienttelep;
    //Turtle spawn service client
    ros::ServiceClient spawn;
        
    std::unique_ptr<OdomTimerListener> odomtimerlistener;
    //Turtle odometry publisher
    ros::Publisher odompub;
    //Turtle kill service client
    ros::ServiceClient kill;
    //Pose subscriber from turtlesim
    ros::Subscriber posesub;

    //Name of the turtle
    std::string turtlename;

    std::mutex mtx;
    std::condition_variable cv;
    std::thread thread;

    tf::TransformListener tflistener;
    tf::TransformBroadcaster br;
    
    //Library loader
    pluginlib::ClassLoader<move_base::BaseListener> move_loader;

    decltype(move_loader.createInstance("")) movelistener;
};

#endif