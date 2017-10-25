#ifndef _MAPSIM_H_
#define _MAPSIM_H_

#include "ros/ros.h"
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <tf/transform_listener.h>


class Lasersim
{
public:
    class TurtleListener
    {
        public:
            void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
            TurtleListener(Lasersim &p, std::string top, ros::NodeHandle &n);
            TurtleListener() = delete;
              
            virtual ~TurtleListener() = default;
            TurtleListener(const TurtleListener&) = default;
            TurtleListener& operator= (const TurtleListener&) = default; //copy assignment operator
            TurtleListener(TurtleListener&&) =default; //move constructor
            TurtleListener& operator=(TurtleListener&&) = default; //move assignment operator
        private:
            Lasersim &parent;
            std::string topic;
            ros::NodeHandle &nh;
            ros::Subscriber turtlessub;
            //turtle pose publisher
            ros::Publisher posepub;
    };

    static const std::string node_name;

    void wallsCallback(const sensor_msgs::PointCloudConstPtr &msg);
    //Transformation from cadestrian coordinates to polar
    void toPolarPC(const sensor_msgs::PointCloud &in, geometry_msgs::Pose2D *out) const;
    void toPolarP(const geometry_msgs::Point &in, geometry_msgs::Pose2D &out) const;
    
    Lasersim(ros::NodeHandle &n);
    Lasersim()=delete;
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
    std::string lasername;
    //Walls subscriber
    ros::Subscriber wallssub;
    tf::TransformListener tflistener;

    //LaserScan publisher
    ros::Publisher laserscan;
    //Laser scan
    sensor_msgs::LaserScan ls;
    std::array<TurtleListener, 4> turtlelisteners;
    //laser message measurements 
    int measurments;
    unsigned int flags;
    int turtles_cnt;
};
#endif