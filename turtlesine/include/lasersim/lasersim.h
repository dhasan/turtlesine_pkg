#ifndef _MAPSIM_H_
#define _MAPSIM_H_

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>
#include <cmath>

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
    void toPolarPC(const sensor_msgs::PointCloud &in, std::vector<double> &alpha, std::vector<double> &r) const;
    void toPolarP(const geometry_msgs::Point &in, double &alpha, double &r) const;
    
    Lasersim(ros::NodeHandle &n);
    //Nodelet is using this constructor, so keeping it non-default.....
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
    std::array<TurtleListener,4> turtlelisteners;
    //laser message measurements 
    int measurments;
    unsigned int flags;
    int turtles_cnt;
};
#endif