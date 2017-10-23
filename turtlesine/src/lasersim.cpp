#include "lasersim.h"
#include "nodelet/loader.h"
#include <string>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>

#include <geometry_msgs/Point32.h>



	const std::string Lasersim::node_name = "lasersim";

    Lasersim::TurtleListener::TurtleListener(Lasersim *p, std::string top, ros::NodeHandle &n) : parent(p), topic(top), nh(n),
        turtlessub(nh.subscribe(top, 10, &TurtleListener::poseCallback, this)),
        posepub(nh.advertise<geometry_msgs::PoseStamped>("/demo/turtletarget", 1000)){} //TODO: use relative path

    void Lasersim::TurtleListener::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    static tf::TransformBroadcaster br;
       
        bool inrange;

        geometry_msgs::PoseStamped transformedpose,mappose;
        ros::Time time_now = ros::Time::now();

        std::vector<std::string> v;
        boost::split(v, msg->header.frame_id, [](char c){return c == '_';});
        auto recivedturtleid = boost::lexical_cast<int>(v.at(0).back());
        auto turtleid = boost::lexical_cast<int>(parent->turtlename.back());

        try{
             parent->tflistener.waitForTransform( v.at(0)+std::string("_base_link"), parent->turtlename + std::string("_")+parent->lasername, time_now, ros::Duration(0.01));
        }catch(tf::TransformException& ex){
            ROS_ERROR("Wait an exception trying to transform a point from \"%s\" to \"%s\": %s",  std::string(v.at(0)+std::string("_base_link")).c_str(), std::string(parent->turtlename + std::string("_")+parent->lasername).c_str(), ex.what());
        }

        try{
            parent->tflistener.transformPose(parent->turtlename +  std::string("_")+parent->lasername, *msg, transformedpose);
        }catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", msg->header.frame_id.c_str(), std::string(parent->turtlename +std::string("_")+parent->lasername).c_str(), ex.what());
        }

        double alpha, range;
        parent->toPolarP(transformedpose.pose.position, alpha, range);
        
        if ((range < parent->ls.range_max) && (alpha>parent->ls.angle_min) && (alpha<parent->ls.angle_max)){
            inrange = true;
            auto index = static_cast<unsigned int>((alpha - parent->ls.angle_min) * (1.0/parent->ls.angle_increment)) % parent->measurments;

            if (range < parent->ls.ranges[index]) {
                parent->ls.ranges[index] = range;
                parent->ls.intensities[index] = range;
            }
        }else
            inrange = false;
     

        //TODO: move this to member area
        auto mask = ((1 << (parent->turtles_cnt + 1)) -1) & ~(1 << (turtleid));

        parent->flags |= (1 << recivedturtleid);
   
        if (parent->flags == mask){
        
            parent->ls.header.stamp = time_now;
            parent->laserscan.publish(parent->ls);
            parent->resetlaser();          
        }
        if (inrange){
            posepub.publish(*msg);
        }



    }


    Lasersim::Lasersim(ros::NodeHandle &n): nh(n), 
        turtlelisteners{ TurtleListener(this,"stampedpose1", n), TurtleListener(this,"stampedpose2", n), TurtleListener(this,"stampedpose3", n), TurtleListener(this,"stampedpose4", n)}, //up to 4 turtles
        laserscan(nh.advertise<sensor_msgs::LaserScan>("laserscan", 1000)),
		wallssub(nh.subscribe("walls", 10, &Lasersim::wallsCallback, this)){

        auto ns = nh.getNamespace();
        auto pos = ns.find_last_of('/');
        std::string res = ns.substr(pos + 1);
        std::vector<std::string> v;
        boost::split(v, res, [](char c){return c == '_';});
        turtlename = v.at(0);
        lasername = v.at(1);

        flags = 0;

        nh.getParam("turtles_cnt", turtles_cnt);

        auto turtleid = boost::lexical_cast<int>(turtlename.back());
        ROS_ERROR("turtleid : %d", turtleid);

        double amin, amax, rmin, rmax;
        nh.getParam("measurments", measurments);
       
        nh.getParam("angle_min", amin);
        ls.angle_min = amin;

        nh.getParam("angle_max", amax);
        ls.angle_max = amax;


        ls.angle_increment =(ls.angle_max - ls.angle_min)/measurments;
 
        nh.getParam("range_min", rmin);
        ls.range_min = rmin;

        nh.getParam("range_max", rmax);
        ls.range_max = rmax;

        ls.header.frame_id = turtlename + std::string("_")+lasername;
        

        resetlaser();


	}

    void Lasersim::toPolarPC(const sensor_msgs::PointCloud &in, std::vector<double> &alpha, std::vector<double> &r) const {

        for (auto i=0;i<in.points.size();i++){
            r[i] = (sqrt((in.points[i].x * in.points[i].x)  + (in.points[i].y * in.points[i].y)));
            alpha[i] = (atan2(in.points[i].y, in.points[i].x));
        }

    }

    void Lasersim::toPolarP(const geometry_msgs::Point &in, double &alpha, double &r) const {
        r= (sqrt((in.x * in.x)  + (in.y * in.y)));
        alpha = (atan2(in.y, in.x));
    }

    void Lasersim::resetlaser(){
    
    
        ls.ranges.resize(measurments);
        ls.intensities.resize(measurments);
        
        for(auto i=0;i<measurments;i++)
            ls.ranges[i] = ls.range_max;

        flags = 0;
    }

    void Lasersim::wallsCallback(const sensor_msgs::PointCloudConstPtr &msg){

        static tf::TransformBroadcaster br;
        sensor_msgs::PointCloud turtlepc;
        
        if (flags & 1)
            return;
    
        ros::Time time_now = ros::Time::now();

        try{
            tflistener.waitForTransform("map", turtlename + std::string("_")+lasername, time_now, ros::Duration(0.01));
        }catch(tf::TransformException& ex){
            ROS_ERROR("Wait an exception trying to transform a point from \"%s\" to \"%s\": %s", "map", std::string(turtlename + std::string("_")+lasername).c_str(), ex.what());
        }

        try{
            tflistener.transformPointCloud(turtlename + std::string("_")+lasername, *msg, turtlepc);
        }catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", msg->header.frame_id.c_str(), std::string(turtlename + std::string("_")+lasername).c_str(), ex.what());
        }

        std::vector<double> alphas(measurments, 0);
        std::vector<double> ranges(measurments, 0);

        toPolarPC(turtlepc, alphas, ranges);

        for(auto i=0;i<measurments;i++){

            if (ranges[i] > ls.range_max)
                continue;

            if ((alphas[i]> ls.angle_max) ||  (alphas[i]< ls.angle_min))
                continue;

            auto index = static_cast<unsigned int>((alphas[i] - ls.angle_min) * (1/ls.angle_increment)) % measurments;

            if (ranges[i] < ls.ranges[index]) {
                ls.ranges[index] = ranges[i];
                ls.intensities[index] = ranges[i];
            }
        }

        flags |= 1;
        auto  turtleid = boost::lexical_cast<int>(turtlename.back());

        auto mask = ((1 << (turtles_cnt + 1)) -1) & ~(1 << (turtleid));

        if (flags == mask){
            
            ls.header.stamp = time_now;
            laserscan.publish(ls);
            resetlaser();
        }

    }





int main(int argc, char **argv)
{
	ros::init(argc, argv, Lasersim::node_name);
	
	ros::NodeHandle n("~");
	Lasersim ts(n);

	ros::spin();

	return 0;
}
