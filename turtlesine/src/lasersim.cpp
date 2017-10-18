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

/*PLUGINLIB_EXPORT_CLASS(task1_pkg::Lasersim, nodelet::Nodelet)

/* namespace task1_pkg {*/

	const std::string Lasersim::node_name = "lasersim";

    Lasersim::TurtleListener::TurtleListener(Lasersim *p) : parent(p){}

    void Lasersim::TurtleListener::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg){
        static tf::TransformBroadcaster br;
       


        geometry_msgs::PoseStamped transformedpose;
        ros::Time time_now = ros::Time::now();

        std::vector<std::string> v;
        boost::split(v, msg->header.frame_id, [](char c){return c == '_';});
        int recivedturtleid = boost::lexical_cast<int>(v.at(0).back());
        int turtleid = boost::lexical_cast<int>(parent->turtlename.back());

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
            int index = static_cast<unsigned int>((alpha - parent->ls.angle_min) * (1.0/parent->ls.angle_increment)) % parent->measurments;

            if (range < parent->ls.ranges[index]) {
                parent->ls.ranges[index] = range;
                parent->ls.intensities[index] = range;
            }
        }
     

        //TODO: move this to member area
        int mask = ((1 << (parent->turtles_cnt + 1)) -1) & ~(1 << (turtleid));

        parent->flags |= (1 << recivedturtleid);
   
        if (parent->flags == mask){
        
            parent->ls.header.stamp = time_now;
            parent->laserscan.publish(parent->ls);
            parent->resetlaser();          
       }

    }


    Lasersim::Lasersim(ros::NodeHandle &n): nh(n), 
        wallslistener(this),

        //TODO: use list for listeners!!!
        turtlelisteners {this, this, this, this, this, this, this, this, this, this}, //Up to 10 turtles can be scanned
        laserscan(nh.advertise<sensor_msgs::LaserScan>("laserscan", 1000)),
		wallssub(nh.subscribe("walls", 10, &WallsListener::wallsCallback, &wallslistener)){

        auto ns = nh.getNamespace();
        int pos = ns.find_last_of('/');
        std::string res = ns.substr(pos + 1);
        std::vector<std::string> v;
        boost::split(v, res, [](char c){return c == '_';});
        turtlename = v.at(0);
        lasername = v.at(1);

        flags = 0;

        nh.getParam("turtles_cnt", turtles_cnt);

        int turtleid = boost::lexical_cast<int>(turtlename.back());
        ROS_ERROR("turtleid : %d", turtleid);

        for(int i = 0; i<turtles_cnt; ++i){
            if (i!= (turtleid - 1)){
                char buff[16];

                sprintf(buff, "stampedpose%d", i+1);
                turtlessub[i] = nh.subscribe(buff, 10, &TurtleListener::poseCallback, &turtlelisteners[i]);
                ROS_ERROR("init");
            }
        }

        dt =ros::Time::now().toSec();

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

    Lasersim::WallsListener::WallsListener(Lasersim *p) : parent(p){}


    void Lasersim::toPolarPC(const sensor_msgs::PointCloud &in, std::vector<double> &alpha, std::vector<double> &r) const {

        for (int i=0;i<in.points.size();i++){
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
        
        for(int i=0;i<measurments;i++)
            ls.ranges[i] = ls.range_max;

        flags = 0;
    }

    void Lasersim::WallsListener::wallsCallback(const sensor_msgs::PointCloudConstPtr &msg){

        static tf::TransformBroadcaster br;
        sensor_msgs::PointCloud turtlepc;
        
        if (parent->flags & 1)
            return;
    
        ros::Time time_now = ros::Time::now();

        try{
            parent->tflistener.waitForTransform("map", parent->turtlename + std::string("_")+parent->lasername, time_now, ros::Duration(0.01));
        }catch(tf::TransformException& ex){
            ROS_ERROR("Wait an exception trying to transform a point from \"%s\" to \"%s\": %s", "map", std::string(parent->turtlename + std::string("_")+parent->lasername).c_str(), ex.what());
        }

        try{
            parent->tflistener.transformPointCloud(parent->turtlename + std::string("_")+parent->lasername, *msg, turtlepc);
        }catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", msg->header.frame_id.c_str(), std::string(parent->turtlename + std::string("_")+parent->lasername).c_str(), ex.what());
        }

        std::vector<double> alphas(parent->measurments, 0);
        std::vector<double> ranges(parent->measurments, 0);

        parent->toPolarPC(turtlepc, alphas, ranges);

        for(int i=0;i<parent->measurments;i++){

            if (ranges[i] > parent->ls.range_max)
                continue;

            if ((alphas[i]> parent->ls.angle_max) ||  (alphas[i]< parent->ls.angle_min))
                continue;

            int index = static_cast<unsigned int>((alphas[i] - parent->ls.angle_min) * (1/parent->ls.angle_increment)) % parent->measurments;

            if (ranges[i] < parent->ls.ranges[index]) {
                parent->ls.ranges[index] = ranges[i];
                parent->ls.intensities[index] = ranges[i];
            }
        }

        parent->flags |= 1;
        int turtleid = boost::lexical_cast<int>(parent->turtlename.back());

        int mask = ((1 << (parent->turtles_cnt + 1)) -1) & ~(1 << (turtleid));

        if (parent->flags == mask){
            
            parent->ls.header.stamp = time_now;
            parent->laserscan.publish(parent->ls);
            parent->resetlaser();
        }

    }

	Lasersim::Lasersim(): nh(getPrivateNodeHandle()){

	}



	void Lasersim::onInit(){

	}
	

/* } */


int main(int argc, char **argv)
{
	ros::init(argc, argv, /* task1_pkg::*/Lasersim::node_name);
#if 1
	
	ros::NodeHandle n("~");
	/*task1_pkg::*/Lasersim ts(n);

#else
	nodelet::Loader nodelet;
  	nodelet::M_string remap(ros::names::getRemappings());
  	nodelet::V_string nargv;
  	std::string nodelet_name = ros::this_node::getName();
  	nodelet.load(nodelet_name, "task1_pkg/Lasersim", remap, nargv);
#endif

	ros::spin();

	return 0;
}
