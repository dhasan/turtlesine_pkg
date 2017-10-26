#include <pluginlib/class_list_macros.h>
#include <pluginlib_turtle_move/plugin_base.h>
#include <pluginlib_turtle_move/twist.h>

PLUGINLIB_EXPORT_CLASS(move_plugins::TwistTimerListener, move_base::BaseListener)

namespace move_plugins {
    
    TwistTimerListener::TwistTimerListener():BaseListener(){}

    void TwistTimerListener::initialize(double dur, ros::NodeHandle &n)
    {
        nh = n;
        duration = dur;
    }


    void TwistTimerListener::timerCallback(const ros::TimerEvent& e)
    {
        geometry_msgs::Twist twist;
        lastevent = e;
        double as=.0,ls=.0;
        nh.getParam("a_speed", as);
        nh.getParam("l_speed", ls);

        ros::TimerEvent odomeevent;

        twist.linear.x = ls;
        twist.linear.y = 0.;
        twist.linear.z = 0.;
    
        twist.angular.x = 0.;
        twist.angular.y = 0.;
        twist.angular.z = as;
        
        if ((count % 3) == 0){
            twist.angular.z *= -1.0;
        }

        pubsine.publish(twist);
        count++;   
    }

}