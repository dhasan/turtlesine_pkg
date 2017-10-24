
#ifndef PLUGINLIB_TURTLE_MOVE__PLUGIN_TWIST_H_
#define PLUGINLIB_TURTLE_MOVE__PLUGIN_TWIST_H_

#include "plugin_base.h"

namespace move_plugins {

    class TwistTimerListener : public move_base::BaseListener{
        
        public:

            TwistTimerListener();
                 
            virtual ~TwistTimerListener() = default;
            TwistTimerListener(const TwistTimerListener&) = delete;
            TwistTimerListener& operator= (const TwistTimerListener&) = delete; //copy assignment operator
            TwistTimerListener(TwistTimerListener&&) =delete; //move constructor
            TwistTimerListener& operator=(TwistTimerListener&&) = delete; //move assignment operator

            virtual void timerCallback(const ros::TimerEvent& e);
            virtual void initialize(double dur, ros::NodeHandle &n);
            
        private:
            //counter of twist commands, 
            int count;
    };
}
#endif