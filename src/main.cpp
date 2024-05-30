#include <ros/ros.h>

#include "a3env/motors.h"
#include "a3env/sonars.h"
#include "a3env/odom.h"
#include "a3planner/plan.h"

#include "a3env/motors.h"
#include "agent.hpp"

#include "../../a3env/src/common.hpp"

static std::vector<Agent>           agents          (a3env::NUM_AGENTS);
// static std::vector<int>             agent_positions (a3env::NUM_AGENTS);
static std::vector<ros::Subscriber> subscribers0    (a3env::NUM_AGENTS);
static std::vector<ros::Subscriber> subscribers1    (a3env::NUM_AGENTS);
static std::vector<ros::Publisher>  publishers      (a3env::NUM_AGENTS);


int main( int argc, char **argv )
{
    srand(clock());

    ros::init(argc, argv, "a3agent");
    ros::NodeHandle n;

    ros::ServiceClient plan_client   = n.serviceClient<a3planner::plan>("/a3planner/plan");

    for (int i=0; i<a3env::NUM_AGENTS; i++)
    {
        std::string label0 = "/a3env/motors" + std::to_string(i);
        std::string label1 = "/a3env/sonars" + std::to_string(i);
        std::string label2 = "/a3env/odom" + std::to_string(i);

        publishers[i]   = n.advertise<a3env::motors>(label0, 64);
        subscribers0[i] = n.subscribe(label1, 64, &Agent::sonars_callback, &agents[i]);
        subscribers1[i] = n.subscribe(label2, 64, &Agent::odom_callback, &agents[i]);
    
        agents[i] = Agent(i, &plan_client, &publishers[i]);
    }


    ros::spin();

    return 0;
}


