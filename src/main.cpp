#include <ros/ros.h>

#include "a3env/motors.h"
#include "a3env/sonars.h"
#include "a3env/odom.h"
#include "a3planner/plan.h"

#include "agent.hpp"



static constexpr size_t             NUM_AGENTS = 6;
static std::vector<Agent>           agents       (NUM_AGENTS);
static std::vector<ros::Subscriber> subscribers0 (NUM_AGENTS);
static std::vector<ros::Subscriber> subscribers1 (NUM_AGENTS);


int main( int argc, char **argv )
{
    srand(clock());

    ros::init(argc, argv, "a3agent");
    ros::NodeHandle n;

    ros::ServiceClient plan_client   = n.serviceClient<a3planner::plan>("/a3planner/plan");
    ros::ServiceClient motors_client = n.serviceClient<a3env::motors>("/a3env/motors");

    for (int i=0; i<NUM_AGENTS; i++)
    {
        agents[i] = Agent(i, &plan_client, &motors_client);

        std::string label1 = "/a3env/sonars" + std::to_string(i);
        std::string label2 = "/a3env/odom" + std::to_string(i);

        subscribers0[i] = n.subscribe(label1, 512, &Agent::sonars_callback, &agents[i]);
        subscribers1[i] = n.subscribe(label2, 512, &Agent::odom_callback, &agents[i]);
    }

    ros::spin();


    return 0;
}


