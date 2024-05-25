#include <ros/ros.h>

#include "a3env/motors.h"
#include "a3env/sonars.h"



void agentLogic( int agentID, ros::ServiceClient &sonars, ros::ServiceClient &motors )
{
    a3env::sonars sonars_srv;
    sonars_srv.request.agentid  = agentID;

    if (!sonars.call(sonars_srv))
    {
        return;
    }
    

    float dist = sonars_srv.response.distance;

    float linear, angular;

    if (dist < 1.0f)
    {
        linear  = 0.0f;
        angular = (rand() % 100) / 100.0f;
        angular -= 0.5f;
        angular *= 0.5f;
    }

    else
    {
        linear  = 0.1f;
        angular = 0.0f;
    }


    a3env::motors motors_srv;
    motors_srv.request.agentid = agentID;
    motors_srv.request.linear  = linear;
    motors_srv.request.angular = angular;

    if (!motors.call(motors_srv))
    {
        return;
    }

}


int main( int argc, char **argv )
{
    ros::init(argc, argv, "a3agent");
    ros::NodeHandle n;

    ros::ServiceClient motors = n.serviceClient<a3env::motors>("/a3env/motors");
    ros::ServiceClient sonars = n.serviceClient<a3env::sonars>("/a3env/sonars");

    // for (int i=0; i<5; i++)
    // {
    //     a3env::motors motors_srv;
    //     motors_srv.request.agentid = i;
    //     motors_srv.request.linear  = 0.0f;
    //     // motors_srv.request.angular = (rand()%100) / 100.0f;

    //     motors.call(motors_srv);
    // }

    while (ros::ok())
    {
        for (int i=0; i<5; i++)
        {
            agentLogic(i, sonars, motors);
        }
    }


    return 0;
}
