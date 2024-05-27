#pragma once

#include <vector>
#include <glm/glm.hpp>

#include <ros/ros.h>

#include "a3env/sonars.h"
#include "a3env/odom.h"
#include "a3env/motors.h"
#include "a3planner/plan.h"

#include "../../a3env/src/common.hpp"


class Agent
{
private:
    enum State
    {
        STATE_IDLE,
        STATE_MOVING_TO_SURVIVOR,
        STATE_RETURNING_HOME,
        STATE_SOMETHING_ELSE
    };

    Agent::State m_state = STATE_IDLE;


public:

    int m_ID;
    ros::ServiceClient  *m_plan_client;
    ros::ServiceClient  *m_motors_client;
    a3planner::plan      m_plan_srv;

    glm::vec2 m_position;
    float     m_bearing;
    float     m_linear;
    float     m_angular;
    float     m_sonar_dist;

    Agent() {  };

    Agent( int id, ros::ServiceClient *plan_client, ros::ServiceClient *motors_client )
    :   m_ID            (id),
        m_plan_client   (plan_client),
        m_motors_client (motors_client)
    {
        for (int i=0; i<a3env::NUM_HOSTILES; i++)
        {
            m_plan_srv.request.hostiles[i] = -1;
        }
    };

    void sonars_callback( const a3env::sonars &msg );
    void odom_callback( const a3env::odom &msg );

    void request_plan();
    void update_motors();

};


