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
        STATE_SCANNING,
        STATE_FOLLOWING_PLAN,
        STATE_MOVING_TO_SURVIVOR,
        STATE_RETURNING_HOME,
        STATE_SOMETHING_ELSE
    };

    Agent::State m_state;

    float m_scan_bearing = 0.0f;


public:

    int m_ID;
    ros::ServiceClient  *m_plan_client;
    // ros::ServiceClient  *m_motors_client;
    ros::Publisher      *m_motors_pub;


    std::vector<glm::vec2> m_path;
    std::vector<int>       m_worldview;
    a3planner::plan        m_plan_srv;

    glm::vec2 m_position = glm::vec2(1.5f);
    float     m_bearing = 0.0f;
    float     m_linear;
    float     m_angular;

    float     m_sonar_dist;
    glm::vec2 m_sonar_hit;


    Agent() {  };

    Agent( int id, ros::ServiceClient *plan_client, ros::Publisher *motors_pub )
    :   m_ID            (id),
        m_plan_client   (plan_client),
        m_motors_pub (motors_pub),
        m_state         (STATE_IDLE),
        m_worldview     (a3env::MAP_WIDTH*a3env::MAP_WIDTH, 0)
    {
        m_plan_srv.request.world.resize(a3env::MAP_WIDTH*a3env::MAP_WIDTH);

        for (int i=0; i<a3env::NUM_HOSTILES; i++)
        {
            // m_plan_srv.request.hostiles[i] = -1;
        }
    };

    void set_state( Agent::State state );

    void sonars_callback( const a3env::sonars &msg );
    void odom_callback( const a3env::odom &msg );

    void update_motors();

    void request_plan();
    void follow_plan();

    void update();

    void idle_behaviour();
    void scanning_behaviour();
    void follow_behaviour();
};


