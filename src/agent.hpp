#pragma once

#include <vector>
#include <glm/glm.hpp>

#include <ros/ros.h>

#include "a3env/sonars.h"
#include "a3env/odom.h"
#include "a3env/motors.h"
#include "a3env/deactivate.h"
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
        STATE_FINISHED
    };

    Agent::State m_state;

    bool first_plan = true;
    int  first_plan_count = 0;
    bool first_odom = true;


public:

    int m_ID;
    ros::ServiceClient  *m_plan_client;
    ros::Publisher      *m_motors_pub;


    std::vector<glm::vec2>  m_path;
    std::vector<uint16_t>   m_hostiles;
    a3planner::plan         m_plan_srv;

    glm::vec2 m_home     = glm::vec2(1.5f);
    glm::vec2 m_position = glm::vec2(1.5f);
    float     m_bearing = 0.0f;
    float     m_linear;
    float     m_angular;

    float     m_sonar_dist;
    glm::vec2 m_sonar_hit;


    Agent() {  };
    Agent( int id, ros::ServiceClient *plan_client, ros::Publisher *motors_pub,
           ros::Publisher *deactivate_pub );

    int  hostile_at_cell( int row, int col );
    int  agent_at_cell( int row, int col );

    void set_state( Agent::State state );

    void sonars_callback( const a3env::sonars &msg );
    void odom_callback( const a3env::odom &msg );

    void request_plan();
    void update_motors();
    void update();

    void print_world();

    void idle_behaviour();
    void follow_behaviour();
};


