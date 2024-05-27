#include "agent.hpp"



void
Agent::sonars_callback( const a3env::sonars &msg )
{
    m_sonar_dist = msg.distance;

    glm::vec2 dir = glm::vec2(cos(m_bearing), sin(m_bearing));
    glm::vec2 cell = m_position + m_sonar_dist*dir;

    int row = int(cell.y);
    int col = int(cell.x);

    row = glm::clamp(row, 0, 11);
    col = glm::clamp(col, 0, 11);


    constexpr size_t W = a3env::MAP_WIDTH;

    // Update known hostile locations if response is BLOCK_HOSTILE
    // -----------------------------------------------------------
    if (msg.blocktype == a3env::BLOCK_HOSTILE)
    {
        for (int i=0; i<a3env::NUM_HOSTILES; i++)
        {
            int idx = msg.data & (1 << i);

            if (msg.data & (1 << i))
            {
                m_plan_srv.request.hostiles[i] = W*row + col;
            }
        }
    }
    // -----------------------------------------------------------

    else
    {
        m_plan_srv.request.world[W*row + col] = uint8_t(msg.blocktype);
    }


    update_motors();
}


void
Agent::odom_callback( const a3env::odom &msg )
{
    m_position.x = msg.xpos;
    m_position.y = msg.ypos;
    m_bearing    = msg.bearing;
    update_motors();
}


void
Agent::update_motors()
{
    // Currently just random movement.

    if (m_sonar_dist < 1.0f)
    {
        m_linear = 0.0f;

        float prev = m_angular;
        m_angular = (rand() % 100) / 100.0f;
        m_angular -= 0.5f;
        m_angular *= 0.5f;
        m_angular = 0.8f*prev + 0.2f*m_angular;
    }

    else
    {
        m_linear  = 0.1f;

        float prev = m_angular;
        m_angular = (rand() % 100) / 100.0f;
        m_angular -= 0.5f;
        m_angular *= 0.25f;
        m_angular = 0.8f*prev + 0.2f*m_angular;
    }


    a3env::motors srv;
    srv.request.agentid = m_ID;
    srv.request.linear  = m_linear;
    srv.request.angular = m_angular;


    if (!m_motors_client->call(srv))
    {
        ROS_ERROR("Could not call service \"/a3env/motors\"");
        return;
    }
}



void
Agent::request_plan()
{
    m_plan_srv.request.row = int(m_position.y);
    m_plan_srv.request.col = int(m_position.x);
    // m_plan_srv.request.world // world data is already stored here.

    if (!m_plan_client->call(m_plan_srv))
    {
        ROS_ERROR("Could not call service \"/a3planner/plan\"");
        return;
    }

}

