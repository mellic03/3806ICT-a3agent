#include "agent.hpp"



void
Agent::scanning_behaviour()
{
    static float total = 0.0f;

    // if (m_bearing >= 2.0f*M_PI)
    // {
    //     m_bearing = 0.0f;
    //     m_state = STATE_IDLE;
    // }

    // else
    // {
    //     m_linear  = 0.0f;
    //     m_bearing = 0.1f;
    // }
}


void
Agent::idle_behaviour()
{
    if (!m_plan_client->call(m_plan_srv))
    {
        ROS_ERROR("Could not call service \"/a3planner/plan\"");
        return;
    }

    m_state = STATE_FOLLOWING_PLAN;


    for (int i=0; i<10; i++)
    {
        std::cout << char(m_plan_srv.response.plan[i]) << " ";
    }
    std::cout << "\n";

    // if (m_sonar_dist < 1.0f)
    // {
    //     m_linear = 0.0f;

    //     float prev = m_bearing;
    //     m_bearing += 0.01f;
    //     // m_bearing = (rand() % 100) / 100.0f;
    //     // m_bearing -= 0.5f;
    //     // m_bearing *= 3.14159f;
    //     // m_bearing = 0.8f*prev + 0.2f*m_bearing;
    // }

    // else
    // {
    //     m_linear  = 0.1f;

    //     float prev = m_bearing;
    //     m_bearing -= 0.01f;
    //     // m_bearing = (rand() % 100) / 100.0f;
    //     // m_bearing -= 0.5f;
    //     // m_bearing *= 3.14159f;
    //     // m_bearing = 0.8f*prev + 0.2f*m_bearing;
    // }
}


void
Agent::follow_behaviour()
{
    static int current = 0;

    auto &plan = m_plan_srv.response.plan;

    if (plan[current] != 0)
    {

    }

}



void
Agent::update()
{
    switch (m_state)
    {
        case STATE_IDLE:            idle_behaviour();     break;
        case STATE_SCANNING:        scanning_behaviour(); break;
        case STATE_FOLLOWING_PLAN:  follow_behaviour();   break;
    }

    update_motors();
}



void
Agent::sonars_callback( const a3env::sonars &msg )
{
    // static uint64_t prev_utc = msg.timestamp;
    // uint64_t utc = msg.timestamp;
    m_sonar_dist = msg.distance;
    // float delta = (float(utc) / float(prev_utc)) / 1000000.0f;
    // std::cout << "dt: " << delta << "\n";
    // prev_utc = utc;


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

    update();
}


void
Agent::odom_callback( const a3env::odom &msg )
{
    m_position.x = msg.xpos;
    m_position.y = msg.ypos;
    // m_bearing    = msg.bearing;

    update();
}


void
Agent::update_motors()
{
    a3env::motors srv;
    srv.request.agentid = m_ID;
    srv.request.linear  = m_linear;
    srv.request.bearing = m_bearing;


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



void
Agent::follow_plan()
{
    // uint16_t *plan = m_plan_srv.response.plan;

    // static int current = 0;

    // if (m_state != STATE_FOLLOWING_PLAN)
    // {
    //     m_state = STATE_FOLLOWING_PLAN;
    //     current = 0;
    // }

    // if (plan[current] != 0)
    // {
    //     uint16_t action = plan[current];

    //     switch (action)
    //     {
    //         case 'L':   break;
    //         case 'R':   break;
    //         case 'U':   break;
    //         case 'D':   break;

    //         default:
    //             break;
    //     }

    //     current += 1;
    // }

    // else
    // {
    //     m_state = STATE_IDLE;
    // }




}


