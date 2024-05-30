#include "agent.hpp"



void
Agent::scanning_behaviour()
{
    if (m_bearing >= m_scan_bearing + 3.0f*M_PI)
    {
        m_state = STATE_IDLE;
    }

    m_linear = 0.0f;
    m_bearing += 0.05f;
}


void
Agent::idle_behaviour()
{
    request_plan();


    auto &plan = m_plan_srv.response.plan;

    m_path.resize(0);
    glm::vec2 current = glm::floor(m_position) + 0.5f;

    for (int i=0; i<m_plan_srv.response.moves; i++)
    {
        switch (plan[i])
        {
            default: break;
            case 'l': current += glm::vec2(-1.0f, 0.0f); break;
            case 'r': current += glm::vec2(+1.0f, 0.0f); break;
            case 'u': current += glm::vec2(0.0f, -1.0f); break;
            case 'd': current += glm::vec2(0.0f, +1.0f); break;
        }

        int row = int(current.y);
        int col = int(current.x);

        if (m_worldview[a3env::MAP_WIDTH*row + col] != a3env::BLOCK_AIR)
        {
            break;
        }

        std::cout << "plan[" << i << "]: " << current.x << ", " << current.y << "\n";

        m_path.push_back(current);
    }

    std::reverse(m_path.begin(), m_path.end());



    for (int i=0; i<12; i++)
    {
        for (int j=0; j<12; j++)
        {
            int n = int(m_worldview[12*i + j]);

            if (i == int(m_position.y) && j == int(m_position.x))
            {
                std::cout << "o ";
            }

            else if (n == 0) std::cout << "? ";
            else if (n == 1) std::cout << "  ";
            else if (n == 2) std::cout << "# ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";



    std::cout << "Plan: ";
    for (int i=0; i<m_plan_srv.response.moves; i++)
    {
        std::cout << char(m_plan_srv.response.plan[i]) << " ";
    }
    std::cout << "\n";

    m_state = STATE_FOLLOWING_PLAN;
}


void
Agent::follow_behaviour()
{
    if (m_path.empty())
    {
        set_state(STATE_SCANNING);
        return;
    }

    static int count = 0;

    if (m_sonar_dist < 0.1f)
    {
        count += 1;
    }

    if (count >= 256)
    {
        count = 0;
        set_state(STATE_SCANNING);
    }

    glm::vec2 current = m_path.back();
    auto cell = m_worldview[a3env::MAP_WIDTH*int(current.y) + int(current.x)];
    if (cell != a3env::BLOCK_AIR)
    {
        set_state(STATE_SCANNING);
        return;
    }


    if (glm::distance(m_position, current) <= 0.25f)
    {
        m_path.pop_back();
    }

    else
    {
        glm::vec2 dir = glm::normalize(current - m_position);
        m_bearing = glm::mix(m_bearing, asin(dir.y), 0.1f);
        // std::cout << "bearing: " << m_bearing << "\n";
        // std::cout << "dir: " << dir.x << ", " << dir.y << "\n";
    }

    m_linear = 0.1f;
}


void
Agent::set_state( Agent::State state )
{
    if (state == STATE_SCANNING)
    {
        m_scan_bearing = m_bearing;
    }

    m_state = state;
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
    glm::vec2 cell = m_position + (m_sonar_dist + 0.00001f)*dir;

    int row = int(msg.yhit);
    int col = int(msg.xhit);

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
                // m_plan_srv.request.hostiles[i] = W*row + col;
            }
        }
    }
    // -----------------------------------------------------------


    if (uint8_t(msg.blocktype) == a3env::BLOCK_WALL)
    {
        m_worldview[W*row + col] = uint8_t(msg.blocktype);
    }


    for (float i=0.0f; i<=msg.distance; i+=1.0f)
    {
        glm::vec2 pos = m_position + i*dir;

        int r = int(pos.y);
        int c = int(pos.x);
    
        if (r < 0 || r >= 12 || c < 0 || c >= 12)
        {
            continue;
        }

        m_worldview[12*r + c] = uint8_t(a3env::BLOCK_AIR);
    }

    // std::cout << row << ", " << col << ": " << msg.blocktype << "\n";

    // for (int i=0; i<12; i++)
    // {
    //     for (int j=0; j<12; j++)
    //     {
    //         int n = int(m_worldview[12*i + j]);
        
    //         if (n == 0) std::cout << "? ";
    //         if (n == 1) std::cout << "  ";
    //         if (n == 2) std::cout << "# ";
    //         if (n == 4) std::cout << "X ";

    //         // std::cout << int(m_worldview[12*i + j]) << " ";
    //     }
    //     std::cout << "\n";
    // }
    // std::cout << "\n";



    update();
}


void
Agent::odom_callback( const a3env::odom &msg )
{
    m_position.x = msg.xpos;
    m_position.y = msg.ypos;
    // m_bearing    = msg.bearing;

    // std::cout << "m_position: " << m_position.x << ", " << m_position.y << "\n";


    int W   = a3env::MAP_WIDTH;
    int row = int(m_position.y);
    int col = int(m_position.x);

    m_worldview[W*row + col] = uint8_t(a3env::BLOCK_AIR);

    // update();
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

    int W = a3env::MAP_WIDTH;

    for (int i=0; i<W*W; i++)
    {
        m_plan_srv.request.world[i] = m_worldview[i];
    }

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


