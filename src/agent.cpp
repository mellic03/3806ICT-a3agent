#include "agent.hpp"

static std::vector<int> m_agent_positions(a3env::NUM_AGENTS, -1);


void
Agent::print_world()
{
    // Print worldview and plan
    // --------------------------------------------------------------------
    for (int i=0; i<a3env::MAP_WIDTH; i++)
    {
        for (int j=0; j<a3env::MAP_WIDTH; j++)
        {
            int n = int(m_worldview[a3env::MAP_WIDTH*i + j]);

            if (0) // (i == int(m_position.y) && j == int(m_position.x))
            {
                std::cout << "o ";
            }

            else if (n == 0) std::cout << "? ";
            else if (n == 1) std::cout << "  ";
            else if (n == 2) std::cout << "# ";
            else if (n == 3) std::cout << "s ";
        }
        std::cout << "\n";
    }

    std::cout << "Plan: ";
    for (int i=0; i<m_plan_srv.response.moves; i++)
    {
        std::cout << char(m_plan_srv.response.plan[i]) << " ";
    }
    std::cout << "\n\n";
    // --------------------------------------------------------------------

}


void
Agent::idle_behaviour()
{
    // Initial delay allowing agent to scan environment first
    // --------------------------------------------------------------------
    static bool first = true;
    static int count = 0;

    if (first)
    {
        if (count >= 256)
        {
            first = false;
        }
        count += 1;

        return;
    }
    // --------------------------------------------------------------------

    request_plan();
    print_world();

    m_state = STATE_FOLLOWING_PLAN;
}


void
Agent::follow_behaviour()
{
    if (m_path.empty())
    {
        set_state(STATE_IDLE);
        return;
    }


    glm::vec2 current = m_path.back();
    int row = int(current.y);
    int col = int(current.x);

    auto cell = m_worldview[a3env::MAP_WIDTH*row + col];

    if (cell == a3env::BLOCK_WALL)
    {
        set_state(STATE_IDLE);
        return;
    }

    else
    {
        m_worldview[a3env::MAP_WIDTH*row + col] = a3env::BLOCK_AIR;
    }

    if (glm::distance(m_position, current) <= 0.1f)
    {
        m_path.pop_back();
    }

    else
    {
        glm::vec2 dir = glm::normalize(current - m_position);
        m_bearing = atan2(dir.y, dir.x);
    }

    m_linear = 0.1f;
}


void
Agent::set_state( Agent::State state )
{
    if (state == STATE_IDLE)
    {
        m_linear = 0.0f;
    }

    m_state = state;
}



void
Agent::update()
{
    int row = int(m_position.y);
    int col = int(m_position.x);
    m_agent_positions[m_ID] = a3env::MAP_WIDTH*row + col; 

    // ROS_INFO("State: %d", m_state);

    switch (m_state)
    {
        case STATE_IDLE:            idle_behaviour();     break;
        case STATE_FOLLOWING_PLAN:  follow_behaviour();   break;
    }

    update_motors();
}



void
Agent::update_motors()
{
    a3env::motors m;

    m.agentid = m_ID;
    m.bearing = m_bearing;
    m.linear  = m_linear;

    m_motors_pub->publish(m);
}

void
Agent::sonars_callback( const a3env::sonars &msg )
{
    m_sonar_dist = msg.distance;

    glm::vec2 dir = glm::vec2(msg.dx, msg.dy);
    glm::vec2 cell = m_position + (m_sonar_dist + 0.00001f)*dir;

    int row = int(msg.yhit);
    int col = int(msg.xhit);

    constexpr int W = a3env::MAP_WIDTH;

    row = glm::clamp(row, 0, W-1);
    col = glm::clamp(col, 0, W-1);


    // Update known hostile locations if response is BLOCK_HOSTILE
    // -----------------------------------------------------------
    if (msg.blocktype == a3env::BLOCK_HOSTILE)
    {
        for (int i=0; i<a3env::NUM_HOSTILES; i++)
        {
            int idx = msg.data & (1 << i);

            if (msg.data & (1 << i))
            {
                m_hostiles[i] = uint16_t(W*row + col);
            }
        }
    }
    // -----------------------------------------------------------

    if (uint8_t(msg.blocktype) == a3env::BLOCK_WALL)
    {
        m_worldview[W*row + col] = msg.blocktype;
    }

    if (uint8_t(msg.blocktype) == a3env::BLOCK_SURVIVOR)
    {
        m_worldview[W*row + col] = msg.blocktype;
    }

    for (float i=0.0f; i<=msg.distance; i+=0.25f)
    {
        glm::vec2 pos = m_position + i*dir;

        int r = int(pos.y);
        int c = int(pos.x);
    
        if (r < 0 || r >= a3env::MAP_WIDTH || c < 0 || c >= a3env::MAP_WIDTH)
        {
            break;
        }

        int cell = m_worldview[a3env::MAP_WIDTH*r + c];

        if (cell == a3env::BLOCK_WALL)
        {
            break;
        }

        else if (cell == a3env::BLOCK_UNKNOWN)
        {
            m_worldview[a3env::MAP_WIDTH * r + c] = uint8_t(a3env::BLOCK_AIR);
        }
    }


    update();
}


void
Agent::odom_callback( const a3env::odom &msg )
{
    m_position.x = msg.xpos;
    m_position.y = msg.ypos;
    // m_bearing    = msg.bearing;

    int W   = a3env::MAP_WIDTH;
    int row = int(m_position.y);
    int col = int(m_position.x);

    m_worldview[W*row + col] = uint8_t(a3env::BLOCK_AIR);

    update();
}




void
Agent::request_plan()
{
    m_plan_srv.request.row = int(m_position.y);
    m_plan_srv.request.col = int(m_position.x);
    m_plan_srv.request.home_row = int(m_home.y);
    m_plan_srv.request.home_col = int(m_home.x);


    int W = a3env::MAP_WIDTH;

    for (int i=0; i<W*W; i++)
    {
        m_plan_srv.request.world[i] = m_worldview[i];
    }

    for (int i=0; i<a3env::NUM_AGENTS; i++)
    {
        m_plan_srv.request.agent_cells[i] = m_agent_positions[i];
    }

    for (int i=0; i<a3env::NUM_HOSTILES; i++)
    {
        m_plan_srv.request.hostile_cells[i] = m_hostiles[i];
    }

    if (!m_plan_client->call(m_plan_srv))
    {
        ROS_ERROR("Could not call service \"/a3planner/plan\"");
        return;
    }


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

        uint8_t cell = m_worldview[a3env::MAP_WIDTH*row + col];

        if (cell == a3env::BLOCK_UNKNOWN)
        {
            break;
        }

        m_path.push_back(current);
    }

    std::reverse(m_path.begin(), m_path.end());

}


