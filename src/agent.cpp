#include "agent.hpp"

static std::vector<int> m_agent_positions(a3env::NUM_AGENTS, -1);
static std::vector<int> m_worldview(a3env::MAP_WIDTH*a3env::MAP_WIDTH, 0);
static ros::Publisher  *m_deactivate_pub;


Agent::Agent( int id, ros::ServiceClient *plan_client, ros::Publisher *motors_pub,
              ros::Publisher *deactivate_pub )
:   m_ID              (id),
    m_plan_client     (plan_client),
    m_motors_pub      (motors_pub),
    m_state           (STATE_IDLE),
    m_hostiles        (a3env::NUM_HOSTILES, std::numeric_limits<uint16_t>::max())
{
    m_plan_srv.request.world.resize(m_worldview.size());
    m_plan_srv.request.agent_cells.resize(a3env::NUM_AGENTS);
    m_plan_srv.request.hostile_cells.resize(a3env::NUM_AGENTS);

    m_deactivate_pub = deactivate_pub; // last-minute spaghett.

    const int W = a3env::MAP_WIDTH;

    for (int i=0; i<a3env::MAP_WIDTH; i++)
    {
        m_worldview[W*i + 0] = a3env::BLOCK_WALL;
        m_worldview[W*0 + i] = a3env::BLOCK_WALL;
        m_worldview[W*i + (W-1)] = a3env::BLOCK_WALL;
        m_worldview[W*(W-1) + i] = a3env::BLOCK_WALL;
    }
}




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
    if (first_plan == true)
    {
        if (first_plan_count >= 256)
        {
            first_plan = false;
        }
        first_plan_count += 1;

        return;
    }
    // --------------------------------------------------------------------

    request_plan();
    print_world();

    if (m_plan_srv.response.moves == 0)
    {
        set_state(STATE_FINISHED);
    }

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

    int h = hostile_at_cell(row, col);
    if (h != -1)
    {
        set_state(STATE_IDLE);
        return;
    }

    int a = agent_at_cell(row, col);
    if (a != -1 && a != m_ID)
    {
        set_state(STATE_IDLE);
        return;
    }

    // else
    // {
    //     m_worldview[a3env::MAP_WIDTH*row + col] = a3env::BLOCK_AIR;
    // }

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

    if (state == STATE_FINISHED)
    {
        a3env::deactivate d;
        d.agentid = m_ID;

        m_deactivate_pub->publish(d);
    }

    m_state = state;
}



void
Agent::update()
{
    if (m_state == STATE_FINISHED)
    {
        return;
    }


    int row = int(m_position.y);
    int col = int(m_position.x);
    m_agent_positions[m_ID] = a3env::MAP_WIDTH*row + col; 


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
    if (m_state == STATE_FINISHED)
    {
        return;
    }

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

        int h = hostile_at_cell(r, c);
        if (h != -1)
        {
            m_hostiles[h] = std::numeric_limits<uint16_t>::max();
        }

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


int
Agent::hostile_at_cell( int row, int col )
{
    for (int i=0; i<a3env::NUM_HOSTILES; i++)
    {
        if (m_hostiles[i] != std::numeric_limits<uint16_t>::max())
        {
            if (m_hostiles[i] == a3env::MAP_WIDTH*row + col)
            {
                return i;
            }
        }
    }

    return -1;
}


int
Agent::agent_at_cell( int row, int col )
{
    for (int i=0; i<a3env::NUM_AGENTS; i++)
    {
        if (m_agent_positions[i] != std::numeric_limits<uint16_t>::max())
        {
            if (m_agent_positions[i] == a3env::MAP_WIDTH*row + col)
            {
                return i;
            }
        }
    }

    return -1;
}


void
Agent::odom_callback( const a3env::odom &msg )
{
    if (m_state == STATE_FINISHED)
    {
        return;
    }

    m_position.x = msg.xpos;
    m_position.y = msg.ypos;
    // m_bearing    = msg.bearing;

    if (first_odom)
    {
        m_home.x = msg.xpos;
        m_home.y = msg.ypos;
        first_odom = false;
    }

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

    // std::cout << "Agents: ";
    for (int i=0; i<a3env::NUM_AGENTS; i++)
    {
        // std::cout << m_agent_positions[i] << " ";
        m_plan_srv.request.agent_cells[i] = m_agent_positions[i];
    }
    // std::cout << "\n";

    // std::cout << "Hostiles: ";
    for (int i=0; i<a3env::NUM_HOSTILES; i++)
    {
        // std::cout << m_hostiles[i] << " ";
        m_plan_srv.request.hostile_cells[i] = m_hostiles[i];
    }
    // std::cout << "\n";

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

        m_path.push_back(current);
    }

    std::reverse(m_path.begin(), m_path.end());

}


