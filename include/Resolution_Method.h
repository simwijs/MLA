//
// Created by Florian Grenouilleau on 2018-10-31.
//

#ifndef MAPD_V2_RESOLUTION_METHOD_H
#define MAPD_V2_RESOLUTION_METHOD_H

#include "Instance.h"
#include "Node.h"
#include <functional>
#include <map>

using namespace std;

class Resolution_Method {

private:

    // General Methods
    int solve_AStar(Instance * instance, Agent * agent, int start_location, int goal_location,
                    int time_step_start);
    void updatePath(Agent * agent, const Node &goal);
    void releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table);
    bool isConstrained(Instance * instance, Agent * agent, int curr_id, int next_id, int next_timestep);
    void solve_TOTP(Instance * instance);
    void solve_Greedy_Heuristic(Instance * instance);
    void solve_Greedy_Heuristic_With_Update_Active_Agents(Instance * instance);
    void solve_Greedy_Heuristic_Wait(Instance * instance);
    void solve_Greedy_Heuristic_Task_First(Instance * instance);
    bool apply_TOTP(Instance * instance, Agent * agent);
    bool try_assignment(Instance * instance, Agent * agent, Task * task);
    bool compute_move_to_endpoint(Instance * instance, Agent * agent);
    bool compute_move_to_endpoint_active_agent(Instance * instance, Agent * agent);
    void check_if_agent_block_goals(Instance * instance);

public:

    // General Methods
    void solve_instance(Instance * instance, int solver_id);

};

#endif //MAPD_V2_RESOLUTION_METHOD_H
