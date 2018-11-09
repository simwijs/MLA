//
// Created by Florian Grenouilleau on 2018-11-09.
//

#ifndef MAPD_V2_SET_PARTITIONING_HEURISTIC_H
#define MAPD_V2_SET_PARTITIONING_HEURISTIC_H

#include "Instance.h"
#include "Node.h"
#include <functional>
#include <map>
#include <utility>

using namespace std;

class Set_Partitioning_Heuristic {

private:

    // General Methods
    int solve_AStar(Instance * instance, Agent * agent, int start_location, int goal_location,
                    int time_step_start);
    void updatePath(Agent * agent, const Node &goal);
    void releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table);
    bool isConstrained(Instance * instance, Agent * agent, int curr_id, int next_id, int next_timestep);
    void solve_SP_Heuristic(Instance * instance);
    bool apply_TOTP(Instance * instance, Agent * agent);
    bool try_assignment(Instance * instance, Agent * agent, Task * task);
    bool compute_move_to_endpoint(Instance * instance, Agent * agent);
    bool compute_move_to_endpoint_active_agent(Instance * instance, Agent * agent);
    void check_if_agent_block_goals(Instance * instance);
    void compute_set_partitioning_assignment(Instance * instance, vector<Task *> & list_possible_tasks,
    vector<Agent *> & list_available_agents, vector<pair<int,int> > & chosen_assignments);

public:

    // General Methods
    void solve_instance(Instance * instance, int solver_id);

};

#endif //MAPD_V2_SET_PARTITIONING_HEURISTIC_H
