//
// Created by Florian Grenouilleau on 2018-10-31.
//

#ifndef MAPD_V2_RESOLUTION_METHOD_H
#define MAPD_V2_RESOLUTION_METHOD_H

#include "Instance.h"
#include "Node.h"
#include "Complex_Node.h"
#include <functional>
#include <map>

using namespace std;

class Resolution_Method {

private:

    // Attributes
    int solve_type = 1;
    // Type 1 = Their TOTP
    // Type 2 = TOTP + improved A star
    // Type 3 = TOTP + Improved A star + Allow modification when the agent goes to its endpoint
    // Type 4 = Greedy Heuristic
    // Type 5 = Greedy Heuristic + Allow modification when the agent goes to its endpoint
    // Type 6 = Greedy Heuristic + Wait
    // Type 7 = Greedy Heuristic + Modification of the number of agents

    bool allow_modification_endpoint = false;

    // General Methods
    int solve_AStar(Instance * instance, Agent * agent, int start_location, int goal_location,
                    int time_step_start);
    void updatePath(Agent * agent, const Node &goal);
    void Update_Path(Agent * agent, const Complex_Node & goal);
    void releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table);
    void releaseClosedListComplexNodes(map<string, Complex_Node*> &allNodes_table);
    bool isConstrained(Instance * instance, Agent * agent, int curr_id, int next_id, int next_timestep);
    void solve_TOTP(Instance * instance);
    void solve_Greedy_Heuristic(Instance * instance);
    void solve_Greedy_Heuristic_Wait(Instance * instance);
    bool apply_TOTP(Instance * instance, Agent * agent);
    bool apply_TOTP_2(Instance * instance, Agent * agent);
    bool compute_move_to_endpoint(Instance * instance, Agent * agent);
    bool check_if_assignment_feasible(Instance * instance, Agent * agent, Task * task);
    int return_h_value_endpoint(Instance * instance, Agent * agent, Task * task,vector<int> & list_endpoints_used);
    int compute_max_reach_pickup_node(Instance * instance, Agent * agent, Task * task,
                                       vector<int> & list_endpoints_used);
    int compute_max_reach_delivery_node(Instance * instance, Agent * agent, Task * task,
                                       vector<int> & list_endpoints_used);
public:

    // General Methods
    void solve_instance(Instance * instance, int solver_id);

};

#endif //MAPD_V2_RESOLUTION_METHOD_H
