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
    // Type 2 = TOTP + improved A star but without move after endpoint
    // Type 3 = TOTP + improved A star
    // Type 4 = Greedy Heuristic but without move after endpoint
    // Type 5 = Greedy Heuristic
    // Type 6 = Greedy Heuristic with Exchange without end
    // Type 7 = Greedy Heuristic with Exchange
    // Type 8 = Their TOTP + improved A star without allowing scheduled when used as endpoint by another agent
    // Type 9-20 = Decentralized

    bool allow_modification_endpoint = false;
    bool allow_move_after_endpoint = true;

    // General Methods
    int solve_AStar(Instance * instance, Agent * agent, int start_location, int goal_location,
                    int time_step_start);
    void updatePath(Agent * agent, const Node &goal);
    void Update_Path(Agent * agent, const Complex_Node & goal);
    void releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table);
    void releaseClosedListComplexNodes(map<string, Complex_Node*> &allNodes_table);
    bool isConstrained(Instance * instance, Agent * agent, int curr_id, int next_id, int next_timestep);
    void solve_TOTP(Instance * instance);
    void solve_multi_task_TOTP(Instance * instance);
    void solve_multi_task_TOTP_2(Instance * instance);
    bool compute_multi_task_path(Instance * instance, Agent * agent, vector<int> & list_id_task,
    int start_node);
    bool compute_multi_task_path_2(Instance * instance, Agent * agent, vector<int> & list_id_task,
                                 int start_node, bool apply_path);
    void solve_Greedy_Heuristic(Instance * instance);
    void solve_Decentralized(Instance * instance, int n_value = 0);
    void solve_Decentralized_ST_Focused(Instance * instance, int n_value = 0);
    void solve_Greedy_Heuristic_With_Exchange(Instance * instance);
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
    void compute_set_partitioning_assignment(Instance * instance, vector<Task *> & list_open_goals,
                                             vector<Agent *> & list_available_agents,
                                             vector<pair<int,int> > & chosen_assignments);
    void compute_path(Instance * instance, Agent * agent, vector<pair<int,int> > & list_tasks,int final_node);
    int get_h_value_next_goals(vector<int> & list_h_values_between_goals_to_reach, int current_index);
public:

    // General Methods
    void solve_instance(Instance * instance, int solver_id);

};

#endif //MAPD_V2_RESOLUTION_METHOD_H
