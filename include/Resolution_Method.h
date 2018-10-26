//
// Created by Florian Grenouilleau on 2018-10-25.
//

#ifndef MAPD_RESOLUTION_METHOD_H
#define MAPD_RESOLUTION_METHOD_H

#include "Solution.h"
#include "Search_Node.h"

using namespace std;

class Resolution_Method {

private:

    // General methods
    void apply_random_heuristic(Instance * instance, Solution * solution);
    void get_list_available_agents(Solution * solution, int current_time_step, vector<int> & list_available_agents);
    bool check_if_assignment_feasible(Solution * solution, int id_task, int id_agent, int time_step,
                                      vector<Position> & list_new_positions);
    bool search_path(Solution * solution, Position & initial_position, int id_task, int id_node_goal,
                     vector<Position> & list_new_positions);
    void expand_node(Solution * solution, int id_agent, Search_Node & node_to_expand,
                     vector<Search_Node> & list_open_nodes,vector<Search_Node> & list_checked_nodes);
    void build_path(Solution * solution, int id_agent, int id_task,
                    vector<Search_Node> & list_checked_nodes, vector<Position> & list_new_positions);
public:

    // Constructor
    Resolution_Method(){};

    // General methods
    void solve_instance(Instance * instance, Solution * solution, int id_heuristic);
};

#endif //MAPD_RESOLUTION_METHOD_H
