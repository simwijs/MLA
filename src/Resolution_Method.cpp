//
// Created by Florian Grenouilleau on 2018-10-25.
//

#include "../include/Resolution_Method.h"
#include <iostream>
#include <vector>
#include <stdlib.h>

void Resolution_Method::solve_instance(Instance * instance, Solution * solution, int id_heuristic){

    // We check which heuristic we have to call
    if (id_heuristic == 1){

        // We apply the random heuristic
        this->apply_random_heuristic(instance,solution);
    }
    else {

        cout << "This id heuristic does not correspond to a known value" << endl;
    }
}

void Resolution_Method::apply_random_heuristic(Instance * instance, Solution * solution){

    // We initialize the values;
    int nb_task_scheduled = 0, current_time_step = -1;

    // While necessary
    while (nb_task_scheduled < instance->get_list_tasks().size()){

        // We increment the current time step
        ++ current_time_step;

        // We update the list of open tasks
        solution->update_list_open_tasks(current_time_step);

        // We copy the list of open tasks
        vector<int> copy_list_open_tasks;
        copy_list_open_tasks.insert(copy_list_open_tasks.begin(),solution->get_list_open_tasks().begin(),
                                    solution->get_list_open_tasks().end());

        // We get the list of available agents
        vector<int> list_available_agents;
        this->get_list_available_agents(solution,current_time_step,list_available_agents);

        // While there is a goal to test
        while (!copy_list_open_tasks.empty() && !list_available_agents.empty()){

            // We randomly select a task
            int rdm_task = rand() % copy_list_open_tasks.size();
            int id_current_task = copy_list_open_tasks[rdm_task];

            // We remove the task from the list
            copy_list_open_tasks.erase(copy_list_open_tasks.begin() + rdm_task);

            // We copy the list of available agents
            vector<int> copy_list_available_agents;
            copy_list_available_agents.insert(copy_list_available_agents.begin(), list_available_agents.begin(),
                                              list_available_agents.end());

            // We compute the positions matrix in the solution
            solution->compute_positions_matrix();

            // While the task is not assigned and there is an agent to test
            while (!copy_list_available_agents.empty() && true){

                // We randomly select an agent
                int rdm_agent = rand() % copy_list_available_agents.size();
                int id_current_agent = copy_list_available_agents[rdm_agent];

                // We remove the current agent from the list
                copy_list_available_agents.erase(copy_list_available_agents.begin() + rdm_agent);

                // We initialize the list of positions
                vector<Position> list_new_positions;

                // We check if the agent can do the task
                if (check_if_assignment_feasible(solution,id_current_task,id_current_agent,current_time_step,
                                                 list_new_positions)){

                    // We update the solution (new positions / list open tasks / update of task's dates)
                    solution->apply_assignment(id_current_task,id_current_agent,list_new_positions);

                    // We increment the number of scheduled task
                    ++ nb_task_scheduled;

                    // We remove the agent from the list of available ones
                    list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                     id_current_agent));

                    // We create the wait position for the agent
                    int current_size_list = solution->get_list_positions_per_time_step().size();

                    for (int k = current_time_step;
                         k <= current_size_list; ++k){

                        // We create the wait positions
                        solution->create_wait_positions(k);
                    }

                    // We update the positions matrix
                    solution->compute_positions_matrix();

                    cout << "Nb of task scheduled" << nb_task_scheduled << endl;

                    // We stop the search for this task
                    break;
                }
            }
        }

        // We create the wait for the current time step
        solution->create_wait_positions(current_time_step);
    }

    // We create the wait position for the agent
    int current_size_list = solution->get_list_positions_per_time_step().size();

    for (int k = current_time_step;
         k <= current_size_list; ++k){

        // We create the wait positions
        solution->create_wait_positions(k);
    }
}

bool Resolution_Method::check_if_assignment_feasible(Solution * solution, int id_task, int id_agent, int time_step,
                                  vector<Position> & list_new_positions){

    //cout << "Start of the check assignment " << endl;

    // We get the current position of the agent
    Position initial_position = solution->return_position(id_agent,time_step);

    // We search the shortest path until the pick up node
    if (search_path(solution,initial_position, id_task,
                    solution->get_instance()->get_list_tasks()[id_task]->get_pickup_node(), list_new_positions)){

        // We get the position of the agent
        Position position = list_new_positions[list_new_positions.size()-1];

        // We get the current size of the list
        int current_size = list_new_positions.size();

        // We search the shortest path to the delivery node
        if (search_path(solution,position,id_task,
                        solution->get_instance()->get_list_tasks()[id_task]->get_delivery_node(),
                        list_new_positions)){

            // We remove the doublon
            list_new_positions.erase(list_new_positions.begin()+current_size);

            // We update the assigned task value at the end of the path
            list_new_positions[list_new_positions.size()-1].set_assigned_task(-1);

            // We return true
            return true;
        }
        else {

            // We return false
            return false;
        }
    }
    else {

        // We return false
        return false;
    }
}

bool Resolution_Method::search_path(Solution * solution, Position & initial_position, int id_task, int id_node_goal,
                 vector<Position> & list_new_positions){

    //cout << "Start of the search path method" << endl;

    // We create the lists of nodes
    vector<Search_Node*> list_created_nodes;
    vector<Search_Node*> list_open_nodes, list_checked_nodes;
    vector<int> list_id_node_graph_visited_post_horizon;

    // We create the first search node
    list_created_nodes.push_back(new Search_Node (initial_position.get_id_node(),initial_position.get_time_step()));
    list_open_nodes.push_back(list_created_nodes[0]);

    // We initialize the boolean value
    bool goal_node_reached = false;

    // While a node can be expanded
    while (!list_open_nodes.empty()){

        // We get the first node
        Search_Node * current_node = list_open_nodes[0];

        // We remove it from the list
        list_open_nodes.erase(list_open_nodes.begin());

        // We add the node in the checked nodes list
        list_checked_nodes.push_back(current_node);

        // We check if the node is a goal node
        if (current_node->id_node_graph == id_node_goal){

            // We update the boolean value
            goal_node_reached = true;

            // We stop the process
            break;
        }

        // We expand the current node
        this->expand_node(solution,initial_position.get_id_agent(),current_node,list_open_nodes,list_created_nodes,
                          list_id_node_graph_visited_post_horizon);
    }

    // We check if the goal node has been reached
    if (goal_node_reached){

        // We build the corresponding path for the node
        build_path(solution,initial_position.get_id_agent(),id_task,list_checked_nodes,list_new_positions);

        // We clear the list of nodes created
        for (Search_Node * node : list_created_nodes){
            delete node;
            node = 0;
        }

        // We return true
        return true;
    }
    else {

        // We clear the list of nodes created
        for (Search_Node * node : list_created_nodes){
            delete node;
            node = 0;
        }

        //cout << "End of the search path method" << endl;

        // We return false
        return false;
    }
}

void Resolution_Method::expand_node(Solution * solution, int id_agent, Search_Node * node_to_expand,
                                    vector<Search_Node*> & list_open_nodes,vector<Search_Node*> & list_created_nodes,
                                    vector<int> & list_id_node_graph_visited_post_horizon){

    //cout << "Start  of the expand node method" << endl;

    // We check if we can add the current node to the list of checked after horizon
    if (node_to_expand->time_step >= solution->get_list_positions_per_time_step().size()){
        list_id_node_graph_visited_post_horizon.push_back(node_to_expand->id_node_graph);
    }

    // For each successor of the current node
    for (int id_successor : solution->get_instance()->get_list_nodes()[
            node_to_expand->id_node_graph]->get_list_id_nodes_successors()){

        // We initialize the boolean value
        bool expansion_feasible = true;

        // We check if we are still in the moving horizon
        if (node_to_expand->time_step >= solution->get_list_positions_per_time_step().size()){

            // We check if the successor has already been visited
            if (find(list_id_node_graph_visited_post_horizon.begin(),list_id_node_graph_visited_post_horizon.end(),
            id_successor) != list_id_node_graph_visited_post_horizon.end()){

                // We update the boolean value
                expansion_feasible = false;

                // We break the process
                continue;
            }
        }

        // We check if the expansion is still feasible
        if (!expansion_feasible) continue;

        // We check if the node is already in the open list
        for (Search_Node * search_node_to_check : list_open_nodes){

            // We check if the node corresponds
            if (search_node_to_check->id_node_graph == id_successor &&
                    search_node_to_check->time_step == node_to_expand->time_step+1) {

                // We update the boolean value
                expansion_feasible = false;

                // We break the process
                break;
            }
        }

        // We check if the expansion is still feasible
        if (!expansion_feasible) continue;

        // We check the vertex constraint
        for (int agent = 0; agent < solution->get_instance()->get_nb_agent(); ++agent){

            // We don't take into account the current agent
            if (agent == id_agent) continue;

            if (node_to_expand->time_step+1 < solution->get_list_positions_per_time_step().size()){

                if (solution->get_positions_matrix()[node_to_expand->time_step+1][agent] == id_successor){

                    // We update the boolean value
                    expansion_feasible = false;

                    // We break the process
                    break;
                }
            }
            else {

                // We check the last position of the agent
                if (solution->get_positions_matrix()[solution->get_list_positions_per_time_step().size()-1][agent]
                    == id_successor){

                    // We update the boolean value
                    expansion_feasible = false;

                    // We break the process
                    break;
                }
            }
        }

        // We check if the expansion is still feasible
        if (!expansion_feasible) continue;

        // We check if we are still in the moving horizon
        if (node_to_expand->time_step+1 < solution->get_list_positions_per_time_step().size()){

            // We check the edge constraint
            for (int agent = 0; agent < solution->get_instance()->get_nb_agent(); ++agent){

                // We don't take into account the current agent
                if (agent == id_agent) continue;

                if (solution->get_positions_matrix()[node_to_expand->time_step+1][agent]
                    == node_to_expand->id_node_graph &&
                        solution->get_positions_matrix()[node_to_expand->time_step][agent] == id_successor){

                    // We update the boolean value
                    expansion_feasible = false;

                    // We break the process
                    break;
                }
            }
        }

        // We check if the expansion is feasible
        if (expansion_feasible){

            // We create a new search node
            list_created_nodes.push_back(new Search_Node(id_successor,node_to_expand->time_step+1,
                                                    node_to_expand));

            // We add it in the open list
            list_open_nodes.push_back(list_created_nodes[list_created_nodes.size()-1]);
        }
    }

    //cout << "End of the expand node method" << endl;
}

void Resolution_Method::build_path(Solution * solution, int id_agent, int id_task,
                                   vector<Search_Node*> & list_checked_nodes, vector<Position> & list_new_positions){

    //cout << "Start of the build path method" << endl;

    // We get the initial size
    int initial_size = list_new_positions.size();

    // We get the final node of the path
    Search_Node * current_node = list_checked_nodes[list_checked_nodes.size()-1];

    // We add this node to the list
    list_new_positions.insert(list_new_positions.begin()+initial_size,
                              Position (id_agent,current_node->id_node_graph,current_node->time_step,id_task));

    while (current_node->parent_node != nullptr){

        // We update the current node
        current_node = current_node->parent_node;

        // We add the new position
        list_new_positions.insert(list_new_positions.begin()+initial_size,
                                  Position (id_agent,current_node->id_node_graph,current_node->time_step,id_task));

    }

    //cout << "End of the build path method" << endl;
}

void Resolution_Method::get_list_available_agents(Solution * solution, int current_time_step,
                                                  vector<int> & list_available_agents){

    // For each position of the current time step
    for (Position & position : solution->get_list_positions_per_time_step()[current_time_step]){

        // We check if the agent is assigned
        if (position.get_assigned_task() == -1){

            // If not, we add it in the list
            list_available_agents.push_back(position.get_id_agent());
        }
    }
}
