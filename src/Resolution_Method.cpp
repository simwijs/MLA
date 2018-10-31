//
// Created by Florian Grenouilleau on 2018-10-31.
//

#include "../include/Resolution_Method.h"

void Resolution_Method::solve_instance(Instance * instance){

}

int Resolution_Method::solve_AStar(Instance * instance, Agent * agent, int start_location, int goal_location,
                                   int time_step_start){

    // We initialize the values
    heap_open_t open_list;
    map<unsigned int, Node*> allNodes_table; // key = g_val*map_size + loc

    // generate start and add it to the OPEN list
    Node *start = new Node(start_location, 0, instance->get_h_values_per_node()[goal_location][start_location],
                           NULL, time_step_start, false);

    // We add the first node in the list
    open_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(make_pair(start_location, start)); //g_val=0 --> key=loc

    while (!open_list.empty())
    {
        // We take the first node in the list
        Node * current_node = open_list.top();
        open_list.pop();

        // We update the open list value for the current node
        current_node->in_openlist = false;

        // We check if we have reach the goal
        if (current_node->loc == goal_location)
        {

            // We initialize the boolean value
            bool hold = true;

            // We check if the agent can stay at this node until the end
            for (unsigned int i = current_node->timestep + 1; i < instance->get_max_horizon(); i++)
            {
                for (unsigned int j = 0; j < instance->get_nb_agent(); j++)
                {
                    // We check if the agent corresponds and if the vertex corresponds
                    if (j != agent->get_id() && current_node->loc == instance->get_agent(j)->get_path()[i])
                    {
                        hold = false;
                        break;
                    }
                }
            }

            if (hold) //if it can be held, then return the path
            {
                // We compute the found path
                updatePath(agent, *current_node);

                // We get the final time step
                int t = current_node->timestep;

                // We release the created nodes
                releaseClosedListNodes(allNodes_table);

                // We return the found value
                return t;
            }
            // else, keep searching
        }

        // check timestep
        if (current_node->timestep >= instance->get_max_horizon() - 1) continue;

        // We intialize the successor id
        int successor_id;

        // We create the list of possible moves
        int action[5] = {0, 1, -1, instance->get_nb_column(), -(instance->get_nb_column())};

        for (int i = 0; i < 5;i++)
        {

            // We check if the move is feasible
            if (i == 1 && current_node->loc % (instance->get_nb_column()) == instance->get_nb_column() - 1) continue;
            if (i == 2 && current_node->loc % (instance->get_nb_column()) == 0) continue;
            if (i == 3 && current_node->loc / (instance->get_nb_column()) == instance->get_nb_row() - 1) continue;
            if (i == 4 && current_node->loc / (instance->get_nb_column()) == 0) continue;

            // We get the location of the successor
            successor_id = current_node->loc + action[i];

            // We initialize the current time step value
            int next_timestep = current_node->timestep + 1;

            // We check if the successor is accessible
            if (!isConstrained(instance, agent, current_node->loc, successor_id, next_timestep))
            {
                // We compute the successor values
                int next_g_val = current_node->g_val + 1;
                int next_h_val = instance->get_h_values_per_node()[goal_location][successor_id];

                // We generate the corresponding node
                Node * next=new Node(successor_id, next_g_val, next_h_val, current_node, next_timestep, false);

                // We check if the current successor has been checked before
                map<unsigned int, Node* >::iterator it = allNodes_table.find(
                        next->loc + next->g_val * instance->get_nb_row() * instance->get_nb_column());

                if (it == allNodes_table.end()) //undiscovered
                {  // add the newly generated node to open_list and hash table

                    // We udate open list value for the node
                    next->in_openlist = true;

                    // We insert the current node in the list of all the nodes
                    allNodes_table.insert(pair<unsigned int, Node*>(
                            next->loc + next->g_val * instance->get_nb_row() * instance->get_nb_column(), next));

                    // We add the node to the open list
                    open_list.push(next);
                }

                else //discovered
                {
                    // We delete the created node
                    delete(next);
                }
            }
        }
    }

    // We release all the created node
    releaseClosedListNodes(allNodes_table);

    // We return the default value
    return -1;
}

void Resolution_Method::updatePath(Agent * agent, const Node & goal) {

    // We update the path of the agent until the end of the horizon
    for (int i = goal.timestep + 1; i < agent->get_path().size(); i++)
    {
        agent->get_path()[i] = goal.loc;
    }

    // We search for the path for the agent
    const Node* current_node = &goal;
    while (current_node!=NULL)
    {
        // We update the agent path
        agent->get_path()[current_node->timestep] = current_node->loc;

        // We get the parent node
        current_node = current_node->parent;
    }
}

bool Resolution_Method::isConstrained(Instance * instance, Agent * agent, int curr_id, int next_id, int next_timestep) {

    // We check if the node is reachable
    if (!instance->get_list_map_nodes()[next_id]) return true;

    // We check the vertex and edge constraints
    for (int current_agent = 0; current_agent < instance->get_nb_agent(); current_agent++)
    {
        // We check if the agents' ids correspond
        if (current_agent == agent->get_id()) {
            continue;
        }

            // We check the vertex constraint
        else if (instance->get_agent(current_agent)->get_path()[next_timestep] == next_id) {

            // We return true
            return true;
        }

            // We check the edge constraint
        else if (instance->get_agent(current_agent)->get_path()[next_timestep - 1] == next_id &&
                instance->get_agent(current_agent)->get_path()[next_timestep] == curr_id) {

            // We return true
            return true;
        }
    }

    // We return false
    return false;
}

void Resolution_Method::releaseClosedListNodes(map<unsigned int, Node*> & allNodes_table){

    // We initialize the iterator
    map<unsigned int, Node*>::iterator it;

    // For each node in the table
    for (it = allNodes_table.begin(); it != allNodes_table.end(); it++){

        // We delete the node
        delete ((*it).second);
    }

    // We clear the table
    allNodes_table.clear();
}
