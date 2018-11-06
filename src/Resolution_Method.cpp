//
// Created by Florian Grenouilleau on 2018-10-31.
//

#include "../include/Resolution_Method.h"
#include <queue>
#include <limits>
#include <utility>


void Resolution_Method::solve_instance(Instance * instance, int solver_id){

    if (solver_id == 1){

        // We solve the TOTP
        solve_TOTP(instance);
    }
    else if (solver_id == 2){

        // We solve the Greedy heuristic
        solve_Greedy_Heuristic(instance);
    }
}

void Resolution_Method::solve_TOTP(Instance * instance){

    //cout << "Solve the TOTP Algorithm" << endl;

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
            instance->get_current_time_step() <= instance->get_max_horizon()) {

        // pick the first agent in the lis
        Agent * current_agent = instance->get_agent(0);

        // We search for the first agent waiting at the time step t
        for (int i = 1; i < instance->get_nb_agent(); i++) {

            // We check if the time step corresponds
            if (instance->get_agent(i)->get_finish_time() == instance->get_current_time_step()) {
                current_agent = instance->get_agent(i);
                break;

            } else if (instance->get_agent(i)->get_finish_time() < current_agent->get_finish_time()) {
                current_agent = instance->get_agent(i);
            }
        }

        // We add the new tasks
        for (unsigned int i = instance->get_current_time_step(); i <= current_agent->get_finish_time(); i++) {
            if (instance->get_id_released_tasks_per_time_step()[i].empty()) continue;
            for (int id_task : instance->get_id_released_tasks_per_time_step()[i]) {
                if (find(instance->get_list_open_tasks().begin(),
                         instance->get_list_open_tasks().end(),
                         instance->get_list_tasks()[id_task]) == instance->get_list_open_tasks().end() &&
                        instance->get_list_tasks()[id_task]->get_id_assigned_agent() == -1){

                    instance->get_list_open_tasks().push_back(instance->get_task(id_task));
                }
            }
        }

        // We update the current time step of the instance
        instance->set_current_time_step(current_agent->get_finish_time());

        // We update the current location of the agent
        current_agent->set_current_location(current_agent->get_path()[instance->get_current_time_step()]);

        // We check if there is some tasks in the open list
        if (instance->get_list_open_tasks().empty())
        {
            // We update the finish time for the agent
            current_agent->set_finish_time(current_agent->get_finish_time() + 1);

            // We continue the process
            continue;
        }

        // We check if a TOTP solution has been found
        if (!this->apply_TOTP(instance,current_agent))
        {
            cout << "Problem, no assignment found for the current agent " << endl;
            getchar();
        }
    }

    //cout << "End of the TOTP Algorithm" << endl;
}

void Resolution_Method::solve_Greedy_Heuristic(Instance * instance){

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
           instance->get_current_time_step() <= instance->get_max_horizon()) {

        // We get the list of all the available agents for the current time step
        vector<Agent *> list_possible_agents;
        for (int i = 0; i < instance->get_nb_agent(); ++i) {

            // We check if the time step corresponds
            if (instance->get_agent(i)->get_finish_time() == instance->get_current_time_step()) {

                // We add the agent to the list of possible ones
                list_possible_agents.push_back(instance->get_agent(i));

            }
        }

        // We add the new tasks
        for (unsigned int i = instance->get_current_time_step(); i <= instance->get_current_time_step(); i++) {
            if (instance->get_id_released_tasks_per_time_step()[i].empty()) continue;
            for (int id_task : instance->get_id_released_tasks_per_time_step()[i]) {
                if (find(instance->get_list_open_tasks().begin(),
                         instance->get_list_open_tasks().end(),
                         instance->get_list_tasks()[id_task]) == instance->get_list_open_tasks().end() &&
                    instance->get_list_tasks()[id_task]->get_id_assigned_agent() == -1){

                    // We add the goal to the list of open ones
                    instance->get_list_open_tasks().push_back(instance->get_task(id_task));
                }
            }
        }

        // We check if the list of possible agents is empty
        if (!list_possible_agents.empty()){

            // We update the current locations of all the available agents
            for (Agent * agent : list_possible_agents){
                agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);
            }

            while(!list_possible_agents.empty() && !instance->get_list_open_tasks().empty()){

                // We initialize the values
                int min_h_value = std::numeric_limits<int>::max(), id_agent_min_h_value = -1;

                // For each agent
                for (Agent * agent : list_possible_agents){


                    // For each open task
                    for (Task * task : instance->get_list_open_tasks()){

                        // We get the h value
                        int current_h_value = instance->get_h_values_per_node()[
                                agent->get_current_location()][task->get_pickup_node()];

                        // We check the found value
                        if (current_h_value < min_h_value){

                            // We update the values
                            min_h_value = current_h_value;
                            id_agent_min_h_value = agent->get_id();
                        }

                    }
                }

                // We check if a TOTP solution has been found
                if (!this->apply_TOTP(instance,instance->get_agent(id_agent_min_h_value)))
                {
                    cout << "Problem, no assignment found for the current agent " << endl;
                    getchar();
                }
                else {

                    // We remove the selected agent from the list of available ones
                    list_possible_agents.erase(find(list_possible_agents.begin(),list_possible_agents.end(),
                                                    instance->get_agent(id_agent_min_h_value)));
                }
            }


            // We update the finish time for the agents
            for (Agent * agent : list_possible_agents){
                agent->set_finish_time(agent->get_finish_time() + 1);
            }

        }

        // We update the time step of the instance
        instance->set_current_time_step(instance->get_current_time_step() + 1);
    }

    cout << "End of the greedy heuristic" << endl;
}

bool Resolution_Method::apply_TOTP(Instance * instance, Agent * agent){

    // We update the current location for the agent
    agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);

    // We get the list of the used endpoints by the other agents
    vector<bool> hold(instance->get_nb_column()*instance->get_nb_row(), false);
    for (unsigned int i = 0; i < instance->get_nb_agent(); i++)
    {
        if (i != agent->get_id()) hold[instance->get_agent(i)->get_path()[instance->get_max_horizon() - 1]] = true;
    }

    // We sort the task by heuristic distance to the current location of the agent
    Task * task = NULL;
    for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
         it != instance->get_list_open_tasks().end(); it++)
    {

        // We check if the pick or delivery nodes are available
        if (hold[(*it)->get_pickup_node()] || hold[(*it)->get_delivery_node()]) {
            continue;
        }
            // We check if a value has been found
        else if (NULL == task) {
            task = (*it);
        }
            // We check if the heuristic value is better
        else if (instance->get_h_values_per_node()[(*it)->get_pickup_node()][agent->get_current_location()]
                 < instance->get_h_values_per_node()[task->get_pickup_node()][agent->get_current_location()])
        {
            // We update the values
            task = *it;
        }
    }

    // We check if no task has been found
    if (NULL == task) // No available tasks
    {
        // We check if the agent has to move from its current position
        bool move = false;
        for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
             it != instance->get_list_open_tasks().end(); it++) {

            // We check if the delivery location correponds with the agent's current location
            if ((*it)->get_delivery_node() == agent->get_current_location()) {
                move = true;
                break;
            }
        }

        // We check if a move is necessary
        if (move) {

            // We try to move the agent to another free endpoint
            if (compute_move_to_endpoint(instance,agent)) {

                // We return true
                return true;
            }
        }
        else {

            // We update the finish time of the agent
            agent->set_finish_time(agent->get_finish_time() + 1);

            // We return true
            return true;
        }

    }
    else {

        // We call the A start algorithm to the pickup location
        int arrive_start = solve_AStar(instance, agent, agent->get_current_location(), task->get_pickup_node(),
                                 agent->get_finish_time());

        // We check that the returned value is feasible
        if (arrive_start < 0)
        {
            cout << "Problem, the arrival start is equal to -1" << endl;
            getchar();
        }

        // We search the shortest path from the task's pickup node and the task's delivery node
        int arrive_goal = solve_AStar(instance, agent, task->get_pickup_node(), task->get_delivery_node(),
                                      arrive_start);

        // We check that the returned value is feasible
        if (arrive_goal < 0) //find a path to goal
        {
            cout << "Problem, the arrival goal is equal to -1" << endl;
            getchar();
        }

        // We update the agent's finish time
        agent->set_finish_time(arrive_goal);

        // We apply the assignment
        instance->apply_assignment(agent->get_id(), task->get_id(), arrive_start, arrive_goal);

        // We return true
        return true;
    }

    // We return false
    return false;
}

bool Resolution_Method::compute_move_to_endpoint(Instance * instance, Agent * agent){

    //BFS algorithm, choose the first empty endpoint to go to

    // We initialize the values
    queue<Node*> Q;
    map<unsigned int, Node*> allNodes_table; //key = g_val * map_size + loc

    // We create the list of possible actions
    int action[5] = {0, 1, -1, instance->get_nb_column(), -(instance->get_nb_column())};

    // We create the start node
    Node *start = new Node(agent->get_current_location(), 0, NULL, instance->get_current_time_step());

    // We add the start node to the lists
    allNodes_table.insert(make_pair(agent->get_current_location(), start)); //g_val = 0 --> key = loc
    Q.push(start);

    while (!Q.empty())
    {

        // We get the first node in the queue
        Node * current_node = Q.front();
        Q.pop();

        // We check if the maximal horizon is reached
        if (current_node->timestep >= instance->get_max_horizon() - 1) continue;

        // We check if the current node is an endpoint
        if (instance->get_list_endpoints()[current_node->loc])
        {
            // We initialize the boolean value
            bool occupied = false;

            // We check that during the following time step, the node is not used
            for (unsigned int t = current_node->timestep; t < instance->get_max_horizon() && !occupied; ++t)
            {
                // For each agent
                for (unsigned int agent_id = 0; agent_id < instance->get_nb_agent() && !occupied; ++agent_id)
                {
                    // We check if the node corresponds
                    if (agent_id != agent->get_id() &&
                            instance->get_agent(agent_id)->get_path()[t] == current_node->loc) {

                        // We update the boolean value
                        occupied = true;

                    }
                }
            }

            // We check if it's a goal node of an open task
            for (Task * task : instance->get_list_open_tasks())
            {
                if (task->get_delivery_node() == current_node->loc) {

                    // We update the boolean value
                    occupied = true;
                }
            }

            // We check if the bool value is true
            if (!occupied)
            {

                // We update the path of the agent to this node
                updatePath(agent,*current_node);

                // We update the finish time of the agent
                agent->set_finish_time(current_node->timestep);

                // We release the created nodes
                releaseClosedListNodes(allNodes_table);

                // We return true
                return true;
            }
        }

        // For each possible move
        for (int i = 0; i < 5; i++) {

            // We check if the move is feasible
            if (i == 1 && current_node->loc % (instance->get_nb_column()) == instance->get_nb_column() - 1) continue;
            if (i == 2 && current_node->loc % (instance->get_nb_column()) == 0) continue;
            if (i == 3 && current_node->loc / (instance->get_nb_column()) == instance->get_nb_row() - 1) continue;
            if (i == 4 && current_node->loc / (instance->get_nb_column()) == 0) continue;

            // We check if the node is available
            if (!isConstrained(instance, agent, current_node->loc, current_node->loc + action[i],
                               current_node->timestep + 1)) {

                // We check if the node has already been visited
                map<unsigned int, Node* >::iterator it = allNodes_table.find(
                        current_node->loc + action[i] + (current_node->g_val + 1) *
                                                                instance->get_nb_row()*instance->get_nb_column());

                if (it == allNodes_table.end()) {

                    // We create the new node
                    Node * new_node = new Node(current_node->loc + action[i], current_node->g_val + 1,
                                               current_node, current_node->timestep + 1);

                    // We add the new node to the lists
                    allNodes_table.insert(pair<unsigned int,
                            Node*>(new_node->loc + new_node->g_val * instance->get_nb_row() * instance->get_nb_column(),
                                    new_node));
                    Q.push(new_node);
                }
            }
        }
    }
    return false;
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
