//
// Created by Florian Grenouilleau on 2018-10-25.
//
#include "../include/Instance.h"
#include <queue>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <bits/stdc++.h>

void Instance::compute_h_values(vector<int> & h_values, int start_location) {

    // We initialize the values
    queue<int> Q;
    vector<bool> status(this->list_map_nodes.size(), false);
    h_values.resize(this->list_map_nodes.size(),-1);
    int neighbor[4] = {1,-1,this->nb_column,-(this->nb_column)};

    // We update the values for the start location
    status[start_location] = true;
    h_values[start_location] = 0;

    // We add the start location to the queue
    Q.push(start_location);

    while (!Q.empty())
    {
        // We get the first value
        int v = Q.front();
        Q.pop();

        // For each neighbor
        for (int i = 0; i < 4; i++)
        {
            // We check that the neighbor is reachable
            if (i == 0 && v % (this->nb_column) == this->nb_column-1) continue;
            if (i == 1 && v % (this->nb_column) == 0) continue;
            if (i == 2 && v / (this->nb_column) == this->nb_row-1) continue;
            if (i == 3 && v / (this->nb_column) == 0) continue;

            // We get the corresponding value
            int u = v + neighbor[i];

            // We check if this neighbor is accessible
            if (this->list_map_nodes[u])
            {

                // We check if the new value has been checked
                if (!status[u])
                {
                    // We update the values and add the successor to the queue
                    status[u] = true;
                    h_values[u] = h_values[v] + 1;
                    Q.push(u);
                }
            }
        }
    }
}

void Instance::update_release_tasks_per_time_step(){

    // We resize the matrix
    this->id_released_tasks_per_time_step.resize(this->max_horizon,vector<int>());

    // For each task
    for (Task * task : this->list_tasks){

        // We add the task at its time step
        this->id_released_tasks_per_time_step[task->get_release_date()].push_back(task->get_id());
    }
}

bool Instance::check_solution_feasible(){

    // We check that each task has an agent
    for (Task * task : this->list_tasks){

        // We check if the task is assigned
        if (task->get_id_assigned_agent() == -1){

            task->write();
            cout << "Problem, the task is not assigned" << endl;
            return false;
        }

        // We check that the pickup and delivery dates are feasible
        if (task->get_picked_date() >= task->get_delivered_date() ||
                task->get_picked_date() == -1 || task->get_delivered_date() == -1){

            task->write();
            cout << "Problem, the pickup and/or delivery dates are not feasible" << endl;
            return false;
        }

        // We check that the dates correspond to the assigned agent's path
        if (this->list_agents[task->get_id_assigned_agent()]->get_path()[task->get_picked_date()] !=
                task->get_pickup_node() ||
            this->list_agents[task->get_id_assigned_agent()]->get_path()[task->get_delivered_date()] !=
                task->get_delivery_node()){

            cout << "Problem, the dates do not correspond to the agent's path" << endl;
            return false;
        }
    }

    // We check that the agent's path is feasible
    for (Agent * agent : this->list_agents){

        for (int time_step = 0; time_step < this->max_horizon; ++time_step){

            // We check that the current node of the agent is an available one
            if (!this->list_map_nodes[agent->get_path()[time_step]]){

                cout << "Problem, the current node of the agent is not allowed" << endl;
                return false;
            }

            // We check that the final position of the agent is an endpoint
            if (!this->list_endpoints[agent->get_path()[this->max_horizon-1]]){

                cout << "Problem, the final position of the agent is not an endpoint" << endl;
            }

            if (time_step < this->max_horizon - 1){

                // We get the current node of the agent
                int current_node = agent->get_path()[time_step];

                // We get the next node of the agent
                int next_node = agent->get_path()[time_step];

                // We check that the move is feasible
                if (current_node + 1 != next_node &&
                        current_node - 1 != next_node &&
                        current_node + this->nb_column != next_node &&
                        current_node - this->nb_column != next_node &&
                        current_node != next_node){

                    cout << "Problem, the move is not feasible for the agent" << endl;
                    return false;
                }
            }
        }
    }

    // We check the vertex constraint
    for (int agent_1 = 0; agent_1 < this->nb_agent - 1; ++agent_1){

        for (int agent_2 = agent_1 + 1; agent_2 < this->nb_agent; ++agent_2){

            // For each time step
            for (int time_step = 0; time_step < this->max_horizon; ++time_step){

                // We check that their positions are not the same
                if (this->list_agents[agent_1]->get_path()[time_step] ==
                        this->list_agents[agent_2]->get_path()[time_step]){

                    cout << "Problem the agents use the same vertex" << endl;
                    cout << "Agent : " << agent_1 << endl;
                    cout << "Agent : " << agent_2 << endl;
                    cout << "Time step : " << time_step << endl;
                    return false;
                }
            }
        }
    }

    // We check the edge constraint
    for (int agent_1 = 0; agent_1 < this->nb_agent - 1; ++agent_1){

        for (int agent_2 = agent_1 + 1; agent_2 < this->nb_agent; ++agent_2){

            // For each time step
            for (int time_step = 0; time_step < this->max_horizon - 1; ++time_step){

                // We get the current node
                int current_node_1 = this->list_agents[agent_1]->get_path()[time_step];
                int current_node_2 = this->list_agents[agent_2]->get_path()[time_step];

                // We get the next nodes
                int next_node_1 = this->list_agents[agent_1]->get_path()[time_step + 1];
                int next_node_2 = this->list_agents[agent_2]->get_path()[time_step + 1];

                // We check that the edge constraint is respected
                if (current_node_1 == next_node_2 && current_node_2 == next_node_1){

                    cout << "Problem, the edge constraint is not respected" << endl;
                    return false;
                }
            }
        }
    }

    // We return true
    return true;
}

void Instance::apply_assignment(int id_agent, int id_task, int arrive_start, int arrive_goal){


    //cout << "Assign the task " << id_task << " to the agent " << id_agent << " from the time step " <<
         //arrive_start << " to the time step " << arrive_goal << endl;

    Task* task = this->list_tasks[id_task];
    // We update the assigned agent
    task->set_id_assigned_agent(id_agent);

    // We update the dates
    task->set_picked_date(arrive_start);
    task->set_delivered_date(arrive_goal);

    // Try finishing the batch the task belongs to
    this->batches[task->get_batch_id()]->try_finish();

    // We remove the task for the open tasks' list
    this->get_list_open_tasks().erase(find(this->list_open_tasks.begin(),
                                           this->list_open_tasks.end(),
                                          task));

    // We check that the agent's path correspond
    if (this->list_agents[id_agent]->get_path()[arrive_start] !=task->get_pickup_node() ||
            this->list_agents[id_agent]->get_path()[arrive_goal] !=task->get_delivery_node()){

        cout << "Problem, the agent's path does not correspond" << endl;
    }

    // We increment the number of scheduled task
    ++ this->nb_task_scheduled;

    //cout << "End Assignment " << endl;
}

void Instance::compute_final_makespan(){

    int max_makespan = 0;

    for (Agent * agent : this->list_agents){

        // We get the end location of the agent
        int end_location = agent->get_path()[this->max_horizon-1];
        int current_time_step_agent = max_horizon - 1;

        // We go backward until the position change
        while (true && current_time_step_agent >= 0){

            if (end_location == agent->get_path()[current_time_step_agent - 1]){

                // We update the values
                -- current_time_step_agent;
                end_location = agent->get_path()[current_time_step_agent];
            }
            else {

                // We stop the process
                agent->set_finish_time(current_time_step_agent);
                break;
            }
        }

        // We update the max makespan if necessary
        if (agent->get_finish_time() > max_makespan){
            max_makespan = agent->get_finish_time();
        }
    }

    // We update the current time step value
    this->current_time_step = max_makespan;
}

double Instance::compute_average_service_time(){

    // We initialize the sum
    double sum = 0;

    for (Task * task : this->list_tasks){

        sum += task->get_delivered_date() - task->get_release_date();
    }

    return sum / (double) this->list_tasks.size();
}

double Instance::compute_average_batch_service_time() {
    int total = 0;
    int size = batches.size();
    for (auto b : batches) {
        total += b->get_service_time();
    }

    return total / (double) size;
}

double Instance::compute_min_batch_service_time() {
    int min = INT_MAX;
    for (auto b : batches) {
        int st = b->get_service_time();
        if (st < min) {
            min = st;
        }
    }
    return min;
}

double Instance::compute_max_batch_service_time() {
    int max = 0;
    for (auto b : batches) {
        int st = b->get_service_time();
        if (st > max) {
            max = st;
        }
    }
    return max;
}


double Instance::compute_average_impact_traffic(){

    // We initialize the sum
    double sum = 0;

    // For each task of the problem
    for (Task * task : this->list_tasks){

        // We add the difference between the h value and the real value
        sum += (task->get_delivered_date()-task->get_picked_date()) -
                h_values_per_node[task->get_pickup_node()][task->get_delivery_node()];
    }

    // We return the average of the computed sum
    return sum / (double) this->list_tasks.size();
}

double Instance::compute_average_nb_agent_avail(){

    if (this->nb_agent_available_per_time_step.empty()){
        return 0;
    }

    // We initialize the sum
    double sum = 0;

    // For each task of the problem
    for (int value : this->nb_agent_available_per_time_step){

        sum += value;
    }

    // We return the average of the computed sum
    return sum / (double) this->nb_agent_available_per_time_step.size();
}

double Instance::compute_max_service_time(){

    double max_value = 0;

    for (Task * task : this->list_tasks){

        if (task->get_delivered_date() - task->get_release_date() > max_value){

            max_value = task->get_delivered_date() - task->get_release_date();
        }
    }

    return max_value;
}

double Instance::compute_ninth_decile_service_time(){

    vector<int> values;

    for (Task * task : this->list_tasks){

        values.push_back(task->get_delivered_date() - task->get_release_date());

    }

    sort(values.begin(),values.end());
    reverse(values.begin(),values.end());

    double sum = 0;

    for (int k = 0; k < this->list_tasks.size()/10; ++k){

        sum += values[k];
    }

    return sum / (double) (this->list_tasks.size()/10);

}

void Instance::generate_agents(int nb_agent_to_generate){

    // We clear the list of agents
    this->list_agents.clear();

    // We update the nb agent for the instance
    this->nb_agent = nb_agent_to_generate;

    // We initialize the vector of possible endpoints
    vector<int> list_possible_endpoints;

    // We get the list of possible endpoints
    for (int ep = 0; ep < nb_row*nb_column; ++ep){
        if (this->list_endpoints[ep]){
            list_possible_endpoints.push_back(ep);
        }
    }

    // We create the agents
    for (int k = 0; k < nb_agent_to_generate; ++k){

        // We randomly select an index in the list of possible endpoints
        int rdm_index = rand() % list_possible_endpoints.size();

        // We get the corresponding node
        int rdm_node = list_possible_endpoints[rdm_index];

        // We remove the node from the list
        list_possible_endpoints.erase(list_possible_endpoints.begin() + rdm_index);

        // We create the associated agent
        this->list_agents.push_back(new Agent(this->list_agents.size(),rdm_node,this->max_horizon));
    }
}

void Instance::show_h_value_between_tasks_per_agent(){

    // For each agent of the problem
    for (Agent * agent : this->list_agents){

        // We initialize the list of tasks for the agent
        vector<pair<int,int> > list_tasks_for_current_agent;

        // For each task of the problem
        for (Task * task : this->list_tasks){

            // We check if the id of the agent corresponds
            if (task->get_id_assigned_agent() == agent->get_id()){

                // We add the task in the list
                list_tasks_for_current_agent.push_back(pair<int,int> (task->get_picked_date(),task->get_id()));
            }
        }

        // We sort the tasks per picked date
        sort(list_tasks_for_current_agent.begin(),list_tasks_for_current_agent.end());

        // For each task of the list
        for (int i = 0; i < list_tasks_for_current_agent.size()-1; ++i){

            // We get the current task
            Task * current_task = this->list_tasks[list_tasks_for_current_agent[i].second];

            // We get the next task
            Task * next_task = this->list_tasks[list_tasks_for_current_agent[i+1].second];

            // We compute the h value
            int current_h_value = this->h_values_per_node[current_task->get_delivery_node()][next_task->get_pickup_node()];

            // We print out the current h value
            cout << "H value between tasks : " << current_h_value << endl;
        }
    }

    getchar();
}

void Instance::output_solution(char** argv){

    // We open the existing file
    fstream file;
    file.open ("Instances_Summary.txt", fstream::in | fstream::out | fstream::app);

    // We add the values
    file << this->get_map_file_name() << ";";
    file << this->get_task_file_name() << ";";
    file << this->get_nb_agent() << ";" ;
    file << this->get_list_tasks().size() << ";";
    file << this->get_current_time_step() << ";";
    file << this->computation_time << ";";
    file << this->compute_average_service_time() << ";";
    file << this->compute_average_batch_service_time() << ";";
    file << this->compute_min_batch_service_time() << ";";
    file << this->compute_max_batch_service_time() << ";";
    file << argv[3] << ";";
    file << this->wait_value << ";";
    file << compute_average_impact_traffic() << ";";
    file << this->nb_created_search_nodes << ";";
    file << this->nb_checked_search_nodes << ";";
    file << this->compute_max_service_time() << ";";
    file << this->compute_ninth_decile_service_time() << ";";
    file << this->max_distance_multi_task << ";";
    file << this->max_size_multi_task << ";";

    file << endl;

    // We close the file
    file.close();
}

void Instance::output_map_for_visualization(){

    // We open the existing file
    fstream file;
    file.open ("map.yaml", fstream::out);

    // We write the agents
    file << "agents:" << endl;
    for (Agent * agent : this->list_agents){
        file << "-   goal: [" << agent->get_path()[0]/nb_column << "," << agent->get_path()[0]%nb_column << "]" << endl;
        file << "    name: agent" << agent->get_id() << endl;
        file << "    start: [0, 0]" << endl;
    }

    // We write the obstacles
    file << "map:" << endl;

    // We write the dimensions
    file << "    dimensions: [" << nb_row << ", " << nb_column << "]" << endl;

    // We write the obstacles
    file << "    obstacles:" << endl;
    // For each node of the map
    for (int k = 0; k < nb_column*nb_row; ++k){
        if (!this->list_map_nodes[k]){
            // We add this node in the list of obstacles
            file << "    - !!python/tuple [" << k / nb_column << ", " << k % nb_column << "]" << endl;
        }
    }

    // We write the endpoints
    file << "    endpoints:" << endl;
    // For each node of the map
    for (int k = 0; k < nb_column*nb_row; ++k){
        if (this->list_endpoints[k]){
            // We add this node in the list of obstacles
            file << "    - !!python/tuple [" << k / nb_column << ", " << k % nb_column << "]" << endl;
        }
    }
}

void Instance::output_moves_for_visualization(std::string filepath){

    // We open the existing file
    fstream file;
    file.open (filepath, fstream::out);

    // We write the agents
    file << "schedule:" << endl;
    for (Agent * agent : this->list_agents){
        file << "  agent" << agent->get_id() << ":" << endl;
        for (int ts = 0; ts <= this->get_current_time_step(); ++ts){
            file << "    - x: " << agent->get_path()[ts]/nb_column << endl;
            file << "      y: " << agent->get_path()[ts]%nb_column << endl;
            file << "      t: " << ts << endl;
        }
    }
    file.close();
}

void Instance::output_tasks_for_visualization(std::string filepath) {
    fstream file;
    file.open (filepath, fstream::out);
    file << "tasks:" << endl;
    for (int t = 0; t < this->list_tasks.size(); t++) {
        Task* task = this->list_tasks[t];
        file << "   task" << t << ":" << endl;

        file << "   - start: " << task->get_picked_date() << endl;;
        file << "     finish: " << task->get_delivered_date() << endl;
        file << "     batch: " << task->get_batch_id() << endl;
    }

    file.close();
}

void Instance::create_instances_first_set(int nb_agent_for_map,int nb_task_for_instance, double frequency_for_instance){

    // We create the copy of the list of pairs
    vector<pair<int,int> > copy_list_pairs;
    for (pair<int,int> & pair_to_copy : this->list_pair_node_endpoint){
        copy_list_pairs.push_back(pair<int,int> (pair_to_copy.first,pair_to_copy.second));
    }

    // For each agent of the problem
    for (Agent * agent : this->list_agents) {

        // We add the agent's location to the send of possible locations
        copy_list_pairs.push_back(pair<int,int> (
                copy_list_pairs.size(),agent->get_path()[0]));
    }

    // We open the existing file
    fstream file;
    int frequency_to_write = frequency_for_instance*10;
    file.open ("./Instances_First_Set/map_"+ to_string(nb_agent_for_map) + "_" + to_string(nb_task_for_instance)
               + "_" + to_string(frequency_to_write) + ".map", fstream::out);

    // We write the size of the map
    file << this->nb_row << " " << this->nb_column << endl;

    // We write the number of endpoints
    file << (copy_list_pairs.size()-nb_agent_for_map) << endl;

    // We write the number of agents
    file << to_string(nb_agent_for_map) << endl;

    // We write the default value
    file << this->list_agents[0]->get_path().size() << endl;

    // We initialize the list of positions for the agents
    vector<int> positions_agents, indexes_agents;

    // We randomly define the positions of the agents
    for (int k = 0; k < nb_agent_for_map; ++k){

        while(true){

            // We randomly choose a node
            int rdm_node = rand() % (copy_list_pairs.size());

            // We get the associated grid node
            int current_value = copy_list_pairs[rdm_node].second;

            // We check that the node is not already in the lise
            if (find(positions_agents.begin(), positions_agents.end(), current_value) !=
                    positions_agents.end()) continue;

            // We add the node in the list
            positions_agents.push_back(current_value);

            // We add the index in the list
            indexes_agents.push_back(rdm_node);

            // We break the process for the current agent
            break;
        }
    }

    // We initialize the current index of the node
    int current_index = -1;

    // We write the map
    for (int row = 0; row < this->nb_row; ++row){

        for (int column = 0; column < this->nb_column; ++column){

            // We increment the current index
            ++ current_index;

            // We check if the current node is a map node
            if (!this->list_map_nodes[current_index]){

                // We write the obstacle value
                file << "@";
            }
            else {

                // We check if the node is an endpoint
                if(!this->list_endpoints[current_index]){

                    // We write the non-endpoint value
                    file << ".";
                }
                else {

                    // We check if the node is in the list (robot start point)
                    if (find(positions_agents.begin(),positions_agents.end(),current_index) != positions_agents.end()){

                        // We write the robot value
                        file << "r";
                    }
                    else {

                        // We write the endpoint value
                        file << "e";
                    }
                }
            }
        }

        // We go to the next line
        file << endl;
    }

    // We sort the agents' locations
    sort(indexes_agents.begin(),indexes_agents.end());
    reverse(indexes_agents.begin(),indexes_agents.end());

    // We remove the corresponding values in the list of pairs
    for (int value : indexes_agents){

        // We remove the corresponding value
        copy_list_pairs.erase(copy_list_pairs.begin()+value);
    }

    // We create/open the file
    fstream file_task;
    file_task.open ("./Instances_First_Set/task_"+ to_string(nb_agent_for_map) + "_" + to_string(nb_task_for_instance)
               + "_" + to_string(frequency_to_write) + ".task", fstream::out);

    // We write the number of task
    file_task << nb_task_for_instance << endl;

    // We initialize the list of pickup and delivery point per task
    vector<pair<int,int> > pickup_delivery_node_per_task;

    // We define the pickup and delivery point for each task of the problem
    for (int k = 0; k < nb_task_for_instance; ++k){

        // We get the random value
        int node_pickup = rand() % (copy_list_pairs.size());

        // We randomly select a delivery node
        while(true){

            // We get the random value
            int node_delivery = rand() % (copy_list_pairs.size());

            // We check that it's not the same as the pickup one
            if (node_delivery == node_pickup) continue;

            // We add the pair in the list
            pickup_delivery_node_per_task.push_back(pair<int,int> (node_pickup,node_delivery));

            // We break the process for this task
            break;
        }
    }

    // We initialize the values for the release time values
    int nb_per_stop = 0;
    int step_stop = 0;

    if (frequency_for_instance == 0.2){
        nb_per_stop = 1;
        step_stop = 5;
    }
    else if (frequency_for_instance == 0.5){
        nb_per_stop = 1;
        step_stop = 2;
    }
    else {
        nb_per_stop = frequency_for_instance;
        step_stop = 1;
    }

    // We initialize the initial values
    int current_stop = 0, nb_done = 0;

    for (int k = 0; k < nb_task_for_instance; ++k){

        // We check if we have to update the current stop value
        if (nb_done == nb_per_stop){

            // We increment with the step
            current_stop += step_stop;

            // We reset the nb done value
            nb_done = 0;
        }

        // We create the tasks
        file_task  << current_stop << " " << pickup_delivery_node_per_task[k].first << " " <<
              pickup_delivery_node_per_task[k].second << " 0 0" << endl;

        // We increment the number of done
        ++nb_done;
    }
}

void Instance::create_instances_second_set(int nb_agent_for_map,int nb_task_for_instance,
                                           double frequency_for_instance){

    // We create the copy of the list of pairs
    vector<pair<int,int> > copy_list_pairs;
    for (pair<int,int> & pair_to_copy : this->list_pair_node_endpoint){
        copy_list_pairs.push_back(pair<int,int> (pair_to_copy.first,pair_to_copy.second));
    }

    // For each agent of the problem
    for (Agent * agent : this->list_agents) {

        // We add the agent's location to the send of possible locations
        copy_list_pairs.push_back(pair<int,int> (
                copy_list_pairs.size(),agent->get_path()[0]));
    }

    // We create the list of possible nodes for the agents
    vector<int> possible_nodes_agents;
    for (int i = 1; i < 20; ++i){
        possible_nodes_agents.push_back(35*i + 1);
        possible_nodes_agents.push_back(35*i + 2);
        possible_nodes_agents.push_back(35*i + 32);
        possible_nodes_agents.push_back(35*i + 33);
    }

    // We remove those values from the possible endpoints for the pickup and deliveries
    for (int value : possible_nodes_agents){

        // For each pair of the list
        for (int pair_index = 0; pair_index < copy_list_pairs.size(); ++pair_index){

            // We check if the node value corresponds
            if (copy_list_pairs[pair_index].second == value){

                // We remove the pair from the list
                copy_list_pairs.erase(copy_list_pairs.begin() + pair_index);

                // We stop the process for the current value
                break;
            }
        }
    }

    // We open the existing file
    fstream file;
    int frequency_to_write = 10* frequency_for_instance;
    file.open ("./Instances_Second_Set/map_"+ to_string(nb_agent_for_map) + "_" + to_string(nb_task_for_instance)
               + "_" + to_string(frequency_to_write) + ".map", fstream::out);

    // We write the size of the map
    file << this->nb_row << " " << this->nb_column << endl;

    // We write the number of endpoints
    file << (copy_list_pairs.size()) << endl;

    // We write the number of agents
    file << to_string(nb_agent_for_map) << endl;

    // We write the default value
    file << this->list_agents[0]->get_path().size() << endl;

    // We initialize the list of positions for the agents
    vector<int> positions_agents;

    // We randomly define the positions of the agents
    for (int k = 0; k < nb_agent_for_map; ++k){

        while(true){

            // We randomly choose a node
            int rdm_node = rand() % (possible_nodes_agents.size());

            // We get the associated grid node
            int current_value = possible_nodes_agents[rdm_node];

            // We check that the node is not already in the lise
            if (find(positions_agents.begin(), positions_agents.end(), current_value) !=
                positions_agents.end()) continue;

            // We add the node in the list
            positions_agents.push_back(current_value);

            // We break the process for the current agent
            break;
        }
    }

    // We initialize the current index of the node
    int current_index = -1;

    // We write the map
    for (int row = 0; row < this->nb_row; ++row){

        for (int column = 0; column < this->nb_column; ++column){

            // We increment the current index
            ++ current_index;

            // We check if the current node is a map node
            if (!this->list_map_nodes[current_index]){

                // We write the obstacle value
                file << "@";
            }
            else {

                // We check if the node is an endpoint
                if(!this->list_endpoints[current_index]){

                    // We write the non-endpoint value
                    file << ".";
                }
                else {

                    // We check if the node is in the list (robot start point)
                    if (find(positions_agents.begin(),positions_agents.end(),current_index) != positions_agents.end()){

                        // We write the robot value
                        file << "r";
                    }
                    else {

                        if (find(possible_nodes_agents.begin(),possible_nodes_agents.end(), current_index) !=
                             possible_nodes_agents.end()){

                            // We write the non-endpoint value
                            file << ".";
                        }
                        else {

                            // We write the endpoint value
                            file << "e";
                        }
                    }
                }
            }
        }

        // We go to the next line
        file << endl;
    }

    // We create/open the file
    fstream file_task;
    file_task.open ("./Instances_Second_Set/task_"+ to_string(nb_agent_for_map) + "_" + to_string(nb_task_for_instance)
                    + "_" + to_string(frequency_to_write) + ".task", fstream::out);

    // We write the number of task
    file_task << nb_task_for_instance << endl;

    // We initialize the list of pickup and delivery point per task
    vector<pair<int,int> > pickup_delivery_node_per_task;

    // We define the pickup and delivery point for each task of the problem
    for (int k = 0; k < nb_task_for_instance; ++k){

        // We get the random value
        int node_pickup = rand() % (copy_list_pairs.size());

        // We randomly select a delivery node
        while(true){

            // We get the random value
            int node_delivery = rand() % (copy_list_pairs.size());

            // We check that it's not the same as the pickup one
            if (node_delivery == node_pickup) continue;

            // We add the pair in the list
            pickup_delivery_node_per_task.push_back(pair<int,int> (node_pickup,node_delivery));

            // We break the process for this task
            break;
        }
    }

    // We initialize the values for the release time values
    int nb_per_stop = 0;
    int step_stop = 0;

    if (frequency_for_instance == 0.2){
        nb_per_stop = 1;
        step_stop = 5;
    }
    else if (frequency_for_instance == 0.5){
        nb_per_stop = 1;
        step_stop = 2;
    }
    else {
        nb_per_stop = frequency_for_instance;
        step_stop = 1;
    }

    // We initialize the initial values
    int current_stop = 0, nb_done = 0;

    for (int k = 0; k < nb_task_for_instance; ++k){

        // We check if we have to update the current stop value
        if (nb_done == nb_per_stop){

            // We increment with the step
            current_stop += step_stop;

            // We reset the nb done value
            nb_done = 0;
        }

        // We create the tasks
        file_task  << current_stop << " " << pickup_delivery_node_per_task[k].first << " " <<
                   pickup_delivery_node_per_task[k].second << " 0 0" << endl;

        // We increment the number of done
        ++nb_done;
    }
}

void Instance::create_instances_third_set(int nb_agent_for_map,int nb_task_for_instance,
                                           double frequency_for_instance){

    // We create the copy of the list of pairs
    vector<pair<int,int> > copy_list_pairs;
    for (pair<int,int> & pair_to_copy : this->list_pair_node_endpoint){
        copy_list_pairs.push_back(pair<int,int> (pair_to_copy.first,pair_to_copy.second));
    }

    // For each agent of the problem
    for (Agent * agent : this->list_agents) {

        // We add the agent's location to the send of possible locations
        copy_list_pairs.push_back(pair<int,int> (
                copy_list_pairs.size(),agent->get_path()[0]));
    }

    // We create the list of possible nodes for the agents
    vector<int> possible_nodes_agents;
    for (int i = 1; i < 20; ++i){
        possible_nodes_agents.push_back(35*i + 1);
        possible_nodes_agents.push_back(35*i + 2);
        possible_nodes_agents.push_back(35*i + 16);
        possible_nodes_agents.push_back(35*i + 18);
        possible_nodes_agents.push_back(35*i + 32);
        possible_nodes_agents.push_back(35*i + 33);
    }

    // We remove those values from the possible endpoints for the pickup and deliveries
    for (int value : possible_nodes_agents){

        // For each pair of the list
        for (int pair_index = 0; pair_index < copy_list_pairs.size(); ++pair_index){

            // We check if the node value corresponds
            if (copy_list_pairs[pair_index].second == value){

                // We remove the pair from the list
                copy_list_pairs.erase(copy_list_pairs.begin() + pair_index);

                // We stop the process for the current value
                break;
            }
        }
    }

    // We open the existing file
    fstream file;
    int frequency_to_write = 10*frequency_for_instance;
    file.open ("./Instances_Third_Set/map_"+ to_string(nb_agent_for_map) + "_" + to_string(nb_task_for_instance)
               + "_" + to_string(frequency_to_write) + ".map", fstream::out);

    // We write the size of the map
    file << this->nb_row << " " << this->nb_column << endl;

    // We write the number of endpoints
    file << (copy_list_pairs.size()) << endl;

    // We write the number of agents
    file << to_string(nb_agent_for_map) << endl;

    // We write the default value
    file << this->list_agents[0]->get_path().size() << endl;

    // We initialize the list of positions for the agents
    vector<int> positions_agents;

    // We randomly define the positions of the agents
    for (int k = 0; k < nb_agent_for_map; ++k){

        while(true){

            // We randomly choose a node
            int rdm_node = rand() % (possible_nodes_agents.size());

            // We get the associated grid node
            int current_value = possible_nodes_agents[rdm_node];

            // We check that the node is not already in the lise
            if (find(positions_agents.begin(), positions_agents.end(), current_value) !=
                positions_agents.end()) continue;

            // We add the node in the list
            positions_agents.push_back(current_value);

            // We break the process for the current agent
            break;
        }
    }

    // We initialize the current index of the node
    int current_index = -1;

    // We write the map
    for (int row = 0; row < this->nb_row; ++row){

        for (int column = 0; column < this->nb_column; ++column){

            // We increment the current index
            ++ current_index;

            // We check if the current node is a map node
            if (!this->list_map_nodes[current_index]){

                // We write the obstacle value
                file << "@";
            }
            else {

                // We check if the node is an endpoint
                if(!this->list_endpoints[current_index]){

                    // We write the non-endpoint value
                    file << ".";
                }
                else {

                    // We check if the node is in the list (robot start point)
                    if (find(positions_agents.begin(),positions_agents.end(),current_index) != positions_agents.end()){

                        // We write the robot value
                        file << "r";
                    }
                    else {

                        if (find(possible_nodes_agents.begin(),possible_nodes_agents.end(), current_index) !=
                            possible_nodes_agents.end()){

                            // We write the non-endpoint value
                            file << ".";
                        }
                        else {

                            // We write the endpoint value
                            file << "e";
                        }
                    }
                }
            }
        }

        // We go to the next line
        file << endl;
    }

    // We create/open the file
    fstream file_task;
    file_task.open ("./Instances_Third_Set/task_"+ to_string(nb_agent_for_map) + "_" + to_string(nb_task_for_instance)
                    + "_" + to_string(frequency_to_write) + ".task", fstream::out);

    // We write the number of task
    file_task << nb_task_for_instance << endl;

    // We initialize the list of pickup and delivery point per task
    vector<pair<int,int> > pickup_delivery_node_per_task;

    // We define the pickup and delivery point for each task of the problem
    for (int k = 0; k < nb_task_for_instance; ++k){

        // We get the random value
        int node_pickup = rand() % (copy_list_pairs.size());

        // We randomly select a delivery node
        while(true){

            // We get the random value
            int node_delivery = rand() % (copy_list_pairs.size());

            // We check that it's not the same as the pickup one
            if (node_delivery == node_pickup) continue;

            // We add the pair in the list
            pickup_delivery_node_per_task.push_back(pair<int,int> (node_pickup,node_delivery));

            // We break the process for this task
            break;
        }
    }

    // We initialize the values for the release time values
    int nb_per_stop = 0;
    int step_stop = 0;

    if (frequency_for_instance == 0.2){
        nb_per_stop = 1;
        step_stop = 5;
    }
    else if (frequency_for_instance == 0.5){
        nb_per_stop = 1;
        step_stop = 2;
    }
    else {
        nb_per_stop = frequency_for_instance;
        step_stop = 1;
    }

    // We initialize the initial values
    int current_stop = 0, nb_done = 0;

    for (int k = 0; k < nb_task_for_instance; ++k){

        // We check if we have to update the current stop value
        if (nb_done == nb_per_stop){

            // We increment with the step
            current_stop += step_stop;

            // We reset the nb done value
            nb_done = 0;
        }

        // We create the tasks
        file_task  << current_stop << " " << pickup_delivery_node_per_task[k].first << " " <<
                   pickup_delivery_node_per_task[k].second << " 0 0" << endl;

        // We increment the number of done
        ++nb_done;
    }
}
