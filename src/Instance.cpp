//
// Created by Florian Grenouilleau on 2018-10-25.
//
#include "../include/Instance.h"
#include <queue>
#include <iostream>
#include <fstream>

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

    // We update the assigned agent
    this->list_tasks[id_task]->set_id_assigned_agent(id_agent);

    // We update the dates
    this->list_tasks[id_task]->set_picked_date(arrive_start);
    this->list_tasks[id_task]->set_delivered_date(arrive_goal);

    // We remove the task for the open tasks' list
    this->get_list_open_tasks().erase(find(this->list_open_tasks.begin(),
                                           this->list_open_tasks.end(),
                                           this->list_tasks[id_task]));

    // We check that the agent's path correspond
    if (this->list_agents[id_agent]->get_path()[arrive_start] != this->list_tasks[id_task]->get_pickup_node() ||
            this->list_agents[id_agent]->get_path()[arrive_goal] != this->list_tasks[id_task]->get_delivery_node()){

        cout << "Problem, the agent's path does not correspond" << endl;
    }

    // We increment the number of scheduled task
    ++ this->nb_task_scheduled;
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

    // We initialize the sum
    double sum = 0;

    // For each task of the problem
    for (int value : this->nb_agent_available_per_time_step){

        sum += value;
    }

    // We return the average of the computed sum
    return sum / (double) this->nb_agent_available_per_time_step.size();
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
    file << compute_average_service_time() << ";";
    file << argv[3] << ";";
    file << this->wait_value << ";";
    file << compute_average_impact_traffic() << ";";
    file << compute_average_nb_agent_avail() << ";";
    file << endl;

    // We close the file
    file.close();
}