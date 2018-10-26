//
// Created by Florian Grenouilleau on 2018-10-25.
//

#include <iostream>
#include "../include/Solution.h"

void Solution::update_list_open_tasks(int current_time_step){

    // For each task
    for (Task * task : instance->get_list_tasks()){

        // We check if the release time corresponds
        if (task->get_release_date() == current_time_step){

            // We add the task's id in the list
            this->list_open_tasks.push_back(task->get_id());
        }

        else if (task->get_release_date() > current_time_step){

            // We stop the process
            break;
        }
    }
}

void Solution::apply_assignment(int id_task, int id_agent, vector<Position> & list_new_positions){

    //cout << "Start of the apply assignment " << endl;

    // We increase the size of the positions matrix if necessary
    if (this->list_positions_per_time_step.size() < list_new_positions[list_new_positions.size()-1].get_time_step()+1){

        // For each uncovered time step
        for (int k = this->list_positions_per_time_step.size();
             k <= list_new_positions[list_new_positions.size()-1].get_time_step(); ++k){

            // We add a new vector of position
            this->list_positions_per_time_step.push_back(vector<Position> ());
        }
    }

    // We initialize the values
    int picked_date = -1, delivered_date = -1;

    // We get the current task
    Task * current_task = this->instance->get_list_tasks()[id_task];

    // For each new position
    for (Position & position : list_new_positions){ // We assume that the positions are order according to the time step

        // We check if the picked location has been reached
        if (position.get_id_node() == current_task->get_pickup_node() && picked_date == -1){

            // We update the picked date
            picked_date = position.get_time_step();
        }
        else if (position.get_id_node() == current_task->get_delivery_node() && picked_date != -1 &&
                delivered_date == -1){

            // We update the delivered date
            delivered_date = position.get_time_step();
        }

        // We add the position in the matrix
        this->apply_position(position);
    }

    // We remove the task from the open tasks' list
    this->list_open_tasks.erase(find(this->list_open_tasks.begin(),this->list_open_tasks.end(),id_task));

    // We update the task's dates
    if (picked_date == -1 || delivered_date == -1 || (picked_date > delivered_date) ){
        cout << "Problem, values in the assignment method not correct" << endl;
    }
    else {
        current_task->set_picked_date(picked_date);
        current_task->set_delivered_date(delivered_date);
    }

    //cout << "End of the apply assignment " << endl;
}

void Solution::apply_position(Position & position){

    // We initialize the boolean value
    bool position_existed = false;

    // We check if a position exists for the current time step and the current agent
    for (Position & existing_position : this->list_positions_per_time_step[position.get_time_step()]){

        // We check if the agent id corresponds
        if (existing_position.get_id_agent() == position.get_id_agent()){

            // We update the values of the position
            existing_position.set_id_node(position.get_id_node());
            existing_position.set_assigned_task(position.get_assigned_task());

            // We update the boolean value
            position_existed = true;
        }
    }

    // We check if the position existed
    if (!position_existed){

        // We create a new position
        this->list_positions_per_time_step[position.get_time_step()].push_back(Position (position.get_id_agent()
                , position.get_id_node(), position.get_time_step(), position.get_assigned_task()));
    }
}

void Solution::create_wait_positions(int current_time_step){

    // We check if the size of the position matrix is sufficient
    if (this->list_positions_per_time_step.size() < current_time_step + 2){

        // We increase the size of the position matrix
        this->list_positions_per_time_step.push_back(vector<Position> ());
    }

    // We initialize the list of current position per agent
    vector<int> current_id_node_per_agent (this->instance->get_nb_agent(),0);

    // We get the current position of each agent
    for (Position & position : this->list_positions_per_time_step[current_time_step]){

        current_id_node_per_agent[position.get_id_agent()] = position.get_id_node();
    }

    // For each agent
    for (int agent = 0; agent < this->instance->get_nb_agent(); ++agent){

        // We initalize the boolean value
        bool position_exists = false;

        // We check if a position exists for the following time step
        for (Position & position_next : this->list_positions_per_time_step[current_time_step+1]){

            if (position_next.get_id_agent() == agent){
                position_exists = true;
            }
        }

        // We check if a position exists
        if (!position_exists){

            // We create a new position for the agent
            this->list_positions_per_time_step[current_time_step+1].push_back(Position(agent
                    ,current_id_node_per_agent[agent], current_time_step+1,-1));
        }
    }
}

Position Solution::return_position(int id_agent, int time_step){

    // For each position of the time step
    for (Position & position : this->list_positions_per_time_step[time_step]){

        // We check if the agent id corresponds
        if (position.get_id_agent() == id_agent){

            // We return the position
            return position;
        }
    }

    return Position(-1,-1,-1,-1);
}

void Solution::compute_positions_matrix(){

    // We clear the positions matrix
    this->positions_matrix.clear();

    // For each list of positions
    for (vector<Position> & list_positions : this->list_positions_per_time_step){

        // We add a new vector
        this->positions_matrix.push_back(vector<int> (this->instance->get_nb_agent(),-1));

        // We each position of the list
        for (Position & position : list_positions){

            // We update the agent's value
            this->positions_matrix[this->positions_matrix.size()-1][position.get_id_agent()] = position.get_id_node();
        }

        // We update the matrix with waits if no position exists for the agent
        for (int agent = 0; agent < this->instance->get_nb_agent(); ++agent){

            // We check if the value is the default one
            if (this->positions_matrix[this->positions_matrix.size()-1][agent] == -1){

                // We use the previous position of the agent
                this->positions_matrix[this->positions_matrix.size()-1][agent] =
                        this->positions_matrix[this->positions_matrix.size()-2][agent];
            }
        }
    }
}

bool Solution::check_solution_feasible(){

    // We compute the position matrix for the solution
    this->compute_positions_matrix();

    // We check that each agent a position for every time step of the horizon
    for (int agent = 0; agent < this->instance->get_nb_agent(); ++agent) {

        // For each time step of the final horizon
        for (int time_step = 0; time_step < this->list_positions_per_time_step.size(); ++time_step){

            // We check if a position exists
            if (this->positions_matrix[time_step][agent] == -1){
                cout << "Problem, no position for the agent " << agent << " at the time step " << time_step << endl;

                // We return false
                return false;
            }
        }
    }

    // We check that for each agent, the move are toward successors
    for (int agent = 0; agent < this->instance->get_nb_agent(); ++agent) {

        // For each time step of the final horizon
        for (int time_step = 0; time_step < this->list_positions_per_time_step.size()-1; ++time_step){

            // We get the current position
            int current_node = this->positions_matrix[time_step][agent];

            // We get the next position
            int next_node = this->positions_matrix[time_step+1][agent];

            // We check that the move is feasible
            if (find(this->instance->get_list_nodes()[current_node]->get_list_id_nodes_successors().begin(),
                     this->instance->get_list_nodes()[current_node]->get_list_id_nodes_successors().end(),
                     next_node) ==
                    this->instance->get_list_nodes()[current_node]->get_list_id_nodes_successors().end()){

                cout << "Problem, the move is not possible " << endl;

                // We return false
                return false;
            }
        }
    }

    // We check the vertex constraints
    for (int time_step = 0; time_step < this->list_positions_per_time_step.size(); ++time_step){

        // For each agent
        for (int agent = 0; agent < this->instance->get_nb_agent()-1; ++agent) {

            // For each other agent
            for (int agent2 = agent + 1; agent2 < this->instance->get_nb_agent(); ++agent2) {

                // We check if they are at the same node
                if (this->positions_matrix[time_step][agent] == this->positions_matrix[time_step][agent2]){

                    cout << "Problem, same vertex used " << endl;

                    // We return false
                    return false;
                }
            }
        }
    }

    // We check the edges constraint
    for (int agent = 0; agent < this->instance->get_nb_agent()-1; ++agent) {

        // For each other agent
        for (int agent2 = agent + 1; agent2 < this->instance->get_nb_agent(); ++agent2) {

            // For each time step of the final horizon
            for (int time_step = 0; time_step < this->list_positions_per_time_step.size()-1; ++time_step){

                // We get the current nodes
                int current_1 = this->positions_matrix[time_step][agent];
                int current_2 = this->positions_matrix[time_step][agent2];

                // We get the next nodes
                int next_1 = this->positions_matrix[time_step+1][agent];
                int next_2 = this->positions_matrix[time_step+1][agent2];

                // We check the constraint
                if (current_1 == next_2 && current_2 == next_1){

                    cout << "Problem, same edge used " << endl;

                    // We return false
                    return false;
                }
            }
        }
    }

    // We check that each task is covered at the assigned date
    for (Task * task : this->instance->get_list_tasks()){

        // We check that the pickup node is covered

        // We initialize the boolean value
        bool node_found = false;

        for (int value : this->positions_matrix[task->get_picked_date()]){

            // We check if the nodes correspond
            if (value == task->get_pickup_node()){

                // We update the boolean value
                node_found = true;

                // We break the loop
                break;
            }
        }

        if (!node_found){

            cout << "Problem, pickup node not covered" << endl;

            // We return false
            return false;
        }

        // We check that the delivery node is covered

        // We re-initialize the boolean value
        node_found = false;

        for (int value : this->positions_matrix[task->get_delivered_date()]){

            // We check if the nodes correspond
            if (value == task->get_delivery_node()){

                // We update the boolean value
                node_found = true;

                // We break the loop
                break;
            }
        }

        if (!node_found){

            cout << "Problem, delivery node not covered" << endl;

            // We return false
            return false;
        }
    }

    // We check that each task is completed by the same agent from the pickup to the delivery node

    for (Task * task : this->instance->get_list_tasks()){

        // We get the assigned agent
        int assigned_agent = distance(this->positions_matrix[task->get_picked_date()].begin(),
        find(this->positions_matrix[task->get_picked_date()].begin(),
             this->positions_matrix[task->get_picked_date()].end(),task->get_pickup_node()));

        // We check that the position of the agent is assigned to the task
        for (int time_step = task->get_picked_date(); time_step < task->get_delivered_date(); ++ time_step){

            for (Position & position : this->list_positions_per_time_step[time_step]){

                // We check if the agent corresponds
                if (position.get_id_agent() == assigned_agent){

                    // We check if the assigned task corresponds
                    if (task->get_id() != position.get_assigned_task()){

                        cout << "Problem, agent not assigned to the task" << endl;

                        // We return false
                        return false;
                    }
                }
            }
        }

    }

    // We return true
    return true;
}

void Solution::write(){

    cout << endl;

    for (Task * task : this->instance->get_list_tasks()){
        task->write();
        cout <<  endl;
    }

    for (int agent = 0; agent < this->instance->get_nb_agent(); ++agent) {

        for (int time_step = 0; time_step < this->list_positions_per_time_step.size(); ++time_step){

            cout << "Agent " << agent << ", Time step " << time_step << ", Node : " <<
                 this->positions_matrix[time_step][agent] << endl;
        }
    }
}