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

    // We increase the size of the positions matrix if necessary
    if (this->list_positions_per_time_step.size() < list_new_positions[list_new_positions.size()-1].get_time_step()){

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