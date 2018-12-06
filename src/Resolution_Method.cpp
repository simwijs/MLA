//
// Created by Florian Grenouilleau on 2018-10-31.
//

#include "../include/Resolution_Method.h"
#include <queue>
#include <limits>
#include <utility>
#include <string>
#include <ilcplex/ilocplex.h>

void Resolution_Method::solve_instance(Instance * instance, int solver_id){

    // We update the solve type
    this->solve_type = solver_id;

    if (this->solve_type == 1){

        // We solve the classic TOTP
        solve_TOTP(instance);
    }
    else if (this->solve_type == 2){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the TOTP with Improved A*
        solve_TOTP(instance);
    }
    else if (this->solve_type == 3){

        // We solve the TOTP with Improved A*
        solve_TOTP(instance);
    }
    else if (this->solve_type == 4){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the Greedy algorithm
        solve_Greedy_Heuristic(instance);
    }
    else if (this->solve_type == 5){

        // We solve the Greedy algorithm
        solve_Greedy_Heuristic(instance);
    }
    else if (this->solve_type == 6){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the problem with the set partitioning heuristic
        solve_Greedy_Heuristic_With_Exchange(instance);
    }
    else if (this->solve_type == 7){

        // We solve the problem with the set partitioning heuristic
        solve_Greedy_Heuristic_With_Exchange(instance);
    }
    else if (this->solve_type == 8){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_TOTP(instance);
    }
    else if (this->solve_type == 9){

        // We update the boolean value
        this->allow_move_after_endpoint = false;


        // We solve the classic TOTP
        solve_Decentralized(instance,0);
    }
    else if (this->solve_type == 10){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_Decentralized(instance,1);
    }
    else if (this->solve_type == 11){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_Decentralized(instance,2);
    }
    else if (this->solve_type == 12){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_Decentralized(instance,3);
    }
    else if (this->solve_type == 13){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_Decentralized(instance,4);
    }
    else if (this->solve_type == 14){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_Decentralized(instance,5);
    }
    else if (this->solve_type == 15){

        // We update the boolean value
        this->allow_move_after_endpoint = true;


        // We solve the classic TOTP
        solve_Decentralized(instance,0);
    }
    else if (this->solve_type == 16){

        // We update the boolean value
        this->allow_move_after_endpoint = true;

        // We solve the classic TOTP
        solve_Decentralized(instance,1);
    }
    else if (this->solve_type == 17){

        // We update the boolean value
        this->allow_move_after_endpoint = true;

        // We solve the classic TOTP
        solve_Decentralized(instance,2);
    }
    else if (this->solve_type == 18){

        // We update the boolean value
        this->allow_move_after_endpoint = true;

        // We solve the classic TOTP
        solve_Decentralized(instance,3);
    }
    else if (this->solve_type == 19){

        // We update the boolean value
        this->allow_move_after_endpoint = true;

        // We solve the classic TOTP
        solve_Decentralized(instance,4);
    }
    else if (this->solve_type == 20){

        // We update the boolean value
        this->allow_move_after_endpoint = true;

        // We solve the classic TOTP
        solve_Decentralized(instance,5);
    }
    else if (this->solve_type == 21){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_Decentralized_ST_Focused(instance,0);
    }
    else if (this->solve_type == 22){

        // We update the boolean value
        this->allow_move_after_endpoint = false;

        // We solve the classic TOTP
        solve_Decentralized_ST_Focused(instance,5);
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

        if (solve_type == 1 || solve_type == 8){

            // We check if a TOTP solution has been found
            if (!this->apply_TOTP(instance,current_agent))
            {
                cout << "Problem, no assignment found for the current agent " << endl;
                getchar();
            }

        }
        else if (solve_type == 2 || solve_type == 3){

            // We check if a TOTP solution has been found
            if (!this->apply_TOTP_2(instance,current_agent))
            {
                cout << "Problem, no assignment found for the current agent " << endl;
                getchar();
            }

        }
        else {

            cout << "Problem, the current solve type does not exist" << endl;
        }

    }

    //cout << "End of the TOTP Algorithm" << endl;
}

void Resolution_Method::solve_Decentralized(Instance * instance, int n_value){

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
           instance->get_current_time_step() <= instance->get_max_horizon()) {

        // We update the list of open goals for the current time step
        for (int id_task : instance->get_id_released_tasks_per_time_step()[instance->get_current_time_step()]) {

                instance->get_list_open_tasks().push_back(instance->get_task(id_task));
        }

        while (true){

            // We get the first agent available
            Agent * current_agent = NULL;

            // For each agent
            for (Agent * agent : instance->get_list_agents()){

                // We check if the finish time corresponds
                if (agent->get_finish_time() == instance->get_current_time_step()){

                    // We get the agent
                    current_agent = agent;

                    // We stop the process
                    break;
                }
            }

            // We check if an available agent exists for the current time step
            if (current_agent != NULL){

                // We update the current agent location
                current_agent->set_current_location(current_agent->get_path()[instance->get_current_time_step()]);

                // We initialize the list of available agents
                vector<Agent*> list_available_agents;

                // We get the list of available agents in the following time steps
                for (Agent * agent : instance->get_list_agents()){

                    // We check if the finish time is in the current frame
                    if (agent->get_finish_time() <= instance->get_current_time_step() + n_value){

                        // We add the agent in the list of available ones
                        list_available_agents.push_back(agent);
                    }
                }

                // We initialize the list of pair task-hvalue
                vector<pair<int,int> > list_h_value_per_task;

                // For each open task
                for (Task * task : instance->get_list_open_tasks()){

                    // We compute the h value from the current agent
                    list_h_value_per_task.push_back(pair<int,int> (
                            instance->get_h_values_per_node()[current_agent->get_current_location()][
                                    task->get_pickup_node()], task->get_id()));
                }

                // We sort the task per h values
                sort(list_h_value_per_task.begin(),list_h_value_per_task.end());

                // We initialize the boolean value
                bool assignment_found = false;

                // While some tasks are still to check
                while (!list_h_value_per_task.empty()){

                    // We get the first task to check
                    Task * current_task = instance->get_task(list_h_value_per_task[0].second);

                    // We get the associated h value
                    int current_h_value = list_h_value_per_task[0].first;

                    // We remove the task from the list
                    list_h_value_per_task.erase(list_h_value_per_task.begin());

                    // We initialize the best value
                    int best_agent = -1;

                    // For every available agent
                    for (Agent * agent_to_check : list_available_agents){

                        if (current_h_value > instance->get_h_values_per_node()[agent_to_check->get_path()[
                                agent_to_check->get_finish_time()]][current_task->get_pickup_node()] +
                                                      (agent_to_check->get_finish_time() -
                                                              instance->get_current_time_step())){

                            // We update the current best agent value
                            best_agent = agent_to_check->get_id();

                            // We update the current h value
                            current_h_value = instance->get_h_values_per_node()[agent_to_check->get_path()[
                                    agent_to_check->get_finish_time()]][current_task->get_pickup_node()] +
                                              (agent_to_check->get_finish_time() -
                                               instance->get_current_time_step());
                        }
                    }

                    // We check if another agent is better that the current one
                    if (best_agent != -1){

                        // We remove this agent from the list of available ones
                        list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                        instance->get_agent(best_agent)));

                        // We continue with the next open goal
                        continue;
                    }
                    else {

                        // We try the assignment and apply it if possible
                        if(check_if_assignment_feasible(instance, current_agent, current_task)){

                            // We update the boolean value
                            assignment_found = true;

                            // We stop the process
                            break;
                        }
                    }
                }

                // We check if the agent has been assigned
                if (!assignment_found){

                    // We initialize the boolean value
                    bool move = false;

                    // We check if the agent has to move from its current position
                    for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
                         it != instance->get_list_open_tasks().end(); it++) {

                        // We check if the delivery location correponds with the agent's current location
                        if ((*it)->get_delivery_node() == current_agent->get_current_location()) {
                            move = true;
                            break;
                        }
                    }

                    // We check if a move is necessary
                    if (move) {

                        // We move the agent
                        compute_move_to_endpoint(instance,current_agent);

                        if (allow_modification_endpoint){

                            // We update the finish time of the agent
                            current_agent->set_finish_time(instance->get_current_time_step() + 1);
                        }
                    }
                    else {

                        // We update the finish time of the agent
                        current_agent->set_finish_time(current_agent->get_finish_time() + 1);
                    }
                }
            }
            else {

                // We stop the search for the current time step
                break;
            }
        }

        // We increment the current time step
        instance->set_current_time_step(instance->get_current_time_step() + 1);
    }
}

void Resolution_Method::solve_Decentralized_ST_Focused(Instance * instance, int n_value){

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
           instance->get_current_time_step() <= instance->get_max_horizon()) {

        // We update the list of open goals for the current time step
        for (int id_task : instance->get_id_released_tasks_per_time_step()[instance->get_current_time_step()]) {
            instance->get_list_open_tasks().push_back(instance->get_task(id_task));
        }

        while (true){

            // We get the first agent available
            Agent * current_agent = NULL;

            // For each agent
            for (Agent * agent : instance->get_list_agents()){

                // We check if the finish time corresponds
                if (agent->get_finish_time() == instance->get_current_time_step()){

                    // We get the agent
                    current_agent = agent;

                    // We stop the process
                    break;
                }
            }

            // We check if an available agent exists for the current time step
            if (current_agent != NULL){

                // We update the current agent location
                current_agent->set_current_location(current_agent->get_path()[instance->get_current_time_step()]);

                // We initialize the list of available agents
                vector<Agent*> list_available_agents;

                // We get the list of available agents in the following time steps
                for (Agent * agent : instance->get_list_agents()){

                    // We check if the finish time is in the current frame
                    if (agent->get_finish_time() <= instance->get_current_time_step() + n_value){

                        // We add the agent in the list of available ones
                        list_available_agents.push_back(agent);
                    }
                }

                // We initialize the list of pair task-releaseTime
                vector<pair<int,int> > list_release_time_per_task;

                // For each open task
                for (Task * task : instance->get_list_open_tasks()){

                    // We compute the h value from the current agent
                    list_release_time_per_task.push_back(pair<int,int> (task->get_release_date(), task->get_id()));
                }

                // We sort the task per release time
                sort(list_release_time_per_task.begin(),list_release_time_per_task.end());

                // We initialize the boolean value
                bool assignment_found = false;

                // While some tasks are still to check
                while (!list_release_time_per_task.empty()){

                    // We get the first task to check
                    Task * current_task = instance->get_task(list_release_time_per_task[0].second);

                    // We get the associated h value
                    int current_h_value = instance->get_h_values_per_node()[current_agent->get_current_location()][
                            current_task->get_pickup_node()];

                    // We remove the task from the list
                    list_release_time_per_task.erase(list_release_time_per_task.begin());

                    // We initialize the best value
                    int best_agent = -1;

                    // For every available agent
                    for (Agent * agent_to_check : list_available_agents){

                        if (current_h_value > instance->get_h_values_per_node()[agent_to_check->get_path()[
                                agent_to_check->get_finish_time()]][current_task->get_pickup_node()] +
                                              (agent_to_check->get_finish_time() -
                                               instance->get_current_time_step())){

                            // We update the current best agent value
                            best_agent = agent_to_check->get_id();

                            // We update the current h value
                            current_h_value = instance->get_h_values_per_node()[agent_to_check->get_path()[
                                    agent_to_check->get_finish_time()]][current_task->get_pickup_node()] +
                                              (agent_to_check->get_finish_time() -
                                               instance->get_current_time_step());
                        }
                    }

                    // We check if another agent is better that the current one
                    if (best_agent != -1){

                        // We remove this agent from the list of available ones
                        list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                         instance->get_agent(best_agent)));

                        // We continue with the next open goal
                        continue;
                    }
                    else {

                        // We try the assignment and apply it if possible
                        if(check_if_assignment_feasible(instance, current_agent, current_task)){

                            // We update the boolean value
                            assignment_found = true;

                            // We stop the process
                            break;
                        }
                    }
                }

                // We check if the agent has been assigned
                if (!assignment_found){

                    // We initialize the boolean value
                    bool move = false;

                    // We check if the agent has to move from its current position
                    for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
                         it != instance->get_list_open_tasks().end(); it++) {

                        // We check if the delivery location correponds with the agent's current location
                        if ((*it)->get_delivery_node() == current_agent->get_current_location()) {
                            move = true;
                            break;
                        }
                    }

                    // We check if a move is necessary
                    if (move) {

                        // We move the agent
                        compute_move_to_endpoint(instance,current_agent);

                        if (allow_modification_endpoint){

                            // We update the finish time of the agent
                            current_agent->set_finish_time(instance->get_current_time_step() + 1);
                        }
                    }
                    else {

                        // We update the finish time of the agent
                        current_agent->set_finish_time(current_agent->get_finish_time() + 1);
                    }
                }
            }
            else {

                // We stop the search for the current time step
                break;
            }
        }

        // We increment the current time step
        instance->set_current_time_step(instance->get_current_time_step() + 1);
    }
}

void Resolution_Method::solve_Greedy_Heuristic(Instance * instance){

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
           instance->get_current_time_step() <= instance->get_max_horizon()) {

        // We update the list of open goals for the current time step
        for (int id_task : instance->get_id_released_tasks_per_time_step()[instance->get_current_time_step()]) {
            if (find(instance->get_list_open_tasks().begin(),
                     instance->get_list_open_tasks().end(),
                     instance->get_list_tasks()[id_task]) == instance->get_list_open_tasks().end() &&
                instance->get_list_tasks()[id_task]->get_id_assigned_agent() == -1){

                instance->get_list_open_tasks().push_back(instance->get_task(id_task));
            }
        }

        // We initialize the list of available agents
        vector<Agent *> list_available_agents;

        // We get the list of available agents
        for (Agent * agent : instance->get_list_agents()){
            if (agent->get_finish_time() == instance->get_current_time_step()){

                // We add the agent in the list
                list_available_agents.push_back(agent);

                // We update the agent current location
                agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);
            }
        }

        // We update the instance's value
        instance->get_nb_agent_available_per_time_step().push_back(list_available_agents.size());

        // We initialize the lists
        vector<pair<int,int> > list_pair_possible_assignment, list_h_value_per_pair;

        // We initialize the current index
        int current_index = -1;

        // For each task in the list of open ones
        for (Task * task : instance->get_list_open_tasks()){

            // For each agent in the list of available agents
            for (Agent * agent : list_available_agents){

                // We increment the current index
                ++ current_index;

                // We create the corresponding pair
                list_pair_possible_assignment.push_back(pair<int,int> (agent->get_id(),task->get_id()));

                // We add the h value in the list
                list_h_value_per_pair.push_back( pair<int,int> (
                        instance->get_h_values_per_node()[agent->get_current_location()][task->get_pickup_node()],
                        current_index));
            }
        }

        // We sort the list by h value
        sort(list_h_value_per_pair.begin(),list_h_value_per_pair.end());

        // While the lists are not empty
        while (!list_h_value_per_pair.empty() && !list_available_agents.empty() &&
                !instance->get_list_open_tasks().empty()){

            // We get the index of the first pair in the list
            int index_to_check = list_h_value_per_pair[0].second;

            // We remove this value from the list
            list_h_value_per_pair.erase(list_h_value_per_pair.begin());

            // We check that the agent is still available
            if (find(list_available_agents.begin(),list_available_agents.end(),
                     instance->get_agent(list_pair_possible_assignment[index_to_check].first))
                == list_available_agents.end()) continue;

            // We check that the task is still available
            if (find(instance->get_list_open_tasks().begin(), instance->get_list_open_tasks().end(),
                     instance->get_task(list_pair_possible_assignment[index_to_check].second))
                == instance->get_list_open_tasks().end()) continue;

            // We try the assignment and apply it if possible
            if(check_if_assignment_feasible(instance,
                                         instance->get_agent(list_pair_possible_assignment[index_to_check].first),
                                         instance->get_task(list_pair_possible_assignment[index_to_check].second))){

                // We remove the agent from the available ones
                list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                 instance->get_agent(
                                                         list_pair_possible_assignment[index_to_check].first)));
            }
        }

        // For each remaining agent
        for (Agent * agent_remaining : list_available_agents){

            // We initialize the boolean value
            bool move = false;

            // We check if the agent has to move from its current position
            for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
                 it != instance->get_list_open_tasks().end(); it++) {

                // We check if the delivery location correponds with the agent's current location
                if ((*it)->get_delivery_node() == agent_remaining->get_current_location()) {
                    move = true;
                    break;
                }
            }

            // We check if a move is necessary
            if (move) {

                // We move the agent
                compute_move_to_endpoint(instance,agent_remaining);

                if (allow_modification_endpoint){

                    // We update the finish time of the agent
                    agent_remaining->set_finish_time(instance->get_current_time_step() + 1);
                }
            }
            else {

                // We update the finish time of the agent
                agent_remaining->set_finish_time(agent_remaining->get_finish_time() + 1);
            }
        }

        // We increment the current time step
        instance->set_current_time_step(instance->get_current_time_step() + 1);
    }
}

void Resolution_Method::solve_Greedy_Heuristic_With_Exchange(Instance * instance){

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
           instance->get_current_time_step() <= instance->get_max_horizon()) {

        // We update the list of open goals for the current time step
        for (int id_task : instance->get_id_released_tasks_per_time_step()[instance->get_current_time_step()]) {
            if (find(instance->get_list_open_tasks().begin(),
                     instance->get_list_open_tasks().end(),
                     instance->get_list_tasks()[id_task]) == instance->get_list_open_tasks().end() &&
                instance->get_list_tasks()[id_task]->get_id_assigned_agent() == -1){

                instance->get_list_open_tasks().push_back(instance->get_task(id_task));
            }
        }

        // We initialize the list of available agents
        vector<Agent *> list_available_agents;

        // We get the list of available agents
        for (Agent * agent : instance->get_list_agents()){
            if (agent->get_finish_time() == instance->get_current_time_step()){

                // We add the agent in the list
                list_available_agents.push_back(agent);

                // We update the agent current location
                agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);
            }
        }

        // We update the instance's value
        instance->get_nb_agent_available_per_time_step().push_back(list_available_agents.size());

        // We initialize the list of possible goals
        vector<Task *> list_possible_goals;

        // We copy the list of open goals
        list_possible_goals.insert(list_possible_goals.begin(),instance->get_list_open_tasks().begin(),
                                  instance->get_list_open_tasks().end());

        // For each passed time step
        for (int ts = 0; ts < instance->get_current_time_step(); ++ts){

            // For each task of the time step
            for (int id_task : instance->get_id_released_tasks_per_time_step()[ts]){

                // We check if the task is assigned
                if (instance->get_task(id_task)->get_id_assigned_agent()!=-1){

                    // We check if the task has been picked up
                    if (instance->get_task(id_task)->get_picked_date() > instance->get_current_time_step()){

                        // We add the task in the list
                        list_possible_goals.push_back(instance->get_task(id_task));
                    }
                }
            }
        }

        // We initialize the boolean value
        bool assignment_found = true;

        // While the lists are not empty
        while (assignment_found && !list_available_agents.empty() && !list_possible_goals.empty()){

            // We update the boolean value
            assignment_found = false;

            // We initialize the lists
            vector<pair<int,int> > list_pair_possible_assignment, list_h_value_per_pair;

            // We initialize the current index
            int current_index = -1;

            // or each task in the list of open ones
            for (Task * task : list_possible_goals){

                // For each agent in the list of available agents
                for (Agent * agent : list_available_agents){

                    // We increment the current index
                    ++ current_index;

                    // We create the corresponding pair
                    list_pair_possible_assignment.push_back(pair<int,int> (agent->get_id(),task->get_id()));

                    // We add the h value in the list
                    list_h_value_per_pair.push_back( pair<int,int> (
                            instance->get_h_values_per_node()[agent->get_current_location()][task->get_pickup_node()],
                            current_index));
                }
            }

            // We sort the list by h value
            sort(list_h_value_per_pair.begin(),list_h_value_per_pair.end());

            while (true && !list_h_value_per_pair.empty()){

                // We get the index of the first pair in the list
                int index_to_check = list_h_value_per_pair[0].second;

                // We remove this value from the list
                list_h_value_per_pair.erase(list_h_value_per_pair.begin());

                // We get the current agent
                Agent * current_agent = instance->get_agent(list_pair_possible_assignment[index_to_check].first);

                // We get the current task
                Task * current_task = instance->get_task(list_pair_possible_assignment[index_to_check].second);

                // We check that the agent is still available
                if (find(list_available_agents.begin(),list_available_agents.end(), current_agent)
                    == list_available_agents.end()) continue;

                // We check that the task is still available
                if (find(list_possible_goals.begin(), list_possible_goals.end(), current_task)
                    == list_possible_goals.end()) continue;

                // We check if it is a classic assignment
                if (find(instance->get_list_open_tasks().begin(), instance->get_list_open_tasks().end(),
                         current_task) != instance->get_list_open_tasks().end()){

                    // We try the assignment and apply it if possible
                    if(check_if_assignment_feasible(instance, current_agent, current_task)){

                        // We remove the agent from the available ones
                        list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                         current_agent));

                        // We remove the task from the list
                        list_possible_goals.erase(find(list_possible_goals.begin(),list_possible_goals.end(),
                                                         current_task));
                    }

                }
                else {

                    // We first check the new pickup date with the h value
                    if (instance->get_current_time_step() +
                            instance->get_h_values_per_node()[
                                    current_agent->get_current_location()][current_task->get_pickup_node()] <
                            current_task->get_picked_date()){

                        // We get the other agent
                        Agent * other_agent = instance->get_agent(current_task->get_id_assigned_agent());

                        // We copy the existing values
                        vector<int> other_agent_path (other_agent->get_path().begin(),
                                                      other_agent->get_path().end());

                        int actual_picked_date = current_task->get_picked_date();
                        int actual_delivered_date = current_task->get_delivered_date();
                        int other_agent_finish_time = other_agent->get_finish_time();

                        // We update the finish time for the other agent
                        other_agent->set_finish_time(instance->get_current_time_step());
                        other_agent->set_current_location(other_agent->get_path()[instance->get_current_time_step()]);

                        // We reset the task values
                        current_task->set_picked_date(-1);
                        current_task->set_delivered_date(-1);
                        current_task->set_id_assigned_agent(-1);

                        // We remove 1 assignment to the instance
                        instance->set_nb_task_scheduled(instance->get_nb_task_scheduled()-1);

                        // We add the task in open task
                        instance->get_list_open_tasks().push_back(current_task);

                        // We reroute the other agent to an endpoint
                        compute_move_to_endpoint(instance,other_agent);

                        if (allow_modification_endpoint){
                            // We update the finish time of the agent
                            other_agent->set_finish_time(instance->get_current_time_step());
                        }

                        // We check if the assignment is possible
                        if (check_if_assignment_feasible(instance,current_agent,current_task)){

                            // We remove the agent from the available ones
                            list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                             current_agent));

                            // We remove the task from the list
                            list_possible_goals.erase(find(list_possible_goals.begin(),list_possible_goals.end(),
                                                           current_task));

                            // We check if the finish time of the other agent is now
                            if (other_agent->get_finish_time() == instance->get_current_time_step()){

                                // We add the other agent in the list of possible ones
                                list_available_agents.push_back(other_agent);

                                // We update its current location
                                other_agent->set_current_location(other_agent->get_path()[
                                                                          instance->get_current_time_step()]);
                            }

                            // TODO We update the boolean value - Voir si il faut le mettre ici ou dans le if du dessus
                            assignment_found = true;

                            // We stop the process (Necessity to recompute the possible h values)
                            break;
                        }
                        else {

                            // We undo the rerouting of the other agent
                            other_agent->get_path().clear();
                            other_agent->get_path().insert(other_agent->get_path().begin(),
                                                           other_agent_path.begin(),other_agent_path.end());
                            other_agent->set_finish_time(other_agent_finish_time);
                            other_agent->set_current_location(
                                    other_agent->get_path()[instance->get_current_time_step()]);

                            // We undo the reset of the task
                            current_task->set_picked_date(actual_picked_date);
                            current_task->set_delivered_date(actual_delivered_date);
                            current_task->set_id_assigned_agent(other_agent->get_id());

                            // We add the task in open task
                            instance->get_list_open_tasks().erase(find(instance->get_list_open_tasks().begin(),
                                                                       instance->get_list_open_tasks().end(),
                                                                       current_task));

                            // We add 1 assignment to the instance
                            instance->set_nb_task_scheduled(instance->get_nb_task_scheduled()+1);

                        }
                    }
                }

                // We check that the list of pairs is not empty
                if (list_h_value_per_pair.empty()){
                    break;
                }
            }
        }

        // For each remaining agent
        for (Agent * agent_remaining : list_available_agents){

            // We initialize the boolean value
            bool move = false;

            // We check if the agent has to move from its current position
            for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
                 it != instance->get_list_open_tasks().end(); it++) {

                // We check if the delivery location correponds with the agent's current location
                if ((*it)->get_delivery_node() == agent_remaining->get_current_location()) {
                    move = true;
                    break;
                }
            }

            // We check if a move is necessary
            if (move) {

                // We move the agent
                compute_move_to_endpoint(instance,agent_remaining);

                if (allow_modification_endpoint){

                    // We update the finish time of the agent
                    agent_remaining->set_finish_time(instance->get_current_time_step() + 1);
                }
            }
            else {

                // We update the finish time of the agent
                agent_remaining->set_finish_time(agent_remaining->get_finish_time() + 1);
            }
        }

        // We increment the current time step
        instance->set_current_time_step(instance->get_current_time_step() + 1);
    }
}

void Resolution_Method::solve_Set_Partitioning_Heuristic(Instance * instance){

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
           instance->get_current_time_step() <= instance->get_max_horizon()) {

        // We update the list of open goals for the current time step
        for (int id_task : instance->get_id_released_tasks_per_time_step()[instance->get_current_time_step()]) {
            if (find(instance->get_list_open_tasks().begin(),
                     instance->get_list_open_tasks().end(),
                     instance->get_list_tasks()[id_task]) == instance->get_list_open_tasks().end() &&
                instance->get_list_tasks()[id_task]->get_id_assigned_agent() == -1){

                instance->get_list_open_tasks().push_back(instance->get_task(id_task));
            }
        }

        // We initialize the list of available agents
        vector<Agent *> list_available_agents;

        // We get the list of available agents
        for (Agent * agent : instance->get_list_agents()){
            if (agent->get_finish_time() == instance->get_current_time_step()){

                // We add the agent in the list
                list_available_agents.push_back(agent);

                // We update the agent current location
                agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);
            }
        }

        // We update the instance's value
        instance->get_nb_agent_available_per_time_step().push_back(list_available_agents.size());

        // We initialize the list of chosen assignments
        vector<pair<int,int> > list_chosen_assignments;

        // We solve the assignment using the set partitioning
        compute_set_partitioning_assignment(instance,instance->get_list_open_tasks(),list_available_agents,
                                            list_chosen_assignments);

        // We try the found assignments
        for (pair<int,int> & pair : list_chosen_assignments){

            // We try the assignment and apply it if possible
            if(check_if_assignment_feasible(instance,
                                            instance->get_agent(pair.first),
                                            instance->get_task(pair.second))){

                // We remove the agent from the available ones
                list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                 instance->get_agent(pair.first)));
            }
        }

        // We check if some agents are still available and some goals are still open
        if (!instance->get_list_open_tasks().empty() && !list_available_agents.empty()){

            // We initialize the lists
            vector<pair<int,int> > list_pair_possible_assignment, list_h_value_per_pair;

            // We initialize the current index
            int current_index = -1;

            // For each task in the list of open ones
            for (Task * task : instance->get_list_open_tasks()){

                // For each agent in the list of available agents
                for (Agent * agent : list_available_agents){

                    // We increment the current index
                    ++ current_index;

                    // We create the corresponding pair
                    list_pair_possible_assignment.push_back(pair<int,int> (agent->get_id(),task->get_id()));

                    // We add the h value in the list
                    list_h_value_per_pair.push_back( pair<int,int> (
                            instance->get_h_values_per_node()[agent->get_current_location()][task->get_pickup_node()],
                            current_index));
                }
            }

            // We sort the list by h value
            sort(list_h_value_per_pair.begin(),list_h_value_per_pair.end());

            // While the lists are not empty
            while (!list_h_value_per_pair.empty() && !list_available_agents.empty() &&
                   !instance->get_list_open_tasks().empty()){

                // We get the index of the first pair in the list
                int index_to_check = list_h_value_per_pair[0].second;

                // We remove this value from the list
                list_h_value_per_pair.erase(list_h_value_per_pair.begin());

                // We check that the agent is still available
                if (find(list_available_agents.begin(),list_available_agents.end(),
                         instance->get_agent(list_pair_possible_assignment[index_to_check].first))
                    == list_available_agents.end()) continue;

                // We check that the task is still available
                if (find(instance->get_list_open_tasks().begin(), instance->get_list_open_tasks().end(),
                         instance->get_task(list_pair_possible_assignment[index_to_check].second))
                    == instance->get_list_open_tasks().end()) continue;

                // We try the assignment and apply it if possible
                if(check_if_assignment_feasible(instance,
                                                instance->get_agent(list_pair_possible_assignment[index_to_check].first),
                                                instance->get_task(list_pair_possible_assignment[index_to_check].second))){

                    // We remove the agent from the available ones
                    list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                     instance->get_agent(
                                                             list_pair_possible_assignment[index_to_check].first)));
                }
            }
        }

        // For each remaining agent
        for (Agent * agent_remaining : list_available_agents){

            // We initialize the boolean value
            bool move = false;

            // We check if the agent has to move from its current position
            for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
                 it != instance->get_list_open_tasks().end(); it++) {

                // We check if the delivery location correponds with the agent's current location
                if ((*it)->get_delivery_node() == agent_remaining->get_current_location()) {
                    move = true;
                    break;
                }
            }

            // We check if a move is necessary
            if (move) {

                // We move the agent
                compute_move_to_endpoint(instance,agent_remaining);

                if (allow_modification_endpoint){

                    // We update the finish time of the agent
                    agent_remaining->set_finish_time(instance->get_current_time_step() + 1);
                }
            }
            else {

                // We update the finish time of the agent
                agent_remaining->set_finish_time(agent_remaining->get_finish_time() + 1);
            }
        }

        // We increment the current time step
        instance->set_current_time_step(instance->get_current_time_step() + 1);
    }
}

void Resolution_Method::compute_set_partitioning_assignment(Instance * instance,
                                                                     vector<Task *> & list_open_goals,
                                                                     vector<Agent *> & list_available_agents,
                                                                     vector<pair<int,int> > & chosen_assignments){

    // -----------------------------------------------------------------------------
    // ENVIRONMENT
    // -----------------------------------------------------------------------------

    IloEnv env;
    IloModel model(env);

    // We get the general values
    int nb_task = list_open_goals.size();
    int nb_agent = list_available_agents.size();
    int nb_required_assignment = 0;

    // We compute the required number of assignment
    if (nb_agent <= nb_task){
        nb_required_assignment = nb_agent;
    }
    else {
        nb_required_assignment = nb_task;
    }

    // We check that both values are greater than 0
    if (nb_agent == 0 || nb_task == 0) {
        return;
    }

    try {

        // -----------------------------------------------------------------------------
        // DECISION VARIABLES
        // -----------------------------------------------------------------------------

        // Xij variables
        IloArray<IloBoolVarArray> xij(env, nb_task);
        for(int i = 0; i < nb_task; i++){
            xij[i] = IloBoolVarArray(env);
            for (int j = 0; j < nb_agent; ++j){
                string name_current = "Assignment_" + to_string(i) + "_" + to_string(j) + "";
                xij[i].add(IloBoolVar(env, name_current.c_str()));
            }
        }

        // Delta variable
        IloIntVar Delta(env);

        // -----------------------------------------------------------------------------
        // OBJECTIVE FUNCTION
        // -----------------------------------------------------------------------------

        // We add the objective function to the problem
        model.add(IloMinimize(env, Delta));

        // -----------------------------------------------------------------------------
        // CONSTRAINTS
        // -----------------------------------------------------------------------------

        // At most 1 assignment per agent + Link between variables

        // For each agent
        for (int current_agent = 0; current_agent < nb_agent; ++current_agent){

            // We create the Ilo Expression
            IloExpr constraint_agent(env);

            // We create the Ilo Expression for the link between variables
            IloExpr constraint_link(env);

            // For each task
            for (int current_task = 0; current_task < nb_task; ++current_task){

                // We add the task in the agent's constraint
                constraint_agent += xij[current_task][current_agent];

                // We add the task and its value to the link constraint
                constraint_link += instance->get_h_values_per_node()[
                                           list_available_agents[current_agent]->get_current_location()][
                                           list_open_goals[current_task]->get_pickup_node()] *
                                   xij[current_task][current_agent];
            }

            // We add the constraint in the model
            model.add(constraint_agent <= 1);

            // We add the link constrain in the model
            model.add(Delta >= constraint_link);
        }

        // At most 1 assignment per task + Required number of assignment

        // We create the Ilo Expression for the required number of assignments
        IloExpr constraint_required_assignment(env);

        // For each task
        for (int current_task = 0; current_task < nb_task; ++current_task){

            // We create the Ilo Expression
            IloExpr constraint_task(env);

            // For each agent
            for (int current_agent = 0; current_agent < nb_agent; ++current_agent){

                // We add the agent in the task's constraint
                constraint_task += xij[current_task][current_agent];

                // We add the value in the assignments' constraint
                constraint_required_assignment += xij[current_task][current_agent];
            }

            // We add the constraint in the model
            model.add(constraint_task <= 1);
        }

        // We add the required assignments' constraint in the model
        model.add(constraint_required_assignment == nb_required_assignment);

        // -----------------------------------------------------------------------------
        // RESOLUTION
        // -----------------------------------------------------------------------------

        // We create the cplex
        IloCplex cplex(model);

        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::TiLim, 3600);
        cplex.setParam(IloCplex::Threads, 1);
        //cplex.exportModel("Model.lp");

        if (cplex.solve()){

            //cout << "A solution exists and the value is " << cplex.getObjValue() << endl;

            // Sortir les selected values
            for (int current_task = 0; current_task < nb_task; ++current_task){

                // For each agent
                for (int current_agent = 0; current_agent < nb_agent; ++current_agent){

                    // We check if the assignment is used
                    if (cplex.getValue(xij[current_task][current_agent]) > 0.5){

                        chosen_assignments.push_back(pair<int,int> (list_available_agents[current_agent]->get_id(),
                                                                    list_open_goals[current_task]->get_id()));
                    }
                }
            }
        }
        else {
            cout << "Infeasible problem in the set partitioning" << endl;
        }
    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();
}

void Resolution_Method::solve_Greedy_Heuristic_Wait(Instance * instance){

    while (instance->get_nb_task_scheduled() < instance->get_list_tasks().size() &&
           instance->get_current_time_step() <= instance->get_max_horizon()) {

        // We update the list of open goals for the current time step
        for (int id_task : instance->get_id_released_tasks_per_time_step()[instance->get_current_time_step()]) {
            if (find(instance->get_list_open_tasks().begin(),
                     instance->get_list_open_tasks().end(),
                     instance->get_list_tasks()[id_task]) == instance->get_list_open_tasks().end() &&
                instance->get_list_tasks()[id_task]->get_id_assigned_agent() == -1){

                instance->get_list_open_tasks().push_back(instance->get_task(id_task));
            }
        }

        // We initialize the list of available agents
        vector<Agent *> list_available_agents;

        // We get the list of available agents
        for (Agent * agent : instance->get_list_agents()){
            if (agent->get_finish_time() == instance->get_current_time_step()){

                // We add the agent in the list
                list_available_agents.push_back(agent);

                // We update the agent current location
                agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);
            }
        }

        // We check if we can assign goals at this time step
        if (instance->get_current_time_step() % instance->get_wait_value() == 0){

            // We initialize the lists
            vector<pair<int,int> > list_pair_possible_assignment, list_h_value_per_pair;

            // We initialize the current index
            int current_index = -1;

            // For each task in the list of open ones
            for (Task * task : instance->get_list_open_tasks()){

                // For each agent in the list of available agents
                for (Agent * agent : list_available_agents){

                    // We increment the current index
                    ++ current_index;

                    // We create the corresponding pair
                    list_pair_possible_assignment.push_back(pair<int,int> (agent->get_id(),task->get_id()));

                    // We add the h value in the list
                    list_h_value_per_pair.push_back( pair<int,int> (
                            instance->get_h_values_per_node()[agent->get_current_location()][task->get_pickup_node()],
                            current_index));
                }
            }

            // We sort the list by h value
            sort(list_h_value_per_pair.begin(),list_h_value_per_pair.end());

            // While the lists are not empty
            while (!list_h_value_per_pair.empty() && !list_available_agents.empty() &&
                   !instance->get_list_open_tasks().empty()){

                // We get the index of the first pair in the list
                int index_to_check = list_h_value_per_pair[0].second;

                // We remove this value from the list
                list_h_value_per_pair.erase(list_h_value_per_pair.begin());

                // We check that the agent is still available
                if (find(list_available_agents.begin(),list_available_agents.end(),
                         instance->get_agent(list_pair_possible_assignment[index_to_check].first))
                    == list_available_agents.end()) continue;

                // We check that the task is still available
                if (find(instance->get_list_open_tasks().begin(), instance->get_list_open_tasks().end(),
                         instance->get_task(list_pair_possible_assignment[index_to_check].second))
                    == instance->get_list_open_tasks().end()) continue;

                // We try the assignment and apply it if possible
                if(check_if_assignment_feasible(instance,
                                                instance->get_agent(list_pair_possible_assignment[index_to_check].first),
                                                instance->get_task(list_pair_possible_assignment[index_to_check].second))){

                    // We remove the agent from the available ones
                    list_available_agents.erase(find(list_available_agents.begin(),list_available_agents.end(),
                                                     instance->get_agent(
                                                             list_pair_possible_assignment[index_to_check].first)));
                }
            }
        }

        // For each remaining agent
        for (Agent * agent_remaining : list_available_agents){

            // We initialize the boolean value
            bool move = false;

            // We check if the agent has to move from its current position
            for (vector<Task*>::iterator it = instance->get_list_open_tasks().begin();
                 it != instance->get_list_open_tasks().end(); it++) {

                // We check if the delivery location correponds with the agent's current location
                if ((*it)->get_delivery_node() == agent_remaining->get_current_location()) {
                    move = true;
                    break;
                }
            }

            // We check if a move is necessary
            if (move) {

                // We move the agent
                compute_move_to_endpoint(instance,agent_remaining);

                if (allow_modification_endpoint){

                    // We update the finish time of the agent
                    agent_remaining->set_finish_time(instance->get_current_time_step() + 1);
                }
            }
            else {

                // We update the finish time of the agent
                agent_remaining->set_finish_time(agent_remaining->get_finish_time() + 1);
            }
        }

        // We increment the current time step
        instance->set_current_time_step(instance->get_current_time_step() + 1);
    }
}

bool Resolution_Method::apply_TOTP(Instance * instance, Agent * agent){

    // We update the current location for the agent
    agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);

    // We get the list of the used endpoints by the other agents
    vector<bool> hold(instance->get_nb_column()*instance->get_nb_row(), false);
    for (unsigned int i = 0; i < instance->get_nb_agent(); i++) {

        // We update the matrix
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

        if (this->solve_type == 8){

            return this->check_if_assignment_feasible(instance,agent,task);

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
    }

    // We return false
    return false;
}

bool Resolution_Method::apply_TOTP_2(Instance * instance, Agent * agent){

    // We update the current location for the agent
    agent->set_current_location(agent->get_path()[instance->get_current_time_step()]);

    // We initialize the lists
    vector<int> max_reach_time_pickup_per_goal, max_reach_time_delivery_per_goal;
    vector<pair<int,int> > list_h_value_per_goal;

    // We initialize the current index
    int current_index = -1;

    // For each open goal
    for (Task * task : instance->get_list_open_tasks()){

        // We increment the curren index value
        ++ current_index;

        // For each agent of the problem
        for (Agent * agent_to_check : instance->get_list_agents()){

            // We check that the agent is not the same
            if (agent_to_check == agent) continue;

            // We add the default values in the list
            max_reach_time_pickup_per_goal.push_back(instance->get_max_horizon());
            max_reach_time_delivery_per_goal.push_back(instance->get_max_horizon());

            // We check if the pickup node is the other agent's end point
            if (agent_to_check->get_path()[instance->get_max_horizon()-1] == task->get_pickup_node()){

                // We get the time step at which the agent reaches this node
                for (int k = instance->get_max_horizon()-1; k >= 0; --k){

                    // We check if the agent to check is at this node at the time step k
                    if (agent_to_check->get_path()[k] == task->get_pickup_node()){

                        // We update the value in the list
                        max_reach_time_pickup_per_goal[current_index] = k;
                    }
                    else {

                        // We stop the process
                        break;
                    }
                }
            }

            // We check if the delivery node is the other agent's end point
            if (agent_to_check->get_path()[instance->get_max_horizon()-1] == task->get_delivery_node()){

                // We get the time step at which the agent reaches this node
                for (int k = instance->get_max_horizon()-1; k >= 0; --k){

                    // We check if the agent to check is at this node at the time step k
                    if (agent_to_check->get_path()[k] == task->get_delivery_node()){

                        // We update the value in the list
                        max_reach_time_delivery_per_goal[current_index] = k;
                    }
                    else {

                        // We stop the process
                        break;
                    }
                }
            }
        }

        // We add the goal in the list according to its h value
        list_h_value_per_goal.push_back(pair<int,int> (
                instance->get_h_values_per_node()[agent->get_current_location()][task->get_pickup_node()],
                current_index));
    }

    // We sort the open goals per h value from the current node of the agent
    sort(list_h_value_per_goal.begin(),list_h_value_per_goal.end());


    // While no assignment has been done and the list of goals is not empty
    while (!list_h_value_per_goal.empty()){

        // We take the closest goal
        int current_index = list_h_value_per_goal[0].second;

        // We remove this goal from the list of open ones
        list_h_value_per_goal.erase(list_h_value_per_goal.begin());

        // We check if a feasible path is possible to this goal using the new A star algorithm
        if (check_if_assignment_feasible(instance,agent,instance->get_list_open_tasks()[current_index])){

            // We return true
            return true;
        }
    }

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

        int previous_finish_time = agent->get_finish_time();

        // We try to move the agent to another free endpoint
        if (compute_move_to_endpoint(instance,agent)) {

            // We check if we allow modification through endpoint movement
            if (allow_modification_endpoint){
                agent->set_finish_time(previous_finish_time + 1);
            }

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

    // We return false
    return false;
}

int Resolution_Method::return_h_value_endpoint(Instance * instance, Agent * agent, Task * task,
                                               vector<int> & list_endpoints_used){

    // We get the list of the endpoints used by the other agents
    for (Agent * agent_to_check : instance->get_list_agents()){

        // We check if the agent is the same
        if (agent_to_check == agent){

            continue;
        }

        // We add the agent endpoint to the current list
        list_endpoints_used.push_back(agent_to_check->get_path()[instance->get_max_horizon()-1]);
    }

    // We initialize the minimum value to a free endpoint
    int h_value_endpoint = 0;

    // We check if the current node is in the list
    if (find(list_endpoints_used.begin(), list_endpoints_used.end(), task->get_delivery_node()) ==
            list_endpoints_used.end()){

        // We return the h value
        return h_value_endpoint;
    }

    // We increment the h value
    ++ h_value_endpoint;

    // While an endpoint has not been found
    while (true){

        // For all the nodes
        for (int k = 0; k < instance->get_nb_column() * instance->get_nb_row(); ++k){

            // We check if the h value of the node correspond to the current h value
            if (instance->get_h_values_per_node()[task->get_delivery_node()][k] == h_value_endpoint){

                // We check if the current node is in the list
                if (find(list_endpoints_used.begin(), list_endpoints_used.end(), k) ==
                        list_endpoints_used.end()){

                    // We return the current h value
                    return h_value_endpoint;
                }
            }
        }

        // We increment the current h value
        ++ h_value_endpoint;

        // We check if no endpoint are feasible
        if (h_value_endpoint > instance->get_nb_column() * instance->get_nb_row()){

            cout << "Problem, no endpoint is feasible for the current task " << endl;
            getchar();
        }
    }
}

int Resolution_Method::compute_max_reach_pickup_node(Instance * instance, Agent * agent, Task * task,
                                   vector<int> & list_endpoints_used){

    // We check if the pickup node is in the list of used endpoints
    if (find(list_endpoints_used.begin(),list_endpoints_used.end(),task->get_pickup_node()) ==
            list_endpoints_used.end()){

        // We return the max time
        return instance->get_max_horizon() - 1;
    }
    else {

        // For each agent
        for (Agent * agent_to_check : instance->get_list_agents()){

            // We check if it is the same agent
            if (agent_to_check == agent) continue;

            // We check if the endpoint of the agent corresponds
            if (agent_to_check->get_path()[instance->get_max_horizon()-1] == task->get_pickup_node()){

                // We initialize the max reach value
                int max_reach_value = instance->get_max_horizon() - 1;

                // While the agent to check stays at this position
                while (true){

                    // We decrement the max value
                    -- max_reach_value;

                    // We check if the first position has been reach
                    if (max_reach_value < 0){

                        return max_reach_value;
                    }

                    // We check if the agent is not at the pickup position
                    if (agent_to_check->get_path()[max_reach_value] != task->get_pickup_node()){

                        // We return the current value
                        return max_reach_value;
                    }
                }
            }
        }
    }

    cout << "Problem, end of the method reached " << endl;
    getchar();
    return -1;
}

int Resolution_Method::compute_max_reach_delivery_node(Instance * instance, Agent * agent, Task * task,
                                                     vector<int> & list_endpoints_used){

    // We check if the delivery node is in the list of used endpoints
    if (find(list_endpoints_used.begin(),list_endpoints_used.end(),task->get_delivery_node()) ==
        list_endpoints_used.end()){

        // We return the max time
        return instance->get_max_horizon() - 1;
    }
    else {

        // For each agent
        for (Agent * agent_to_check : instance->get_list_agents()){

            // We check if it is the same agent
            if (agent_to_check == agent) continue;

            // We check if the endpoint of the agent corresponds
            if (agent_to_check->get_path()[instance->get_max_horizon()-1] == task->get_delivery_node()){

                // We initialize the max reach value
                int max_reach_value = instance->get_max_horizon() - 1;

                // While the agent to check stays at this position
                while (true){

                    // We decrement the max value
                    -- max_reach_value;

                    // We check if the first position has been reach
                    if (max_reach_value < 0){

                        return max_reach_value;
                    }

                    // We check if the agent is not at the pickup position
                    if (agent_to_check->get_path()[max_reach_value] != task->get_delivery_node()){

                        // We return the current value
                        return max_reach_value;
                    }
                }
            }
        }
    }

    cout << "Problem, end of the method reached " << endl;
    getchar();
    return -1;
}

bool Resolution_Method::check_if_assignment_feasible(Instance * instance, Agent * agent, Task * task){

    // We get the current location
    int current_location = agent->get_current_location();

    // We get the pickup location
    int pickup_location = task->get_pickup_node();

    // We get the delivery location
    int delivery_location = task->get_delivery_node();

    // We get the h value from the pickup to the delivery
    int h_val_pd = instance->get_h_values_per_node()[pickup_location][delivery_location];

    // We create the list of endpoints used by the other agents
    vector<int> list_endpoints_used;

    // We get the h value to the closest free endpoint
    int h_val_endpoint = return_h_value_endpoint(instance,agent,task,list_endpoints_used);

    // We compute the interval until which the pickup node is accesible
    int max_reach_pickup = compute_max_reach_pickup_node(instance,agent,task,list_endpoints_used);

    // We compute the interval until which the delivery node is accessible
    int max_reach_delivery = compute_max_reach_delivery_node(instance,agent,task,list_endpoints_used);

    // We check if we are able to stay at the delivery location
    if (!allow_move_after_endpoint && max_reach_delivery < instance->get_max_horizon()-1){

        // We return false
        return false;
    }

    if (max_reach_delivery - h_val_pd < max_reach_pickup){

        // We update the max reach pickup value
        max_reach_pickup = max_reach_delivery - h_val_pd;
    }

    // We initialize the values
    complex_heap_open_t open_list;
    map<string, Complex_Node*> allNodes_table; // key = type*g_val*map_size + loc

    // We generate the start node
    Complex_Node * start = new Complex_Node(current_location, 0,
                                            instance->get_h_values_per_node()[current_location][pickup_location]
                                            + h_val_pd, NULL, agent->get_finish_time(), 1, false);

    // We add the first node in the list
    open_list.push(start);
    start->in_openlist = true;
    instance->add_created_search_node();
    allNodes_table.insert(make_pair(to_string(current_location)+"_0_1", start)); // g_val = 0 --> key = loc

    // We initialize the value of the best node
    Complex_Node * best_node = NULL;

    // While some nodes are still in the open list
    while (!open_list.empty()) {

        // We take the first node in the list
        Complex_Node * current_node = open_list.top();
        open_list.pop();

        // We increment the number of checked node
        instance->add_checked_search_node();

        // We update the open list value for the current node
        current_node->in_openlist = false;

        // We check the timestep of the current node
        if (current_node->timestep >= instance->get_max_horizon() - 1) continue;

        // We check that the node can still reach the pickup in time
        if (current_node->type == 1 && current_node->timestep + current_node->h_val > max_reach_pickup) continue;

        // We check that the node can still reach the delivery in time
        if (current_node->type == 1 && current_node->timestep + current_node->h_val + h_val_pd >
                                               max_reach_delivery) continue;

        // We check that the node can still reach the delivery in time
        if (current_node->type == 2 && current_node->timestep + current_node->h_val > max_reach_delivery) continue;

        // We check if we have reached a goal
        if (current_node->type == 1 && current_node->location == pickup_location){

            // We compute the successor values
            int next_g_val = current_node->g_val;
            int next_h_val = h_val_pd;

            // We generate the corresponding node
            Complex_Node * next = new Complex_Node(current_node->location, next_g_val, next_h_val, current_node,
                                           current_node->timestep, 2, false);

            // We check if the current successor has been checked before
            map<string, Complex_Node * >::iterator it = allNodes_table.find(
                    to_string(next->location)+"_"+to_string(next->g_val)+"_"+to_string(next->type));

            if (it == allNodes_table.end()) //undiscovered
            {  // add the newly generated node to open_list and hash table

                // We udate open list value for the node
                next->in_openlist = true;

                // We insert the current node in the list of all the nodes
                allNodes_table.insert(pair<string, Complex_Node*>(
                        to_string(next->location)+"_"+to_string(next->g_val)+"_"+to_string(next->type), next));

                // We add the node to the open list
                open_list.push(next);

                // We increment the number of created node
                instance->add_created_search_node();
            }

            else //discovered
            {
                // We delete the created node
                delete(next);
            }

            continue;

        }
        else if (current_node->type == 2 && current_node->location == delivery_location){

            // We compute the successor values
            int next_g_val = current_node->g_val;
            int next_h_val = 0;

            // We generate the corresponding node
            Complex_Node * next = new Complex_Node(current_node->location, next_g_val, next_h_val, current_node,
                                           current_node->timestep, 3, false);

            // We check if the current successor has been checked before
            map<string, Complex_Node * >::iterator it = allNodes_table.find(
                    to_string(next->location)+"_"+to_string(next->g_val)+"_"+to_string(next->type));

            if (it == allNodes_table.end()) //undiscovered
            {  // add the newly generated node to open_list and hash table

                // We udate open list value for the node
                next->in_openlist = true;

                // We insert the current node in the list of all the nodes
                allNodes_table.insert(pair<string, Complex_Node*>(
                        to_string(next->location)+"_"+to_string(next->g_val)+"_"+to_string(next->type), next));

                // We add the node to the open list
                open_list.push(next);

                // We increment the number of created node
                instance->add_created_search_node();
            }

            else //discovered
            {
                // We delete the created node
                delete(next);
            }

            continue;
        }
        else if (current_node->type == 3 &&
                find(list_endpoints_used.begin(),list_endpoints_used.end(),current_node->location)
                == list_endpoints_used.end()){

            if (instance->get_list_endpoints()[current_node->location]) {

                // We initialize the boolean value
                bool end_found = true;

                // We check that the current node is node used by an agent during the following time step
                for (Agent * agent_to_check : instance->get_list_agents()){

                    // We check if it is the same agent
                    if (agent_to_check == agent) continue;

                    // For each following time step
                    for (int ts = current_node->timestep + 1; ts < instance->get_max_horizon(); ++ts){

                        // We check if the agent use the current node
                        if (agent_to_check->get_path()[ts] == current_node->location){

                            // We update the boolean value
                            end_found = false;
                        }
                    }
                }

                // We check if the final node has been found
                if (end_found){

                    // We update the value of the best node
                    best_node = current_node;

                    // We stop the search
                    break;
                }
            }
        }

        // We intialize the successor id
        int successor_id;

        // We create the list of possible moves
        int action[5] = {0, 1, -1, instance->get_nb_column(), -(instance->get_nb_column())};

        for (int i = 0; i < 5;i++) {

            // We check if the move is feasible
            if (i == 1 && current_node->location % (instance->get_nb_column()) == instance->get_nb_column() - 1) {
                continue;
            }
            if (i == 2 && current_node->location % (instance->get_nb_column()) == 0) {
                continue;
            }
            if (i == 3 && current_node->location / (instance->get_nb_column()) == instance->get_nb_row() - 1) {
                continue;
            }
            if (i == 4 && current_node->location / (instance->get_nb_column()) == 0) {
                continue;
            }

            // We get the location of the successor
            successor_id = current_node->location + action[i];

            // We initialize the current time step value
            int next_timestep = current_node->timestep + 1;

            // We check if the successor is accessible
            if (!isConstrained(instance, agent, current_node->location, successor_id, next_timestep))
            {

                // We compute the successor g value
                int next_g_val = current_node->g_val + 1, next_h_val;

                // We compute the successor h value
                if (current_node->type == 1){
                    next_h_val = instance->get_h_values_per_node()[successor_id][pickup_location] +
                            h_val_pd;
                }
                else if (current_node->type == 2){
                    next_h_val = instance->get_h_values_per_node()[successor_id][delivery_location];
                }
                else {
                    next_h_val = 0;
                }

                // We generate the corresponding node
                Complex_Node * next = new Complex_Node(successor_id, next_g_val, next_h_val, current_node,
                                                       next_timestep, current_node->type, false);

                // We check if the current successor has been checked before
                map<string, Complex_Node* >::iterator it = allNodes_table.find(
                        to_string(next->location)+"_"+to_string(next->g_val)+"_"+to_string(next->type));

                if (it == allNodes_table.end()) //undiscovered
                {  // add the newly generated node to open_list and hash table

                    // We udate open list value for the node
                    next->in_openlist = true;

                    // We insert the current node in the list of all the nodes
                    allNodes_table.insert(pair<string, Complex_Node*>(
                            to_string(next->location)+"_"+to_string(next->g_val)+"_"+to_string(next->type), next));

                    // We add the node to the open list
                    open_list.push(next);

                    // We increment the number of created node
                    instance->add_created_search_node();
                }

                else //discovered
                {
                    // We delete the created node
                    delete(next);
                }
            }
        }
    }

    // We check if a best complex node has been found
    if (best_node != NULL){

        // We update the agent path
        Update_Path(agent,*best_node);

        // We initialize the pickup and delivery times
        int pickup_time = -1, delivery_time = -1;

        // We get the pickup and delivery times
        for (int ts = instance->get_current_time_step(); ts < instance->get_max_horizon(); ++ ts){

            // We check if the agent is at the pickup location
            if (agent->get_path()[ts] == task->get_pickup_node() && pickup_time == -1){

                // We update the pickup time
                pickup_time = ts;
            }

            // We check if the agent is at the delivery location
            if (agent->get_path()[ts] == task->get_delivery_node() && pickup_time != -1 &&
                   delivery_time == -1){

                // We update the pickup time
                delivery_time = ts;
            }

            // We check if both values have been updated
            if (pickup_time != -1 && delivery_time != -1){

                // We break the process
                break;
            }
        }

        // We check the solve type
        if (!allow_modification_endpoint){

            // We update the finish time for the agent
            agent->set_finish_time(best_node->timestep);
        }
        else {

            // We update the finish time for the agent
            agent->set_finish_time(delivery_time);
        }

        // We apply the assignment
        instance->apply_assignment(agent->get_id(),task->get_id(),pickup_time,delivery_time);

        // We release the nodes
        releaseClosedListComplexNodes(allNodes_table);

        // We return true
        return true;
    }
    else {

        // We release the nodes
        releaseClosedListComplexNodes(allNodes_table);

        // We return false
        return false;
    }
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

    // We increment the number of created node
    instance->add_created_search_node();

    // We add the first node in the list
    open_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(make_pair(start_location, start)); //g_val=0 --> key=loc

    while (!open_list.empty())
    {
        // We take the first node in the list
        Node * current_node = open_list.top();
        open_list.pop();

        // We increment the number of checked node
        instance->add_checked_search_node();

        // We update the open list value for the current node
        current_node->in_openlist = false;

        // We check if we have reach the goal
        if (current_node->loc == goal_location) {

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

        for (int i = 0; i < 5;i++) {

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

                    // We increment the number of created node
                    instance->add_created_search_node();
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

void Resolution_Method::Update_Path(Agent * agent, const Complex_Node & goal) {

    // We update the path of the agent until the end of the horizon
    for (int i = goal.timestep + 1; i < agent->get_path().size(); i++)
    {
        agent->get_path()[i] = goal.location;
    }

    // We search for the path for the agent
    const Complex_Node* current_node = &goal;
    while (current_node!=NULL)
    {
        // We update the agent path
        agent->get_path()[current_node->timestep] = current_node->location;

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

void Resolution_Method::releaseClosedListComplexNodes(map<string, Complex_Node*> & allNodes_table){

    // We initialize the iterator
    map<string, Complex_Node*>::iterator it;

    // For each node in the table
    for (it = allNodes_table.begin(); it != allNodes_table.end(); it++){

        // We delete the node
        delete ((*it).second);
    }

    // We clear the table
    allNodes_table.clear();
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
