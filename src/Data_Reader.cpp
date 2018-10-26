//
// Created by Florian Grenouilleau on 2018-10-24.
//

#include "../include/Data_Reader.h"
#include <iostream>
#include <fstream>

void Data_Reader::read_instance(Instance * instance, Solution * solution){

    // We read the map of the problem
    read_map_file(instance,solution);

    // We read the tasks of the problem
    read_task_file(instance,solution);

    // We update the task values with the corresponding associated nodes
    for (Task * task : instance->get_list_tasks()){
        task->set_pickup_node(instance->get_list_pair_endpoint_node()[task->get_pickup_node()].second);
        task->set_delivery_node(instance->get_list_pair_endpoint_node()[task->get_delivery_node()].second);
    }

    // We update the successor lists for the nodes
    instance->compute_successors_per_node();
}

void Data_Reader::read_task_file(Instance * instance, Solution * solution){

    // We open the file
    ifstream file(instance->get_task_file_name());


    if (file.is_open()) {

        // We initialize the value
        string line,value;
        getline(file, line);

        // We read the number of task
        int nb_tasks = stoi(line);

        // For each task
        for (int task = 0; task < nb_tasks; ++task){

            // We get the release date
            file >> value;
            int release_date = stoi(value);

            // We get the pickup node
            file >> value;
            int pickup_node = stoi(value);

            // We get the delivery node
            file >> value;
            int delivery_node = stoi(value);

            // We don't use the last two values
            file >> value;
            file >> value;

            // We create a new task in the instance's list
            instance->get_list_tasks().push_back(new Task(task,release_date,pickup_node,delivery_node));
        }

        // We close the file
        file.close();
    }
}

void Data_Reader::read_map_file(Instance * instance, Solution * solution){

    // We open the file
    ifstream file(instance->get_map_file_name());

    if (file.is_open()) {

        // We initialize the value
        string line,value;

        // We read the number of row
        file >> value;
        instance->set_nb_row(stoi(value));

        // We read the number of column
        file >> value;
        instance->set_nb_column(stoi(value));

        // We read the number of endpoints
        file >> value;
        instance->set_nb_endpoint(stoi(value));

        // We create the list of agents
        file >> value;
        int nb_agents = stoi(value);
        instance->set_nb_agent(nb_agents);

        // We read the not used value
        getline(file,line);
        getline(file,line);

        // We initialize the values
        int nb_found_agents = 0, current_node_id = -1, nb_found_endpoint = 0;

        // For each row
        for (int row = 0; row < instance->get_nb_row(); ++row){

            // We get the corresponding line
            getline(file,line);

            // For each column
            for (int column = 0; column < instance->get_nb_column(); ++column){

                // We increment the id of the current node
                ++ current_node_id;

                // We get the next node value
                value = line.at(column);

                // We get the type of the node
                if (value == "."){
                    // We create the free node
                    instance->get_list_nodes().push_back(new Node (current_node_id,0,row,column));
                }
                else if (value == "e"){
                    // We create the endpoint node
                    instance->get_list_nodes().push_back(new Node (current_node_id,0,row,column));

                    // We update the list of pair endpoint-node
                    instance->get_list_pair_endpoint_node().push_back(pair<int,int> (
                            nb_found_endpoint,current_node_id));

                    // We increment the number of found endpoint
                    ++ nb_found_endpoint;
                }
                else if (value == "r"){
                    // We create the agent node
                    instance->get_list_nodes().push_back(new Node (current_node_id,0,row,column));

                    // We add a position in the solution
                    solution->get_list_positions_per_time_step()[0].push_back(
                            Position(nb_found_agents,current_node_id,0));

                    // We increment the number of found agents
                    ++ nb_found_agents;
                }
                else {
                    // We create the obstacle node
                    instance->get_list_nodes().push_back(new Node (current_node_id,1,row,column));
                }
            }
        }

        // We check that the number of created agents correspond
        if (nb_agents != solution->get_list_positions_per_time_step()[0].size()){
            cout << "Problem, the number of agents does not correspond" << endl;
            getchar();
        }

        // We check that the number of created endpoints correspond
        if (nb_found_endpoint != instance->get_nb_endpoint()){
            cout << "Problem, the number of endpoints does not correspond" << endl;
            getchar();
        }

        // We check that the number of node correspond
        if (instance->get_nb_row()*instance->get_nb_column() != instance->get_list_nodes().size()){
            cout << "Problem, the number of nodes does not correspond" << endl;
            getchar();
        }

        // We close the file
        file.close();
    }
}