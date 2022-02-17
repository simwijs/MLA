//
// Created by Florian Grenouilleau on 2018-10-24.
//

#include "../include/Data_Reader.h"
#include <iostream>
#include <fstream>

void Data_Reader::read_instance(Instance * instance){

    // We read the map of the problem
    read_map_file(instance);

    // We read the tasks of the problem
    read_task_file(instance);

    // We compute the h values
    for (int node = 0; node < instance->get_list_map_nodes().size(); ++node){

        // We create the vector
        instance->get_h_values_per_node().push_back(vector<int> ());

        // We check if the node is an available point
        if (instance->get_list_map_nodes()[node]){

            // We compute the h values for that node
            instance->compute_h_values(instance->get_h_values_per_node()[node],node);
        }
    }

    // We create the list of task released per time step
    instance->update_release_tasks_per_time_step();
}

void Data_Reader::read_task_file(Instance * instance){

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

            // Read the batch id
            file >> value;
            int batch_id= stoi(value);

            // We create a new task in the instance's list
            instance->get_list_tasks().push_back(new Task(task,release_date,
                                                          instance->get_list_pair_node_endpoint()[pickup_node].second,
                                                          instance->get_list_pair_node_endpoint()[
                                                                  delivery_node].second, batch_id));
        }

        // We close the file
        file.close();
    }
}

void Data_Reader::read_map_file(Instance * instance){

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
        file >> value;
        int max_horizon = stoi(value);
        instance->set_max_horizon(max_horizon);

        // We read the end of the line
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

                    // We update the values for the instance
                    instance->get_list_map_nodes().push_back(true);
                    instance->get_list_endpoints().push_back(false);
                }
                else if (value == "e"){

                    // We update the values for the instance
                    instance->get_list_map_nodes().push_back(true);
                    instance->get_list_endpoints().push_back(true);

                    // We add the pair endpoint-node
                    instance->get_list_pair_node_endpoint().push_back(pair<int,int> (
                            nb_found_endpoint, row*instance->get_nb_column() + column));

                    // We increment the number of found endpoint
                    ++ nb_found_endpoint;

                }
                else if (value == "r"){

                    // We update the values for the instance
                    instance->get_list_map_nodes().push_back(true);
                    instance->get_list_endpoints().push_back(true);

                    // We create a new agent for the problem
                    instance->get_list_agents().push_back(new Agent(nb_found_agents,
                                                                    row*instance->get_nb_column() + column,
                                                                    max_horizon));

                    instance->get_list_not_possible_endpoints().push_back(row*instance->get_nb_column() + column);
                    instance->get_deadline_per_not_feasible_endpoint().push_back(0);

                    // We increment the number of found agents
                    ++ nb_found_agents;
                }
                else {

                    // We update the values for the instance
                    instance->get_list_map_nodes().push_back(false);
                    instance->get_list_endpoints().push_back(false);
                }
            }
        }

        // We check that the number of created agents correspond
        if (nb_agents != instance->get_nb_agent()){
            cout << "Problem, the number of agents does not correspond" << endl;
            getchar();
        }

        // We check that the number of created endpoints correspond
        if (nb_found_endpoint != instance->get_nb_endpoint()){
            cout << "Problem, the number of endpoints does not correspond" << endl;
            getchar();
        }

        // We check that the number of node correspond
        if (instance->get_nb_row()*instance->get_nb_column() != instance->get_list_map_nodes().size()){
            cout << "Problem, the number of nodes does not correspond" << endl;
            getchar();
        }

        // We close the file
        file.close();
    }
}
