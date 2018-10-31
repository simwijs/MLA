//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_INSTANCE_H
#define MAPD_INSTANCE_H

#include <string>
#include <vector>
#include <utility>
#include "Task.h"
#include "Agent.h"

using namespace std;

class Instance {

private:

    // Attributes
    int nb_row, nb_column, nb_endpoint, nb_agent, max_horizon;
    string map_file_name, task_file_name;
    vector<Task*> list_tasks;
    vector<Agent*> list_agents;
    vector<bool> list_map_nodes;
    vector<bool> list_endpoints;
    vector<pair<int,int> > list_pair_node_endpoint;
    vector<vector<int> > h_values_per_node;

public:

    // Constructor
    Instance(string map_file_name_, string task_file_name_) : map_file_name(map_file_name_),
                                                              task_file_name(task_file_name_) {};

    // Destructor
    ~Instance(){

        for (Task * task : list_tasks){
            delete task;
            task = 0;
        }

        for (Agent * agent : list_agents){
            delete agent;
            agent = 0;
        }
    }

    // General Methods
    void compute_h_values(vector<int> & h_values, int start_location);

    // Setters
    void set_nb_row(int value){this->nb_row = value;}
    void set_nb_column(int value){this->nb_column = value;}
    void set_nb_endpoint(int value){this->nb_endpoint = value;}
    void set_nb_agent(int value){this->nb_agent = value;}
    void set_max_horizon(int value){this->max_horizon = value;}

    // Getters
    int get_nb_row(){ return this->nb_row;}
    int get_nb_column(){ return this->nb_column;}
    int get_nb_endpoint(){ return this->nb_endpoint;}
    int get_nb_agent(){ return this->nb_agent;}
    int get_max_horizon(){ return this->max_horizon;}
    vector<Task*> & get_list_tasks(){ return this->list_tasks;}
    vector<Agent*> & get_list_agents(){ return this->list_agents;}
    string & get_map_file_name(){ return this->map_file_name;}
    string & get_task_file_name(){ return this->task_file_name;}
    Agent * get_agent(int id_agent){ return this->list_agents[id_agent];}
    Task * get_task(int id_task){ return this->list_tasks[id_task];}
    vector<bool> & get_list_map_nodes(){ return this->list_map_nodes;};
    vector<bool> & get_list_endpoints(){ return this->list_endpoints;};
    vector<pair<int,int> > & get_list_pair_node_endpoint(){ return this->list_pair_node_endpoint;};
    vector<vector<int> > & get_h_values_per_node(){ return this->h_values_per_node;};
};
#endif //MAPD_INSTANCE_H
