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
    int nb_row, nb_column, nb_endpoint, nb_agent, max_horizon, current_time_step, nb_task_scheduled, wait_value;
    string map_file_name, task_file_name;
    vector<Task*> list_tasks;
    vector<Agent*> list_agents;
    vector<Task*> list_open_tasks;
    vector<bool> list_map_nodes;
    vector<bool> list_endpoints;
    vector<pair<int,int> > list_pair_node_endpoint;
    vector<vector<int> > h_values_per_node, id_released_tasks_per_time_step;
    double computation_time = -1;
    vector<int> nb_agent_available_per_time_step;

    // Methods
    double compute_average_service_time();
    double compute_average_impact_traffic();
    double compute_average_nb_agent_avail();

public:

    // Constructor
    Instance(string map_file_name_, string task_file_name_) : nb_task_scheduled(0),
                                                              map_file_name(map_file_name_),
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
    void update_release_tasks_per_time_step();
    bool check_solution_feasible();
    void apply_assignment(int id_agent, int id_task, int arrive_start, int arrive_goal);
    void compute_final_makespan();
    void output_solution(char** argv);

    // Setters
    void set_nb_row(int value){this->nb_row = value;}
    void set_nb_column(int value){this->nb_column = value;}
    void set_nb_endpoint(int value){this->nb_endpoint = value;}
    void set_nb_agent(int value){this->nb_agent = value;}
    void set_max_horizon(int value){this->max_horizon = value;}
    void set_current_time_step(int value){this->current_time_step = value;}
    void set_wait_value(int value){this->wait_value = value;}
    void set_computation_time(double value){this->computation_time = value;}

    // Getters
    int get_nb_row(){ return this->nb_row;}
    int get_nb_column(){ return this->nb_column;}
    int get_nb_endpoint(){ return this->nb_endpoint;}
    int get_nb_agent(){ return this->nb_agent;}
    int get_max_horizon(){ return this->max_horizon;}
    int get_current_time_step(){ return this->current_time_step;}
    int get_nb_task_scheduled(){ return this->nb_task_scheduled;}
    int get_wait_value(){ return this->wait_value;}
    double get_computation_time(){ return this->computation_time;}
    vector<Task*> & get_list_tasks(){ return this->list_tasks;}
    vector<Task*> & get_list_open_tasks(){ return this->list_open_tasks;}
    vector<Agent*> & get_list_agents(){ return this->list_agents;}
    string & get_map_file_name(){ return this->map_file_name;}
    string & get_task_file_name(){ return this->task_file_name;}
    Agent * get_agent(int id_agent){ return this->list_agents[id_agent];}
    Task * get_task(int id_task){ return this->list_tasks[id_task];}
    vector<bool> & get_list_map_nodes(){ return this->list_map_nodes;}
    vector<bool> & get_list_endpoints(){ return this->list_endpoints;}
    vector<pair<int,int> > & get_list_pair_node_endpoint(){ return this->list_pair_node_endpoint;}
    vector<vector<int> > & get_h_values_per_node(){ return this->h_values_per_node;}
    vector<vector<int> > & get_id_released_tasks_per_time_step(){ return this->id_released_tasks_per_time_step;}
    vector<int> & get_nb_agent_available_per_time_step(){ return this->nb_agent_available_per_time_step;}
};
#endif //MAPD_INSTANCE_H
