//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_INSTANCE_H
#define MAPD_INSTANCE_H

#include <string>
#include <vector>
#include <utility>
#include "Task.h"
#include "Node.h"

using namespace std;

class Instance {

private:

    // Attributes
    int nb_row, nb_column, nb_endpoint, nb_agent;
    string map_file_name, task_file_name;
    vector<Task*> list_tasks;
    vector<Node*> list_nodes;
    vector<pair<int,int> > list_pair_endpoint_node;

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
        for (Node * node: list_nodes){
            delete node;
            node = 0;
        }
    }

    // General Methods
    void compute_successors_per_node();

    // Setters
    void set_nb_row(int value){this->nb_row = value;}
    void set_nb_column(int value){this->nb_column = value;}
    void set_nb_endpoint(int value){this->nb_endpoint = value;}
    void set_nb_agent(int value){this->nb_agent = value;}

    // Getters
    int get_nb_row(){ return this->nb_row;}
    int get_nb_column(){ return this->nb_column;}
    int get_nb_endpoint(){ return this->nb_endpoint;}
    int get_nb_agent(){ return this->nb_agent;}
    vector<Task*> & get_list_tasks(){ return this->list_tasks;}
    vector<Node*> & get_list_nodes(){ return this->list_nodes;}
    string & get_map_file_name(){ return this->map_file_name;}
    string & get_task_file_name(){ return this->task_file_name;}
    vector<pair<int,int> > & get_list_pair_endpoint_node(){ return this->list_pair_endpoint_node;};

};
#endif //MAPD_INSTANCE_H
