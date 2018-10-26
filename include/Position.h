//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_POSITION_H
#define MAPD_POSITION_H

#include <vector>

using namespace std;

class Position {

private:

    // Attributes
    int id_agent, id_node, time_step, assigned_task;

public:

    // Constructor
    Position(int id_agent_, int id_node_, int time_step_) : id_agent(id_agent_), id_node(id_node_),
                                                            time_step(time_step_), assigned_task(-1) {};

    Position(int id_agent_, int id_node_, int time_step_, int assigned_task_) : id_agent(id_agent_), id_node(id_node_),
                                                            time_step(time_step_), assigned_task(assigned_task_) {};

    // Setters
    void set_id_node(int value){this->id_node = value;};
    void set_assigned_task(int value){this->assigned_task = value;};

    // Getters
    int get_id_agent(){ return this->id_agent;}
    int get_id_node(){ return this->id_node;}
    int get_time_step(){ return this->time_step;}
    int get_assigned_task(){ return this->assigned_task;}
};
#endif //MAPD_POSITION_H
