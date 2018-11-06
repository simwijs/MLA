//
// Created by Florian Grenouilleau on 2018-10-30.
//

#ifndef MAPD_V2_AGENT_H
#define MAPD_V2_AGENT_H

using namespace std;

#include <vector>
#include <iostream>

class Agent {

private:
    int id, finish_time, current_location;
    vector<int> path;

public:

    // Constructor
    Agent(int id_) : id(id_), finish_time(0) {};
    Agent(int id_, int start_location, int max_horizon) : id(id_), finish_time(0), current_location(-1) {

        // We resize the agent's path
        path.resize(max_horizon,start_location);
    };

    // General Methods
    void write();

    // Setters
    void set_finish_time(int value){this->finish_time = value;}
    void set_current_location(int value){this->current_location = value;}

    // Getters
    int get_id(){ return this->id;}
    int get_finish_time(){ return this->finish_time;}
    int get_current_location(){ return this->current_location;}
    vector<int> & get_path(){ return this->path;};
};
#endif //MAPD_V2_AGENT_H
