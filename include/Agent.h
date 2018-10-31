//
// Created by Florian Grenouilleau on 2018-10-30.
//

#ifndef MAPD_V2_AGENT_H
#define MAPD_V2_AGENT_H

using namespace std;

#include <vector>

class Agent {

private:
    int id, finish_time;
    vector<int> path;

public:

    // Constructor
    Agent(int id_) : id(id_), finish_time(0) {};
    Agent(int id_, int start_location, int max_horizon) : id(id_), finish_time(0) {

        // We resize the agent's path
        path.resize(max_horizon,start_location);
    };

    // General Methods
    void write();

    // Getters
    int get_id(){ return this->id;}
    int get_finish_time(){ return this->finish_time;}
    vector<int> & get_path(){ return this->path;};

};
#endif //MAPD_V2_AGENT_H
