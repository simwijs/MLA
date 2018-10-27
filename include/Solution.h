//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_SOLUTION_H
#define MAPD_SOLUTION_H

#include <vector>
#include "Position.h"
#include "Instance.h"

using namespace std;

class Solution {

private:

    // Attributes
    Instance * instance = nullptr;
    vector<vector<Position> > list_positions_per_time_step;
    vector<int> list_open_tasks;
    vector<vector<int> > positions_matrix;
    double computation_time;

    // General methods
    void apply_position(Position & position);
    double compute_average_service_time();

public:

    // Constructor
    Solution(Instance * instance_) : instance(instance_){

        // We create the first set of positions for the initial ones
        this->list_positions_per_time_step.push_back(vector<Position> ());
    };

    // General methods
    void update_list_open_tasks(int current_time_step);
    void apply_assignment(int id_task, int id_agent, vector<Position> & list_new_positions);
    void create_wait_positions(int current_time_step);
    Position return_position(int id_agent, int time_step);
    void compute_positions_matrix();
    bool check_solution_feasible();
    void write();
    void output_solution();
    void set_computation_time(double value){this->computation_time = value;}

    // Getters
    Instance * get_instance(){ return this->instance;}
    vector<vector<Position> > & get_list_positions_per_time_step(){ return this->list_positions_per_time_step;}
    vector<int> & get_list_open_tasks() { return this->list_open_tasks;};
    vector<vector<int> > & get_positions_matrix(){ return this->positions_matrix;};
};
#endif //MAPD_SOLUTION_H
