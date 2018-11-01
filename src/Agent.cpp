//
// Created by Florian Grenouilleau on 2018-10-30.
//

#include <iostream>
#include "../include/Agent.h"

void Agent::write(){
    cout << "Agent id " << this->id << endl;
    cout << "Agent finish time" << this->finish_time << endl;
}