//
// Created by Florian Grenouilleau on 2018-10-26.
//

#include <iostream>
#include "../include/Position.h"

using namespace std;

void Position::write() {
    cout << "Position agent " << this->id_agent << endl;
    cout << "Position node " << this->id_node << endl;
    cout << "Position time step " << this->time_step << endl;
    cout << "Position task " << this->assigned_task << endl;
}