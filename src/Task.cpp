//
// Created by Florian Grenouilleau on 2018-10-26.
//

#include "../include/Task.h"
#include <iostream>

using namespace std;


void Task::write(){
    cout << "Task id " << this->id << endl;
    cout << "Task release time " << this->release_date << endl;
    cout << "Task pickup node " << this->pickup_node << endl;
    cout << "Task delivery node " << this->delivery_node << endl;
    cout << "Task pickup time " << this->picked_date << endl;
    cout << "Task delivery time " << this->delivered_date << endl;
}