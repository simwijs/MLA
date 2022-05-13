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
    cout << "Task Assigned Agent " << this->id_assigned_agent << endl;
}

int Task::get_service_time() {
    if (!finished) throw std::runtime_error("Cannot get service time when the task isn't finished");

    return delivered_date - release_date;
}

void Batch::add_task(Task *t) {
    if (release_date == -1) {
        release_date = t->get_release_date();
    }
    tasks.push_back(t);
}

int Batch::get_service_time() {
    if (!is_finished()) throw std::runtime_error("Cannot get service time when the batch isn't finished");

    if (delivered_date == -1 || release_date == -1) {
      throw std::runtime_error(
          "The appear or finished timestep is not set. Cannot get service "
          "time");
    }
    return delivered_date - release_date;
}

bool Batch::is_finished() {

    for (auto t: tasks) {
        if (t->get_delivered_date() == -1) {
            return false;
        }
    }
    return true;
}

void Batch::try_finish() {
    if (!is_finished()) return;

    for (auto t : tasks) {
        if (t->get_delivered_date() > get_delivered_date()) {
            delivered_date = t->get_delivered_date();
        }
    }

}