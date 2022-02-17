//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_TASK_H
#define MAPD_TASK_H
#include <vector>

class Task {

private:

    // Attributes
    int id, release_date, pickup_node, delivery_node, picked_date = -1, delivered_date = -1, id_assigned_agent = -1, batch_id = -1;
    bool finished = false;
public:

    // Constructor
    Task(int id_,int release_date_,int pickup_node_,int delivery_node_, int batch_id) : id(id_),release_date(release_date_),
                                                                          pickup_node(pickup_node_),
                                                                          delivery_node(delivery_node_),
                                                                          id_assigned_agent(-1),
                                                                          batch_id(batch_id) {};

    // General Methods
    void write();

    // Setters
    void set_pickup_node(int value){this->pickup_node = value;}
    void set_delivery_node(int value){this->delivery_node = value;}
    void set_picked_date(int value){this->picked_date = value;}
    // finish the task by setting the delivery date
    void set_delivered_date(int value){this->delivered_date = value; this->finished = true;}
    void set_id_assigned_agent(int value){this->id_assigned_agent = value;}
    void set_batch_id(int value){this->batch_id = value;}

    // Getters
    int get_id(){ return this->id;}
    int get_release_date(){ return this->release_date;}
    int get_pickup_node(){ return this->pickup_node;}
    int get_delivery_node(){ return this->delivery_node;}
    int get_picked_date(){ return this->picked_date;}
    int get_delivered_date(){ return this->delivered_date;}
    int get_id_assigned_agent(){ return this->id_assigned_agent;}
    int get_batch_id(){ return this->batch_id;}
    int get_service_time();
};


class Batch {

private:
    int id, delivered_date, release_date;
    std::vector<Task*> tasks;

public:

    Batch(int id): id(id), release_date(-1), delivered_date(-1) {};

    void set_release_date(int value) {this->release_date = value;};
    void add_task(Task *t);

    int get_batch_id(){ return this->id;};
    int get_delivered_date(){ return this->delivered_date;};
    int get_release_date(){ return this->release_date;};
    int get_size(){ return this->tasks.size();};
    void try_finish();
    bool is_finished();
    int get_service_time();
};
#endif //MAPD_TASK_H
