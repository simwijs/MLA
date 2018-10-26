//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_TASK_H
#define MAPD_TASK_H


class Task {

private:

    // Attributes
    int id, release_date, pickup_node, delivery_node, picked_date, delivered_date;

public:

    // Constructor
    Task(int id_,int release_date_,int pickup_node_,int delivery_node_) : id(id_),release_date(release_date_),
                                                                          pickup_node(pickup_node_),
                                                                          delivery_node(delivery_node_) {};

    // General Methods
    void write();

    // Setters
    void set_pickup_node(int value){this->pickup_node = value;}
    void set_delivery_node(int value){this->delivery_node = value;}
    void set_picked_date(int value){this->picked_date = value;}
    void set_delivered_date(int value){this->delivered_date = value;}

    // Getters
    int get_id(){ return this->id;}
    int get_release_date(){ return this->release_date;}
    int get_pickup_node(){ return this->pickup_node;}
    int get_delivery_node(){ return this->delivery_node;}
    int get_picked_date(){ return this->picked_date;}
    int get_delivered_date(){ return this->delivered_date;}
};

#endif //MAPD_TASK_H
