//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_NODE_H
#define MAPD_NODE_H

#include <vector>

using namespace std;

class Node {

private:
    int id, type, coord_x, coord_y; // 0 = free, 1 = obstacle
    vector<int> list_id_nodes_successors;

public:

    // Constructor
    Node(int id_, int type_) : id(id_), type(type_) {};

    Node(int id_, int type_, int coord_x_, int coord_y_) : id(id_), type(type_), coord_x(coord_x_),
                                                           coord_y(coord_y_) {};

    // General methods
    void write();

    // Getters
    int get_id(){ return this->id;}
    int get_type(){ return this->type;}
    int get_coord_x(){ return this->coord_x;}
    int get_coord_y(){ return this->coord_y;}
    vector<int> & get_list_id_nodes_successors(){ return this->list_id_nodes_successors;}

};
#endif //MAPD_NODE_H
