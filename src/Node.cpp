//
// Created by Florian Grenouilleau on 2018-10-25.
//

#include <iostream>
#include "../include/Node.h"

void Node::write() {

    cout << "Node id : " << this->id << endl;
    cout << "Node type : " << this->type << endl;
    cout << "Node x coord : " << this->coord_x << endl;
    cout << "Node y coord : " << this->coord_y << endl;
}