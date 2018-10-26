//
// Created by Florian Grenouilleau on 2018-10-25.
//
#include <stdlib.h>
#include "../include/Instance.h"

void Instance::compute_successors_per_node(){

    // For each node
    for (Node * node_1 : this->list_nodes){

        // We check if the node is an obstacle
        if (node_1->get_type() == 1) continue;

        // We add itself to the list of successors
        node_1->get_list_id_nodes_successors().push_back(node_1->get_id());

        for (Node * node_2 : this->list_nodes){

            // We check that we need to check the succession
            if (node_2 <= node_1) continue;

            // We check if the node 2 is an obstacle
            if (node_2->get_type() == 1) continue;

            // We check if an edge exists between the nodes
            if ( (node_1->get_coord_y() == node_2->get_coord_y()
                && abs(node_1->get_coord_x()-node_2->get_coord_x()) == 1) ||
                    (node_1->get_coord_x() == node_2->get_coord_x()
                     && abs(node_1->get_coord_y()-node_2->get_coord_y()) == 1)){

                // We update the nodes successor list
                node_1->get_list_id_nodes_successors().push_back(node_2->get_id());
                node_2->get_list_id_nodes_successors().push_back(node_1->get_id());
            }
        }
    }
}