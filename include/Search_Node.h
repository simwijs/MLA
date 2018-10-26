//
// Created by Florian Grenouilleau on 2018-10-25.
//

#ifndef MAPD_SEARCH_NODE_H
#define MAPD_SEARCH_NODE_H

class Search_Node {

public:

    // Attributes
    int id_node_graph, time_step;
    Search_Node * parent_node = nullptr;

    // Constructor
    Search_Node(int id_node_graph_, int time_step_) : id_node_graph(id_node_graph_), time_step(time_step_) {};
    Search_Node(int id_node_graph_, int time_step_, Search_Node * parent_node_) :
            id_node_graph(id_node_graph_), time_step(time_step_), parent_node(parent_node_) {};

};

#endif //MAPD_SEARCH_NODE_H
