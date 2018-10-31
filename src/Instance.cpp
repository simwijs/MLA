//
// Created by Florian Grenouilleau on 2018-10-25.
//
#include "../include/Instance.h"
#include <queue>
#include <iostream>

void Instance::compute_h_values(vector<int> & h_values, int start_location) {

    // We initialize the values
    queue<int> Q;
    vector<bool> status(this->list_map_nodes.size(), false);
    h_values.resize(this->list_map_nodes.size(),-1);
    int neighbor[4] = {1,-1,this->nb_column,-(this->nb_column)};

    // We update the valeus for the start location
    status[start_location] = true;
    h_values[start_location] = 0;

    // We add the start location to the queue
    Q.push(start_location);

    while (!Q.empty())
    {
        // We get the first value
        int v = Q.front();
        Q.pop();

        // For each neighbor
        for (int i = 0; i < 4; i++)
        {
            // We check that the neighbor is reachable
            if (i == 0 && v % (this->nb_column) == this->nb_column-1) continue;
            if (i == 1 && v % (this->nb_column) == 0) continue;
            if (i == 2 && v / (this->nb_column) == this->nb_row-1) continue;
            if (i == 3 && v / (this->nb_column) == 0) continue;

            // We get the corresponding value
            int u = v + neighbor[i];

            // We check if this neighbor is accessible
            if (this->list_map_nodes[u])
            {

                // We check if the new value has been checked
                if (!status[u])
                {
                    // We update the values and add the successor to the queue
                    status[u] = true;
                    h_values[u] = h_values[v] + 1;
                    Q.push(u);
                }
            }
        }
    }
}