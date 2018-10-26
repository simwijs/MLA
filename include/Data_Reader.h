//
// Created by Florian Grenouilleau on 2018-10-24.
//

#ifndef MAPD_DATA_READER_H
#define MAPD_DATA_READER_H

#include "Solution.h"

using namespace std;

class Data_Reader{

private:

    void read_task_file(Instance * instance, Solution * solution);
    void read_map_file(Instance * instance, Solution * solution);

public:

    void read_instance(Instance * instance, Solution * solution);
};
#endif //MAPD_DATA_READER_H
