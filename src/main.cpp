#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <ctime>
#include <string>
#include "../include/Data_Reader.h"

using namespace std;

int main(int argc, char** argv)
{
    // We set the random seed
    srand(124);

    // We initialize the instance
    Instance * instance = new Instance("instances/kiva-10-500-5.map",
                                       "instances/kiva-1.task");

    // We create the data reader
    Data_Reader * data_reader = new Data_Reader();

    // We read the instance
    data_reader->read_instance(instance);

    // We delete the objects
    delete data_reader;
    delete instance;

    // We end the program
	cout << "End of the program" << endl;
}