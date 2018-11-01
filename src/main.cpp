#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <ctime>
#include <string>
#include "../include/Data_Reader.h"
#include "../include/Resolution_Method.h"

using namespace std;

int main(int argc, char** argv)
{
    // We set the random seed
    srand(124);

    // We initialize the instance
    Instance * instance = new Instance("instances/kiva-30-500-5.map",
                                       "instances/kiva-1.task");

    // We create the data reader
    Data_Reader * data_reader = new Data_Reader();

    // We read the instance
    data_reader->read_instance(instance);

    // We create the solver
    Resolution_Method * resolution_method = new Resolution_Method();

    // We solve the instance
    resolution_method->solve_instance(instance);

    // We update the instance's makespan
    instance->compute_final_makespan();


    // We check that the found solution is feasible
    if (instance->check_solution_feasible()){
        cout << "Final solution feasible" << endl;
        cout << "Final makespan " << instance->get_current_time_step() << endl;
    }
    else {
        cout << "Final solution not feasible" << endl;
    }

    // We delete the objects
    delete resolution_method;
    delete data_reader;
    delete instance;

    // We end the program
	cout << "End of the program" << endl;
}