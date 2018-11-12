#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <string>
#include <ctime>
#include <string>
#include "../include/Data_Reader.h"
#include "../include/Resolution_Method.h"

using namespace std;

int main(int argc, char** argv)
{

    // We generate the random seed
    srand(123);

    // We initialize the instance
    Instance * instance = new Instance(argv[1],argv[2]);

    // We create the data reader
    Data_Reader * data_reader = new Data_Reader();

    // We read the instance
    data_reader->read_instance(instance);

    // We create the solver
    Resolution_Method * resolution_method = new Resolution_Method();

    // We set the instance's wait step
    instance->set_wait_value(stoi(argv[4]));

    // We check if the instance has to generate agents
    if (stoi(argv[3]) == 7){
        instance->generate_agents(stoi(argv[5]));
    }

    // We initialize the timer values
    std::clock_t start = std::clock();
    double computation_time;

    // We solve the instance
    resolution_method->solve_instance(instance,stoi(argv[3]));

    // We get the computation time
    computation_time = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);

    // We update the instance's computation time
    instance->set_computation_time(computation_time);

    // We update the instance's makespan
    instance->compute_final_makespan();

    // We check that the found solution is feasible
    if (instance->check_solution_feasible()){

        //cout << "Final solution feasible" << endl;
        cout << "Makespan : " << instance->get_current_time_step() << endl;

        // We create the output of the solution
        instance->output_solution(argv);

    }
    else {
        cout << "Final solution not feasible" << endl;
    }

    // We delete the objects
    delete resolution_method;
    delete data_reader;
    delete instance;

    // We end the program
	cout << " - - - - End of the program - - - - " << endl;
    cout << endl;
}