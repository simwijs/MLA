#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <ctime>
#include "../include/Data_Reader.h"
#include "../include/Resolution_Method.h"
using namespace std;

int main(int argc, char** argv)
{
    // We set the random seed
    srand(1243);

    // We initialize the instance
    Instance * instance = new Instance("instances/kiva-10-500-5.map","instances/kiva-1.task");

    // We initialize the solution
    Solution * solution = new Solution(instance);

    // We create the data reader
    Data_Reader * data_reader = new Data_Reader();

    // We read the instance and initialize the solution
    data_reader->read_instance(instance,solution);

    // We create the solver
    Resolution_Method * resolution_method = new Resolution_Method();

    // We initialize the timer values
    std::clock_t start = std::clock();
    double computation_time;

    // We solve the instance
    resolution_method->solve_instance(instance,solution,1);

    // We get the computation time
    computation_time = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);

    // We update the solution computation time
    solution->set_computation_time(computation_time);

    // We check if the solution is feasible
    if (solution->check_solution_feasible()){
        cout << "Final solution feasible" << endl;

        //solution->write();

        // We create the output of the solution
        solution->output_solution();
    }
    else {
        cout << "Final solution not feasible" << endl;
    }

    // We delete the objects
    delete resolution_method;
    delete data_reader;
    delete solution;
    delete instance;

    // We end the program
	cout << "End of the program" << endl;
}

// TODO Optimiser le code lors de la recherche de chemin
// TODO Revoir si il faut pas changer les types des nodes pour savoir quels sont les endpoints pour la fin de la solution
// TODO Travailler sur la relocation des agents lorsqu'ils attendent