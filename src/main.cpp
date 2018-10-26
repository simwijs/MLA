#include <iostream>
#include <stdlib.h>
#include "../include/Data_Reader.h"
#include "../include/Resolution_Method.h"
using namespace std;

int main(int argc, char** argv)
{

    // We set the random seed
    srand(123456);

    // We initialize the instance
    Instance * instance = new Instance("instances/kiva-10-500-5.map", "instances/kiva-1-small.task");

    // We initialize the solution
    Solution * solution = new Solution(instance);

    // We create the data reader
    Data_Reader * data_reader = new Data_Reader();

    // We read the instance and initialize the solution
    data_reader->read_instance(instance,solution);

    // We create the solver
    Resolution_Method * resolution_method = new Resolution_Method();

    // We solve the instance
    resolution_method->solve_instance(instance,solution,1);

    // We delete the objects
    delete resolution_method;
    delete data_reader;
    delete solution;
    delete instance;

    // We end the program
	cout << "End of the program" << endl;
}

// TODO Create small instance + test code

// TODO Revoir si il faut pas changer les types des nodes pour savoir quels sont les endpoints pour la fin de la solution
// TODO Travailler sur la relocation des agents lorsqu'ils attendent