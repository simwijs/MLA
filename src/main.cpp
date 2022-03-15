#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <string>
#include <ctime>
#include <string>
#include "../include/Data_Reader.h"
#include "../include/Resolution_Method.h"

using namespace std;

int main(int argc, char** argv) {

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
    
    std::string output_file = argc > 5 ? argv[5] : "Instances_Summary.txt";

    // We check that the found solution is feasible
    if (instance->check_solution_feasible()){

        cout << "Final solution feasible" << endl;
        cout << "Makespan : " << instance->get_current_time_step() << endl;

        // We create the output of the solution
        instance->output_solution(argv, output_file);

        // We write the h value between consecutive tasks per agent
        //instance->show_h_value_between_tasks_per_agent();

        // We write the files for the visualization
        instance->output_map_for_visualization();

        // Get the filenames for output
        char* mapptr = new char[instance->get_map_file_name().size() + 1];
        strcpy(mapptr, instance->get_map_file_name().c_str());
        char* ptr = strtok(mapptr, "/");
        std::string latestMap = "";
        std::string latestTask = "";
        while (ptr != NULL) {
            latestMap = std::string(ptr);
            ptr = strtok(NULL, "/");
        }

        char* taskptr = new char[instance->get_task_file_name().size() + 1];
        strcpy(taskptr, instance->get_task_file_name().c_str());
        ptr = strtok(taskptr, "/");
        while (ptr != NULL) {
            latestTask = std::string(ptr);
            ptr = strtok(NULL, "/");
        }


        instance->output_moves_for_visualization("./output/output-" + latestMap + "--" + latestTask + ".yaml");
        instance->output_tasks_for_visualization("./output/task_output-" + latestMap + "--" + latestTask + ".yaml");

        // We create all the instances (NEED TO USE A 50 AGENTS INSTANCE)

        // For each number of agents
        /*for (int nb_a = 5; nb_a <= 50; nb_a+=5){

            // For each number of task
            for (int nb_t = 100; nb_t <= 500; nb_t += 100){

                // For each frequency
                //instance->create_instances_first_set(nb_a,nb_t,0.2);
                instance->create_instances_second_set(nb_a,nb_t,0.2);
                instance->create_instances_third_set(nb_a,nb_t,0.2);

                //instance->create_instances_first_set(nb_a,nb_t,0.5);
                instance->create_instances_second_set(nb_a,nb_t,0.5);
                instance->create_instances_third_set(nb_a,nb_t,0.5);

                //instance->create_instances_first_set(nb_a,nb_t,1);
                instance->create_instances_second_set(nb_a,nb_t,1);
                instance->create_instances_third_set(nb_a,nb_t,1);

                //instance->create_instances_first_set(nb_a,nb_t,2);
                instance->create_instances_second_set(nb_a,nb_t,2);
                instance->create_instances_third_set(nb_a,nb_t,2);

                //instance->create_instances_first_set(nb_a,nb_t,5);
                instance->create_instances_second_set(nb_a,nb_t,5);
                instance->create_instances_third_set(nb_a,nb_t,5);

                //instance->create_instances_first_set(nb_a,nb_t,10);
                instance->create_instances_second_set(nb_a,nb_t,10);
                instance->create_instances_third_set(nb_a,nb_t,10);

            }
        }*/
    }
    else {

        cout << "Final solution not feasible" << endl;
    }

    // We delete the objects
    delete resolution_method;
    delete data_reader;
    delete instance;

    // We end the program
    //cout << " - - - - End of the program - - - - " << endl;
    //cout << endl;
}