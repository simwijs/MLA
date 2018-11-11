
#ifndef MAPD_COMPLEX_NODE_H
#define MAPD_COMPLEX_NODE_H

#include <math.h>
#include <iostream>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim_all.hpp>
#include <cassert>
#include <fstream>
#include <string>
#include <limits>

using namespace std;

class Complex_Node{
public:

    // Attributes
    int location, g_val, h_val, timestep, type;
    bool in_openlist;
	Complex_Node *parent;

    // Constructors
	Complex_Node() {};
	Complex_Node(int location, int g_val, Complex_Node *parent, int timestep) :location(location), g_val(g_val),
																			   h_val(0), timestep(timestep),
																			   parent(parent) {};

	Complex_Node(int location, int g_val, int h_val, Complex_Node *parent, int timestep, int type,
				 bool in_openlist = false) :
			location(location), g_val(g_val), h_val(h_val), timestep(timestep), type(type), in_openlist(in_openlist),
			parent(parent) {};

    // General Methods
	inline int getFVal() const {return g_val + h_val;}

};

// the following is used to compare nodes in the OPEN list
struct compare_complex_node {

	// returns true if n1 > n2 (note -- this gives us *min*-heap).
	bool operator()(const Complex_Node* n1, const Complex_Node* n2) const {

		if (n1->g_val + n1->h_val == n2->g_val + n2->h_val){
			if (n1->type == n2->type){
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			}
			else {
				return n1->type <= n2->type;
			}
		}
		else
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
	}
};


// define typedefs
typedef boost::heap::fibonacci_heap< Complex_Node*, boost::heap::compare<compare_complex_node> > complex_heap_open_t;

#endif //MAPD_COMPLEX_NODE_H