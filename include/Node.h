
#ifndef MAPD_NODE_H
#define MAPD_NODE_H

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

class Node{
public:

    // Attributes
    int loc, g_val, h_val, timestep;
    bool in_openlist;
    Node *parent;

    // Constructors
	Node() {};
	Node(int loc, int g_val, Node *parent, int timestep) :loc(loc), g_val(g_val),  h_val(0),  timestep(timestep),
                                                          parent(parent) {};
	Node(int loc, int g_val, int h_val, Node *parent, int timestep, bool in_openlist = false) :loc(loc), g_val(g_val)
            , h_val(h_val), timestep(timestep), in_openlist(in_openlist), parent(parent) {};

    // General Methods
	inline int getFVal() const {return g_val + h_val;}

};

// the following is used to compare nodes in the OPEN list
struct compare_node {
	// returns true if n1 > n2 (note -- this gives us *min*-heap).
	bool operator()(const Node* n1, const Node* n2) const {

		if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
		else
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
	}
};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)


// define typedefs
typedef boost::heap::fibonacci_heap< Node*, boost::heap::compare<compare_node> > heap_open_t;
//typedef dense_hash_map<AStarNode*, AStarNode*, NodeHasher, eqnode> hashtable_t;
// note -- hash_map (key is a node pointer, data is a node handler,
//                   NodeHasher is the hash function to be used,
//                   eqnode is used to break ties when hash values are equal)

#endif //MAPD_NODE_H