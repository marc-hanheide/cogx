#include <vector>
#include <iostream>

#include "dlib/bayes_utils.h"
#include "dlib/graph_utils.h"
#include "dlib/graph.h"
#include "dlib/directed_graph.h"

using namespace dlib;
using namespace std;
using namespace bayes_node_utils;

typedef directed_graph<bayes_node>::kernel_1a_c bnet;
typedef set<unsigned long>::compare_1b_c set_type;
typedef graph<set_type, set_type>::kernel_1a_c join_tree_type;

void build_bnet(istream &in, bnet &bn);
void add_evidence(istream &in, bnet &bn);
void clear_evidence(bnet &bn);
join_tree_type* evaluate(bnet &bn, join_tree_type *join_tree);

int main() {
    istream &in = cin;
    bnet* bn = 0;
    join_tree_type *join_tree = 0;

    string tag;
    while (!in.eof() && !in.fail()) {
        in >> tag;
        cerr << "tag:" << tag << endl;
        if (tag == "structure") {
            delete bn;
            delete join_tree;
            bn = new bnet();
            build_bnet(in, *bn);
        }
        else if (tag == "update_structure") {
            delete join_tree;
            build_bnet(in, *bn);
        }
        else if (tag == "evidence") {
            add_evidence(in, *bn);
        }
        else if (tag == "clear_evidence") {
            clear_evidence(*bn);
        }
        else if (tag == "evaluate") {
            evaluate(*bn, join_tree);
        }
        tag = "";
    }
    // if (in.fail()) {
    //     char buf[100];
    //     in.getline(buf, 100);
    //     cerr << "error when reading input: " << buf << endl;
    // }
    delete bn;
    delete join_tree;
}

void build_bnet(istream &in, bnet &bn) {
    // Read in nodes and their arities
    unsigned long num_nodes;
    in >> num_nodes;
    unsigned long prev_num_nodes = bn.number_of_nodes();
    num_nodes += prev_num_nodes;
    
    bn.set_number_of_nodes(num_nodes);
    for (int i=prev_num_nodes; i < num_nodes; i++) {
        int num_val;
        in >> num_val;
        set_node_num_values(bn, i, num_val);
    }

    // Read in edges
    int num_edges;
    in >> num_edges;
    cerr << "edges: "<< num_edges << endl;
    for (int i=0; i < num_edges; i++) {
        int from, to;
        in >> from;
        in >> to;
        bn.add_edge(from, to);
    }


    // Read in conditional probabilities
    for (int i=prev_num_nodes; i<num_nodes; i++) {
        string tag;
        int id;
        in >> tag;
        in >> id;
        assert(id == i);
        unsigned long num = node_num_values(bn, i);
        cerr << "node: "<< i << " ("<< num<< " values)" << endl;
        assignment parent_state;

        int parent_count;
        in >> parent_count;
        int row_count = 1;
        std::vector<int> parent_order;
        for (int j=0; j < parent_count; j++) {
            int p;
            in >> p;
            parent_order.push_back(p);
            parent_state.add(p, 0);
            row_count *= node_num_values(bn, p);
        }
        cerr << "parents: " << parent_count << ", rows: "<< row_count << endl;
        for (int j=0; j < row_count; j++) {
            for (int k=0; k < parent_count; k++) {
                int pval;
                in >> pval;
                parent_state[parent_order[k]] = pval;
            }
            for (int k=0; k < num; k++) {
                double p;
                in >> p;
                cerr << p << endl;
                set_node_probability(bn, i, k, parent_state, p);
            }
        }
    }
    cerr << "done" << endl;
}

void add_evidence(istream &in, bnet &bn) {
    int num_evidence;
    in >> num_evidence;
    for (int i=0; i < num_evidence; i++) {
        int node, value;
        in >> node;
        in >> value;
        set_node_value(bn, node, value);
        set_node_as_evidence(bn, node);        
    }
}

void clear_evidence(bnet &bn) {
    int num_nodes = bn.number_of_nodes();
    for (int i=0; i < num_nodes; i++) {
        set_node_as_nonevidence(bn, i);        
    }
}

join_tree_type* evaluate(bnet &bn, join_tree_type *join_tree) {
    if (!join_tree) {
        join_tree = new join_tree_type();
        create_moral_graph(bn, *join_tree);
        create_join_tree(*join_tree, *join_tree);
    }

 
    bayesian_network_join_tree solution(bn, *join_tree);
    for (int i=0; i < bn.number_of_nodes(); i++) {
        for (int j=0; j < node_num_values(bn, i); j++) {
            cerr << "p(" << i << "=" << j << ") = " << solution.probability(i)(j) << endl;
            cout << solution.probability(i)(j) << " ";
        }
        cout << endl;
    }

    return join_tree;
}

