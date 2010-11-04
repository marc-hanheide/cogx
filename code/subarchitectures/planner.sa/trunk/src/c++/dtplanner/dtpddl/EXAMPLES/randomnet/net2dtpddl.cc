

#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <map>
#include <vector>
#include <set>
#include <string>

using namespace std;


#define UNRECOVERABLE_ERROR(X) {cerr<<X<<std::endl;                   \
                                                                      \
        assert(0);exit(0);                                            \
    }                                                                 \
      

enum Side {side1, side2};
enum Device_Type {circuit_breaker, remote_switch};

typedef uint Line_Id;
typedef pair<uint, Device_Type> Device_Id;

map<Line_Id, set<Device_Id> > line__to__devices;
map<Device_Id, map<Side, Line_Id> > device__to__side__to__line;
map<Device_Id, map<Line_Id, Side> > device__to__line__to__side;

set<Line_Id> faulty_lines;
set<Device_Id> closed_devices;

set<Device_Id> devices;
set<Line_Id> lines;

uint number_of_circuit_breakers;
uint number_of_lines;
uint number_of_switches;


set<Line_Id> INITIALLY__POWERED__lines;
map<Line_Id, set<Device_Id>  > INITIALLY__POWERED_BY__lines__circuit_breakers;
map<Device_Id, set<Side> > INITIALLY__SOURCE__remote_switch__side;



#include "preprocessing.hh"

#include "prefix.hh"

#include "parser.hh"



#include "types.hh"
#include "constants.hh"
#include "functions.hh"
#include "predicates.hh"

#include "actions.hh"

#include "objects.hh"
#include "initial_state.hh"


int main(int argc, char** argv)
{
    assert(argc >= 2);
    string input_filename = argv[1];
    
    parse(input_filename);
    preprocess();
    
    ofstream problem_file;
    ofstream domain_file;

    string problem_file_name = DTPDDL::problem_name() + ".dtpddl";
    string domain_file_name = DTPDDL::domain_name() + ".dtpddl";
    
    problem_file.open (problem_file_name.c_str(), ifstream::out);
    domain_file.open (domain_file_name.c_str(), ifstream::out);
    
    domain_file<<DTPDDL::domain_prefix();
    
    domain_file<<DTPDDL::types();
    domain_file<<DTPDDL::constants();
    domain_file<<DTPDDL::predicates();
    domain_file<<DTPDDL::functions();
    
    domain_file<<DTPDDL::close_actions();
    domain_file<<DTPDDL::open_actions();
    domain_file<<DTPDDL::domain_postfix();
    
    
    problem_file<<DTPDDL::problem_prefix();
    problem_file<<DTPDDL::objects();
    problem_file<<DTPDDL::initial_state();
    problem_file<<DTPDDL::problem_postfix();
    
    problem_file.close();
    domain_file.close();
    return 0;
}

