

#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <map>
#include <vector>
#include <set>
#include <string>
#include <algorithm>

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
bool generate_network;
bool soft_goal;
double fault_probability;/*Probability that a line is faulty.*/


set<Line_Id> INITIALLY__POWERED__lines;
map<Line_Id, set<Device_Id>  > INITIALLY__POWERED_BY__lines__circuit_breakers;
map<Device_Id, set<Side> > INITIALLY__SOURCE__remote_switch__side;


#include "Command_Line_Arguments.hh"

Command_Line_Arguments command_Line_Arguments;

#include "preprocessing.hh"

#include "types.hh"
#include "predicates.hh"
#include "observations.hh"


#include "prefix.hh"
#include "parser.hh"



#include "constants.hh"
#include "functions.hh"

#include "actions.hh"

#include "objects.hh"
#include "initial_state.hh"


int main(int argc, char** argv)
{
    command_Line_Arguments = Command_Line_Arguments(argc, argv);

    generate_network = false;
    if(command_Line_Arguments.is_argument("generate_network")){
        generate_network = true;
    }
    
    soft_goal = false;
    if(command_Line_Arguments.is_argument("soft_goal")){
        soft_goal = true;
    }
    
    string input_filename;
    if(command_Line_Arguments.got_guard("--network")){
        input_filename = command_Line_Arguments.get_string();
    } else if (!generate_network){
        UNRECOVERABLE_ERROR("No network specified with:"<<std::endl
                            <<"--network filename.something"<<std::endl
                            <<"Otherwise, you can ask us to generate the\n"
                            <<"network, by including \"generate_network\" as an argument.");
    }


    number_of_circuit_breakers = 2;
    if(command_Line_Arguments.got_guard("--circuit_breakers")){
        number_of_circuit_breakers = command_Line_Arguments.get_int();
    } else if (generate_network) {
        cerr<<"Warning :: --circuit_breakers NOT SPECIFIED."<<std::endl
            <<"Default is :: "<<number_of_circuit_breakers<<std::endl;
    }
    
    number_of_switches = 3;
    if(command_Line_Arguments.got_guard("--remote_switches")){
        number_of_switches = command_Line_Arguments.get_int();
    }  else if (generate_network) {
        cerr<<"Warning :: --remote_switches NOT SPECIFIED."<<std::endl
            <<"Default is :: "<<number_of_switches<<std::endl;
    }
    
    fault_probability = 0.0;
    if(command_Line_Arguments.got_guard("--faulty-lines")){
        fault_probability = command_Line_Arguments.get_double();
    }  else if (generate_network) {
        std::cerr<<"Warning :: --faulty-lines NOT SPECIFIED."<<std::endl
            <<"Default is :: "<<fault_probability<<std::endl
            <<"0.0 means no line is faulty, and otherwise you are"<<std::endl
            <<"specifying the probability that a line is faulty.";
    }

    if(generate_network){
        ostringstream command;
        input_filename = "./Slaney/output.net";
        command<<"cd Slaney; make; ./randomnet -o "
               <<" -n "<<number_of_circuit_breakers
               <<" -u "<<number_of_switches
               <<" -f "<<fault_probability<<" > output.net ; cd .."<<std::endl;
        
        system(command.str().c_str());
    }
    
    
//     assert(argc >= 2);
//     string input_filename = argv[1];
    
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
    domain_file<<DTPDDL::percepts();
    domain_file<<DTPDDL::functions();
    
    domain_file<<"(:action spin :effect (and "<<DTPDDL::rewards()<<" ) ) "<<std::endl;
        
    domain_file<<DTPDDL::close_actions();
    domain_file<<DTPDDL::open_actions();
    domain_file<<DTPDDL::observation();
    domain_file<<DTPDDL::domain_postfix();
    
    
    problem_file<<DTPDDL::problem_prefix();
    problem_file<<DTPDDL::objects();
    problem_file<<DTPDDL::initial_state();
    problem_file<<DTPDDL::problem_postfix();
    
    problem_file.close();
    domain_file.close();
    return 0;
}

