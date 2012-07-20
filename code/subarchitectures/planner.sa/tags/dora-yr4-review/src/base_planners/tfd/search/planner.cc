#include "best_first_search.h"
#include "cyclic_cg_heuristic.h"
#include "no_heuristic.h"

#include "globals.h"
#include "operator.h"
#include "partial_order_lifter.h"
#include "monitoring.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include <cstdio>
#include <math.h>

using namespace std;

#include <sys/times.h>

#define ANYTIME_SEARCH 1

pair<double, double> save_plan(const vector<PlanStep> &plan, const PlanTrace &path, pair<double,double> best_makespan, int &plan_number, string &plan_name);
void readPlanFromFile(const string& filename, vector<string>& plan);
bool validatePlan(vector<string>& plan);

int main(int argc, const char **argv) {
    ifstream file("../preprocess/output");
    if(strcmp(argv[argc-1], "-eclipserun")==0) {
	cin.rdbuf(file.rdbuf());
	cerr.rdbuf(cout.rdbuf());
	argc--;
    }
    else {
        cout.rdbuf(cerr.rdbuf());
    }

    struct tms start, search_start, search_end;
    times(&start);
    bool poly_time_method = false;

    g_greedy = false;
    bool cyclic_cg_heuristic = false, cyclic_cg_preferred_operators = false;
    bool no_heuristic = false;
    string plan_name = "sas_plan";
    //    bool ff_heuristic = false, ff_preferred_operators = false;
    //    bool goal_count_heuristic = false;
    //    bool blind_search_heuristic = false;
    if(argc == 1) {
	cerr << "missing argument: output file name" << endl;
	return 1;
    } else {
	plan_name = string(argv[argc - 1]);
	argc--;
    }

    bool monitor = false;
    for(int i = 1; i < argc; i++) {
	for(const char *c = argv[i]; *c != 0; c++) {
	    if(*c == 't') {
		g_timeout = atoi(string(argv[++i]).c_str());
	    }
	    else if(*c == 'h') {
		g_hard_timeout = atoi(string(argv[++i]).c_str());
	    } else if(*c == 'm') {
	        g_planMonitorFileName = string(argv[++i]);
	        monitor = true;
	    } else if(*c == 'g') {
	        g_greedy = true;
	    } else if(*c == 'y') {
		cyclic_cg_heuristic = true;
	    } else if(*c == 'Y') {
		cyclic_cg_preferred_operators = true;
	    } else if(*c == 'n') {
		no_heuristic = true;
	    } else if(*c == 's') {
		g_force_sequential = true;
	    } else {
		if(i == argc-1) {
		    plan_name = string(argv[i]);
		} else {
		    cerr << "Unknown option: " << *c << endl;
		    return 1;
		}
	    }
	}
    }

    if(g_timeout > 0) {
	cout << "Timeout set to " << g_timeout << " msec." << endl;
    }
    if(g_hard_timeout > 0) {
	cout << "Hard timeout set to " << g_hard_timeout << " msec." << endl;
    }

    if(!cyclic_cg_heuristic && !no_heuristic) {
	cerr << "Error: you must select at least one heuristic!" << endl
		<< "If you are unsure, choose options \"yY\"." << endl;
	return 2;
    }

    cin >> poly_time_method;
    if(poly_time_method) {
	cout << "Poly-time method not implemented in this branch." << endl;
	cout << "Starting normal solver." << endl;
	// cout << "Aborting." << endl;
	// return 1;
    }

    read_everything(cin);
    g_let_time_pass = new Operator(false);
    g_wait_operator = new Operator(true);

//    g_consistency_cache = new ConsistencyCache();

//    dump_DTGs();
//    dump_everything();
//
//    RelaxedState rs = RelaxedState(*g_initial_state);
//    buildTestState(*g_initial_state);
//    cout << "test state:" << endl;
//    g_initial_state->dump();
//    rs = RelaxedState(*g_initial_state);

    if(monitor) {
        vector<string> plan;
        readPlanFromFile(g_planMonitorFileName, plan);
        bool ret = validatePlan(plan);
        cout << "Plan is valid: " << ret << endl;
        exit(0);
    }

    BestFirstSearchEngine *engine = new BestFirstSearchEngine;
    if(cyclic_cg_heuristic || cyclic_cg_preferred_operators)
	engine->add_heuristic(new CyclicCGHeuristic, cyclic_cg_heuristic,
		cyclic_cg_preferred_operators);
    if(no_heuristic)
	engine->add_heuristic(new NoHeuristic, no_heuristic, false);


pair<double,double> best_makespan = make_pair(REALLYBIG,REALLYBIG); // first: original makespan, second: recheduled makespan

    int plan_number = 1;
    while(true) {
	times(&search_start);
	engine->initialize();
	engine->search();
	times(&search_end);
	if(engine->found_solution()) {
	    best_makespan = save_plan(engine->get_plan(),engine->get_path(),best_makespan,plan_number,plan_name);
#ifdef ANYTIME_SEARCH
	    engine->fetch_next_state();
#else
	    break;
#endif
	}
	else {
	    break;
	}
    }


    int search_ms = (search_end.tms_utime - search_start.tms_utime) * 10;
    cout << "Search time: " << search_ms / 1000.0 << " seconds" << endl;
    int total_ms = (search_end.tms_utime - start.tms_utime) * 10;
    cout << "Total time: " << total_ms / 1000.0 << " seconds" << endl;

    return engine->found_at_least_one_solution() ? 0 : 1;
}

void readPlanFromFile(const string& filename, vector<string>& plan) {
    ifstream fin(filename.c_str(), ifstream::in);
    string buffer;
    while(fin.good()) {
        getline(fin,buffer,'\n');
        plan.push_back(buffer);
    }
    fin.close();
}

bool validatePlan(vector<string>& plan) {
    cout << "Validate Plan!" << endl;
    MonitorEngine* mon = MonitorEngine::getInstance();
    bool monitor = mon->validatePlan(plan);
    return monitor;
}

pair<double,double> save_plan(const vector<PlanStep> &plan, const PlanTrace &path, pair<double,double> best_makespan, int &plan_number, string &plan_name) {

//    for(int i = 0; i < path.size(); ++i) {
//        path[i]->dump();
//    }

    PartialOrderLifter partialOrderLifter(plan, path);

//    Plan new_plan = partialOrderLifter.lift();

    Plan new_plan = plan;

    //free PlanTrace!
    for(int i = 0; i < path.size(); ++i) {
        delete(path[i]);
    }

    double makespan = 0;
    for(int i = 0; i < new_plan.size(); i++) {
	double end_time = new_plan[i].start_time + new_plan[i].duration;
	makespan = max(makespan,end_time);
    }

    double original_makespan = 0;
    for(int i = 0; i < plan.size(); i++) {
        double end_time = plan[i].start_time + new_plan[i].duration;
        original_makespan = max(original_makespan,end_time);
    }

    if(makespan >= best_makespan.second) return best_makespan;

    cout << "Plan:" << endl;
    for(int i = 0; i < plan.size(); i++) {
        const PlanStep& step = plan[i];
        fprintf(stderr, "%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
    }

    cout << "new Plan:" << endl;
    for(int i = 0; i < new_plan.size(); i++) {
        const PlanStep& step = new_plan[i];
        fprintf(stderr, "%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
    }
    cout << "Solution with original makespan " << original_makespan << " found (ignoring no-moving-targets-rule)." << endl;
    cout << "Solution was epsilonized and rescheduled to a makespan of " << makespan << "." << endl;


    FILE *file = 0;
    FILE *best_file = 0;

    if (plan_name != "-") {
        int len = static_cast<int>(plan_name.size() + log10(plan_number) + 10);
//    char *temp_filename = (char*)malloc(len * sizeof(char));
//    sprintf(temp_filename, "%sTEMP.%d", plan_name.c_str(), plan_number);
        char *best_filename = (char*)malloc(30);
        sprintf(best_filename, "plan.best");
        char *filename = (char*)malloc(len * sizeof(char));
        sprintf(filename, "%s.%d", plan_name.c_str(), plan_number);
//    len = 2*len + 100;
//    char *syscall = (char*)malloc(len * sizeof(char));
//    sprintf(syscall, "./search/epsilonize_plan.py < %s > %s", temp_filename, filename);
//sprintf(syscall, "/home/eyerich/TemporalFastDownward/eclipseWorkspace/search/epsilonize_plan.py < %s > %s", temp_filename, filename);
    //    sprintf(syscall, "./epsilonize_plan.py < %s > %s", temp_filename, filename);
//    char *syscall2 = (char*)malloc(len * sizeof(char));
//    sprintf(syscall2, "rm %s", temp_filename);

//    printf("FILENAME: %s\n", temp_filename);
        file = fopen(filename,"w");
        best_file = fopen(best_filename,"w");
    }
    else {
        file = stdout;
    }


    plan_number++;
    for(int i = 0; i < new_plan.size(); i++) {
	const PlanStep& step = new_plan[i];
        fprintf(file,"%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
        if (best_file) {
            fprintf(best_file,"%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
        }
//	printf("%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
    }

    if (plan_name != "-") {
        fclose(file);
        fclose(best_file);
    }

    cout << "Plan length: " << new_plan.size() << " step(s)." << endl;
    cout << "Makespan   : " << makespan << endl;

//    int ret_of_syscall_1 = system(syscall);
//    int ret_of_syscall_2 = system(syscall2);

//    if(ret_of_syscall_1 || ret_of_syscall_2) {
//	cout << "Error while calling epsilonize plan!" << endl;
//    }

//    free(temp_filename);
//    free(filename);
//    free(syscall);
//    free(syscall2);

//    return makespan;
    return make_pair(original_makespan,makespan);

}
