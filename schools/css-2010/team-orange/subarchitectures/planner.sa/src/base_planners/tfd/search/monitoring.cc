#include "monitoring.h"
#include "state.h"
#include "operator.h"
#include "globals.h"
#include "axioms.h"

#include <iostream>
#include <cassert>

using namespace std;

MonitorEngine* MonitorEngine::instance = 0;

MonitorEngine* MonitorEngine::getInstance()
{
  if(instance == 0)
    instance = new MonitorEngine();

  return instance;
}

MonitorEngine::MonitorEngine()
{
}

MonitorEngine::~MonitorEngine()
{
}

int lower_case ( int c )
{
  return tolower ( c );
}


bool MonitorEngine::validatePlan(vector<string>& plan)
{
  vector<PlanStep> p;
  for(unsigned int i = 0; i < plan.size(); i++)
  {
    double start = atof(plan[i].substr(0,plan[i].find(":")).c_str());
    string name = plan[i].substr(plan[i].find("(") + 1,plan[i].find(")") - plan[i].find("(") - 1);
    transform(name.begin(),name.end(),name.begin(),lower_case);
    double duration = atof(plan[i].substr(plan[i].find("[") + 1, plan[i].length() - plan[i].find("[")).c_str());
    for(unsigned int i = 0; i < g_operators.size(); i++)
    {
      if(g_operators[i].get_name() == name)
      {
	p.push_back(PlanStep(start,duration,&g_operators[i]));
	break;
      }
    }
    cerr << start << "  " << name << "  " << duration << endl;
  }
  return validatePlan(p);
}

bool MonitorEngine::validatePlan(const vector<PlanStep>& plan)
{
  TimeStampedState current = *g_initial_state;

  for(int i = 0; i < plan.size(); i++) {
      while(plan[i].start_time > current.timestamp) {
          double curren_time = current.timestamp;
          if(plan[i].start_time -  2*EPS_TIME - EPSILON <= current.timestamp) {
              current.timestamp += EPS_TIME;
          } else {
              current = current.let_time_pass();
          }
          if(double_equals(current.timestamp,curren_time)) {
              current.timestamp += EPS_TIME;
          }
      }
      cout << "Current time_stamp: " << current.timestamp << endl;
      cout << "Next op: " << plan[i].op->get_name() << " ";
      if(!plan[i].op->is_applicable(current)) {
          cout << "is not applicable!" << endl;
          return false;
      } else {
          cout << "is applicable!" << endl;
          current = TimeStampedState(current,(*plan[i].op));
      }
  }

  while(!current.operators.empty())
    current = current.let_time_pass();

  return current.satisfies(g_goal);
}
