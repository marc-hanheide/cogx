#include "scheduler.h"

#include <cassert>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

SimpleTemporalProblem::SimpleTemporalProblem(int _number_of_nodes) {
  // take dummy node representing zero time point into account
  // dummy node will always be the last (with index number_of_nodes-1)
  number_of_nodes = _number_of_nodes+1;
  for(size_t i = 0; i < number_of_nodes-1; i++) {
    std::string name;
    std::stringstream sname;
    sname << "var" << i;
    name = sname.str();
    variable_names.push_back(name);
  }
  initialize();
}

SimpleTemporalProblem::SimpleTemporalProblem(std::vector<std::string> _variable_names) {
  variable_names = _variable_names;
  number_of_nodes = variable_names.size()+1;
  initialize();
}

void SimpleTemporalProblem::initialize() {
  variable_names.push_back("X0");
  for(size_t i = 0; i < number_of_nodes; i++) {
    matrix.push_back(MatrixLine(number_of_nodes,INF));
  }
  for(size_t i = 0; i < number_of_nodes; i++) {
    matrix[i][i] = 0.0;
  }
}

// set the maximum distance between from and to.
void SimpleTemporalProblem::setDistance(int from, int to, double distance) {
  assert(0 <= from && from < number_of_nodes);
  assert(0 <= to && to < number_of_nodes);
  assert(from != to);
  matrix[from][to] = distance;
}

// set the interval into which value(to)-value(from) has to fall
void SimpleTemporalProblem::setInterval(int from, int to, double lower, double upper) {
  setDistance(from,to,upper);
  setDistance(to,from,-lower);
}

// set the exact value which value(to)-value(from) must have
void SimpleTemporalProblem::setSingletonInterval(int from, int to, double lowerAndUpper) {
  setInterval(from,to,lowerAndUpper,lowerAndUpper);
}

// set the interval into which value(to)-value(from) has to fall,
// with the assumption that the interval is open to the right.
void SimpleTemporalProblem::setUnboundedInterval(int from, int to, double lower) {
  setInterval(from,to,lower,INF);
}

void SimpleTemporalProblem::setIntervalFromXZero(int to, double lower, double upper) {
  setInterval(number_of_nodes-1,to,lower,upper);
}

void SimpleTemporalProblem::setUnboundedIntervalFromXZero(int to, double lower) {
  setUnboundedInterval(number_of_nodes-1,to,lower);
}

double SimpleTemporalProblem::add(double a, double b) {
  if(a == INF || b == INF || a+b >= INF) return INF;
  return a+b;
}

// solve the STP using the Floyd-Warshall algorithm with running time O(n^3)
void SimpleTemporalProblem::solve() {
  for(size_t k = 0; k < number_of_nodes; k++) {
    for(size_t i = 0; i < number_of_nodes; i++) {
      for(size_t j = 0; j < number_of_nodes; j++) {
	double triangle_length = add(matrix[i][k],matrix[k][j]);
	double original_length = matrix[i][j];
	if(triangle_length < original_length) {
	  // update
	  matrix[i][j] = triangle_length;
	}
      }
    }
  }
}

// return the maximal distance from X0 to any node in the network.
double SimpleTemporalProblem::getMaximalTimePointInTightestSchedule() {
  double min = 0.0;
  for(size_t i = 0; i < number_of_nodes-1; i++) {
    double time = matrix[i][number_of_nodes-1];
    if(time < min) min = time;
  }
  return -min;
}

// reset all arc weights to INFINITY (except for self-loops, whose
// weights are reset to zero).
void SimpleTemporalProblem::reset() {
  for(size_t i = 0; i < number_of_nodes; i++) {
    for(size_t j = 0; j < number_of_nodes; j++) {
      matrix[i][j] = (i == j ? 0.0 : INF);
    }
  }
}

// dump problem
void SimpleTemporalProblem::dump() {
  std::cout << std::setw(50) << "";
  for(size_t i = 0; i < number_of_nodes; i++) {
    std::cout << std::setw(50) << variable_names[i];
  }
  std::cout << std::endl;
  for(size_t i = 0; i < number_of_nodes; i++) {
    std::cout << std::setw(50) << variable_names[i];
    for(size_t j = 0; j < number_of_nodes; j++) {
      std::cout << std::setw(50) << matrix[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

// dump solution
void SimpleTemporalProblem::dumpSolution() {

  std::vector<Happening> happenings = getHappenings();

  for(size_t i = 0; i < happenings.size(); i++) {
    assert(happenings[i].second <= number_of_nodes-1);
    std::string& node = variable_names[happenings[i].second];
    std::cout << "    " << node << "@" << happenings[i].first << std::endl;
  }
  std::cout << std::endl;
}

std::vector<Happening> SimpleTemporalProblem::getHappenings() {
    std::vector<Happening> happenings;
    for(size_t i = 0; i < number_of_nodes - 1; i++) {
        double time = -matrix[i][number_of_nodes - 1];
        happenings.push_back(std::make_pair(time, i));
    }

    sort(happenings.begin(), happenings.end(), HappeningComparator());

    return happenings;
}

//int main(void) {
//  enum { a_start = 0, a_end = 1,
//	 b_start = 2, b_end = 3,
//	 c_start = 4, c_end = 5};
//
//  std::vector<std::string> variable_names;
//  variable_names.push_back("a-");
//  variable_names.push_back("a+");
//  variable_names.push_back("b-");
//  variable_names.push_back("b+");
//  variable_names.push_back("c-");
//  variable_names.push_back("c+");
//
//  SimpleTemporalProblem problem(variable_names);
//
//  // assert that start time point of actions a, b, and c are non-negative
//  problem.setUnboundedIntervalFromXZero(0, 0.0);
//  problem.setUnboundedIntervalFromXZero(2, 0.0);
//  problem.setUnboundedIntervalFromXZero(4, 0.0);
//
//  // assert that differences between start and end time points are exactly
//  // the durations of the actions
//  problem.setSingletonInterval(a_start, a_end, 10.0);
//  problem.setSingletonInterval(b_start, b_end,  2.0);
//  problem.setSingletonInterval(c_start, c_end, 13.0);
//
//  // assert that causal relationships are preserved
//  problem.setUnboundedInterval(c_end, b_start, 0.0);
//  problem.setUnboundedInterval(b_start, a_end, 0.0);
//
//
//  std::cout << "Unsolved Simple Temporal Network:" << std::endl;
//  std::cout << "=================================" << std::endl;
//  problem.dump();
//
//  problem.solve();
//  double h = problem.getMaximalTimePointInTightestSchedule();
//
//  std::cout << "Solved Simple Temporal Network:" << std::endl;
//  std::cout << "===============================" << std::endl;
//  problem.dump();
//
//  std::cout << "Corresponding happenings:" << std::endl;
//  std::cout << "=========================" << std::endl;
//  problem.dumpSolution();
//
//  std::cout << "Corresponding heuristic value:" << std::endl;
//  std::cout << "==============================" << std::endl;
//  std::cout << h << std::endl;
//
//  return 0;
//}
