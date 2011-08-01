#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <vector>
#include <string>
#include "globals.h"

const double INF = 10000000.0;

typedef std::pair<double,int> Happening;

class SimpleTemporalProblem {
private:

  struct HappeningComparator {
    bool operator()(const Happening& h1, const Happening& h2) {
      if(h1.first < h2.first) return true;
      if(h2.first < h1.first) return false;
      return h1.second < h2.second;
    }
  };

  typedef std::vector<double> MatrixLine;
  typedef std::vector<MatrixLine> DistanceMatrix;

  int number_of_nodes;
  std::vector<std::string> variable_names;
  DistanceMatrix matrix;

  void initialize();
  void setDistance(int from, int to, double distance);
  double add(double a, double b);

public:
  SimpleTemporalProblem(int _number_of_nodes);
  SimpleTemporalProblem(std::vector<std::string> _variable_names);

  void setInterval(int from, int to, double lower, double upper);

  void setSingletonInterval(int from, int to, double lowerAndUpper);
  void setUnboundedInterval(int from, int to, double lower);

  void setIntervalFromXZero(int to, double lower, double upper);
  void setUnboundedIntervalFromXZero(int to, double lower);

  void solve();
  double getMaximalTimePointInTightestSchedule();
  void reset();

  void dump();
  void dumpSolution();
  std::vector<Happening> getHappenings();
};



#endif
