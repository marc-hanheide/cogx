#ifndef MONITORING_H
#define MONITORING_H

#include <vector>
#include <string>

#include "search_engine.h"
#include "globals.h"

class MonitorEngine
{
 protected:
  MonitorEngine();
  static MonitorEngine* instance;

 public:
  ~MonitorEngine();

  static MonitorEngine* getInstance();

  bool validatePlan(const std::vector<PlanStep>& plan);
  bool validatePlan(std::vector<std::string>& plan);
};

#endif

