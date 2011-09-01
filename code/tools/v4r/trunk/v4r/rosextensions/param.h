/*
 * param.h
 *
 *  Created on: Feb 13, 2011
 *  Author: Markus Bader
 */

#ifndef PARAM_H_
#define PARAM_H_

namespace ROS{

void updateParameter(ros::NodeHandle n, const std::string &name, std::vector<std::string> &value, std::string defaults) {
  std::string tmp = defaults;
  n.getParam(name.c_str(), tmp);
  ROS_INFO("\t%s: ->  %s", name.c_str(), tmp.c_str());
  boost::erase_all(tmp, " ");
  boost::split(value, tmp, boost::is_any_of(";,"));
}

void updateParameter(ros::NodeHandle n, const std::string &name, int &value) {
  int tmp = value;
  n.getParam(name.c_str(), tmp);
  if (tmp != value) {
    ROS_INFO("\t%s: %i ->  %i", name.c_str(), value, tmp);
    value = tmp;
  }
}

void updateParameter(ros::NodeHandle n, const std::string &name, double &value) {
  double tmp = value;
  n.getParam(name.c_str(), tmp);
  if (tmp != value) {
    ROS_INFO("\t%s: %5.2f ->  %5.2f", name.c_str(), value, tmp);
    value = tmp;
  }
}
}
#endif /* PARAM_H_ */
