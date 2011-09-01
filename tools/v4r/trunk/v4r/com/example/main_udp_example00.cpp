//
// blocking_udp_echo_client.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2010 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <v4r/com/udphdl.h>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/split.hpp>


struct ComData {
  double x;
  double y;
  double z;
  /// produces a human readable string of the camera parameters
  std::string human_readable() const {
    std::stringstream ss;
    ss << "data = [" << std::setw(8) << std::showpos << std::setprecision(5) << x << ", " << y << ", " << z << "];";
    return ss.str();
  }
  int human_readable(const std::string &str) {
    if (str.empty()) return 1;
    std::string tmp = str;
    std::vector<std::string> values;
    boost::erase_all(tmp, " ");
    boost::split(values, tmp, boost::is_any_of(";,"));
    if(values.size() != 3) return 1;
    if (values.size() > 0) x = boost::lexical_cast<double>(values[0]);
    if (values.size() > 1) y = boost::lexical_cast<double>(values[1]);
    if (values.size() > 2) z = boost::lexical_cast<double>(values[2]);
    return 0;
  }
};

inline std::ostream& operator <<(std::ostream &os, const ComData &r) {
  return os << r.human_readable();
}
;

int program_options(int argc, char* argv[], std::string &configFile, bool &receiver, std::string &host, short &port, std::string &dataStr) {
  receiver = false;
  boost::program_options::options_description desc("Allowed Parameters");
  desc.add_options()//
  ("help", "get this help message")//
  ("configfile,c", boost::program_options::value<std::string>(&configFile), "Config file")//
  ("receiver,r", "if set the program acts as receiver: default is transmitter")//
  ("host,h", boost::program_options::value<std::string>(&host)->default_value("127.0.0.1"), "Host")//
  ("port,p", boost::program_options::value<short>(&port)->default_value(25000), "Port")//
  ("data,d", boost::program_options::value<std::string>(&dataStr)->default_value(".0, .0, .0"), "data -> default: 0, 0, 0");

  std::cout << std::endl;

  boost::program_options::variables_map vm;
  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  } catch (const std::exception& ex) {
    std::cout << desc << "\n";
    return 1;
  }
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }
  if (vm.count("receiver")) {
    receiver = true;
  }

  if (vm.count("configfile")) {
    std::ifstream file(configFile.c_str(), std::ifstream::in);
    if (file.is_open()) {
      try {
        boost::program_options::store(boost::program_options::parse_config_file(file, desc), vm);
        boost::program_options::notify(vm);
      } catch (const std::exception& ex) {
        std::cout << "Error reading config file: " << ex.what() << std::endl;
        return 1;
      }
    } else {
      std::cout << "Error opening config file " << configFile << std::endl;
      return 1;
    }
  }
  return 0;
}

void doSomeThing(ComData &d){
        std::cout << d << std::endl;
}

int main(int argc, char* argv[]) {
  using boost::asio::ip::udp;
  std::string configFile;
  std::string host;
  std::string dataStr;
  short port;
  bool receiver;
  program_options(argc, argv, configFile, receiver, host, port, dataStr);

  try {
    V4R::UDPHdl<ComData> s;
    if (receiver) {
      ComData data;
      s.initReceiver(port);
      s.runThread();
      int i = 0;
      
      while (i == 0) {
        i = s.deque(data);
      }
      doSomeThing(data);
      s.stopThread();

    } else {
      ComData data;
      if(data.human_readable(dataStr)){
       std::cerr << "Wrong number of data argumetns!\n";
      }
      s.initTransmitter(host, port);
      s.send(data);
      std::cout << data.human_readable() << std::endl;
    }

  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}

