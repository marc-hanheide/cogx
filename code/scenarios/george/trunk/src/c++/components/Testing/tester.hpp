/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_TESTER_HPP_4F7C3A59_
#define _TESTING_TESTER_HPP_4F7C3A59_

#include "castmachine.hpp"

#include <cast/architecture/ManagedComponent.hpp>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <map>
#include <string>

namespace testing
{

class CTester: public cast::ManagedComponent
{
  std::vector<CMachinePtr> mMachines;

protected:
  void configure(const std::map<std::string,std::string> & _config);
  void start();
  void runComponent();

public:
#ifdef FEAT_VISUALIZATION
  cogx::display::CDisplayClient mDisplay;
#endif
};

}// namespace
#endif /* _TESTING_TESTER_HPP_4F7C3A59_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
