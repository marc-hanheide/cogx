/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_TESTER_HPP_4F7C3A59_
#define _TESTING_TESTER_HPP_4F7C3A59_

#include "castmachine.hpp"

#include <cast/architecture/ManagedComponent.hpp>

namespace testing
{

class CTester: public cast::ManagedComponent
{
public:
  void runComponent();
};

}// namespace
#endif /* _TESTING_TESTER_HPP_4F7C3A59_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
