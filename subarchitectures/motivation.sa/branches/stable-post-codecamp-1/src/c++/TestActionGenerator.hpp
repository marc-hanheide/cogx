#ifndef TEST_ACTION_GENERATOR_HPP
#define TEST_ACTION_GENERATOR_HPP

#include <cast/architecture/ManagedComponent.hpp>

namespace execution {

  /*
   * Class to test proxy reading and writing in C++.
   */
  class TestActionGenerator : 
    public cast::ManagedComponent {
  protected:
    void runComponent();  
    virtual void start();
  private:
    void actionOverwrite(const cast::cdl::WorkingMemoryChange &_wmc);

  };
  
}

#endif
