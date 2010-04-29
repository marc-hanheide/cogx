#ifndef HELLO_WRITER_HPP_
#define HELLO_WRITER_HPP_

#include <cast/architecture.hpp>
#include <HelloWorldData.hpp>


class HelloWriter :
  public cast::ManagedComponent {
protected:
  virtual void runComponent();

};


#endif

