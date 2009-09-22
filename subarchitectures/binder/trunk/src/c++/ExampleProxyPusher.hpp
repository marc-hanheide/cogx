#ifndef EXAMPLE_PROXY_PUSHER_HPP
#define EXAMPLE_PROXY_PUSHER_HPP

#include <cast/architecture/ManagedComponent.hpp>
#include <autogen/BinderEssentials.hpp>
#include <BindingWorkingMemoryWriter.hpp>
#include <BindingWorkingMemoryReader.hpp>


/*
 * Class to test proxy reading and writing in C++.
 */
class ExampleProxyPusher : 
  public binder::BindingWorkingMemoryWriter,
  public binder::BindingWorkingMemoryReader  
{
protected:
  virtual void start();
  void runComponent();  
};

#endif
