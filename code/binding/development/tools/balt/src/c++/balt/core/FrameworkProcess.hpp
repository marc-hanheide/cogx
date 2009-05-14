/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 * 
 */

#ifndef FRAMEWORK_PROCESS_H_
#define FRAMEWORK_PROCESS_H_

#include "includes.hpp"

//CORBA thread abstraction
#include <omnithread.h>

class LocalConnectionManager;


/**
 * Abstract class for a process in C++. The process is based on the
 * omni_thread class to support multi-threading. Users must implement
 * the stop and run_undetacted methods in order to provide some
 * functionality. Each component must be compiled into a dynamically
 * loadable library of its own, and that library must contain a
 * function called newComponent that returns a new instance of the
 * component. This function should look something like...
 *
 *
 * extern "C" {
 * FrameworkProcess* newComponent(const std::string &_id) {
 *   return new ComponentName(_id);
 * }
 }

 * 
 * 
 */
class FrameworkProcess :  public omni_thread {


public:

  /**
   * Enum to determine the state of this process;
   */
  enum ProcessStatus {STATUS_STOP,STATUS_RUN};

  /**
   * Process constructor. Contructs the super class and stores the
   * process ID.
   * 
   * @param _id A std::string that must uniquely identify this process.
   */
  FrameworkProcess(const std::string & _id)
    : omni_thread() {
    m_processID = std::string(_id);
    m_status = STATUS_STOP;
  }
      
  virtual void configure(std::map<std::string,std::string> & _config) = 0;
    

  /**
   * Get the process identifier of this process.
   */
  const std::string & getProcessIdentifier() const {
    return m_processID;
  }
  
      
  /**
   * Signal that the processing is about to start.
   */
  virtual void start() {
    m_status = STATUS_RUN;
  }
  
  /**
   * Start the process running. This method should always be called
   * class as it actually starts the thread.
   */
  void run() {
    start_undetached();
  }


  /**
   * Stop the process running. This does not handle thread operations,
   * but should be used to inform the underlying process that it
   * should stop what it's doing as the thread is about to be stopped
   * (with omni_thread::join).
   */
  virtual void stop() {
    m_mutex.lock();
    m_status = STATUS_STOP;
    m_mutex.unlock();
  }
  
  
protected:

  /**
   * Destructor is protected and should only be called via the
   * omni_thread::join/exit methods.
   */
  virtual ~FrameworkProcess() {/*cout<<"~FrameworkProcess"<<endl;*/
  }

  /**
   * Method called when thread is run. This method should be the start
   * of the actual processing of the derived class.
   */
  virtual void* run_undetached(void *arg)  = 0;

  /**
   * Status of the processes.
   */
  ProcessStatus m_status;


  /**
   * Mutex for controlling thread access
   */
  //omni_semaphore m_semaphore;
  omni_mutex m_mutex;


private:

  /**
   * The ID of this process. This is currently protected by this class
   * to ensure it remains unchanged after process construction.
   */
  std::string m_processID;

};

#endif
