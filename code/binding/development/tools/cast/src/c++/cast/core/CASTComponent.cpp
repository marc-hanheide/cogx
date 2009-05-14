/*
 * CAST - The CoSy Architecture Schema Toolkit
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
 
#include "CASTComponent.hpp"

using namespace std;

namespace cast {
  const std::string CASTComponent::END_COLOUR_ESCAPE = "\033[0m";

  CASTComponent::CASTComponent(const string &_id) : 
    InspectableComponent(_id),
    m_bLogOutput(false),
    m_bDebugOutput(false),
    m_startColourEscape(END_COLOUR_ESCAPE),
    m_startCalled(false),
    m_configureCalled(false) {
    m_pUnlockNotificationCondition
      = new omni_condition(&m_unlockNotificationMutex);


    static bool init = false;
    if(!init) {
      initCASTDatatypes();
      init = true;
    }

  }


  CASTComponent::~CASTComponent() {
    m_unlockNotificationMutex.lock();
    m_pUnlockNotificationCondition->broadcast();
    m_unlockNotificationMutex.unlock();
    delete m_pUnlockNotificationCondition;
  }

  void CASTComponent::stop() {
    //FrameworkProcess::
    InspectableComponent::stop();

    //release sleeping threads
    m_unlockNotificationMutex.lock();
    m_pUnlockNotificationCondition->broadcast();
    m_unlockNotificationMutex.unlock();
  }

  void CASTComponent::println(const char *format, ...) const {
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);
    startColourise(cout);
    cout<<"[\""<<getProcessIdentifier()<<"\": "<< msg<<"]"; 
    endColourise(cout);
    cout<<endl;
  }

  void CASTComponent::println(const string & _s) const {
    println(_s.c_str());
  }


  void CASTComponent::log(const char *format, ...) const {
    if(m_bLogOutput) {
      char msg[1024];
      va_list arg_list;
      va_start(arg_list, format);
      vsnprintf(msg, 1024, format, arg_list);
      va_end(arg_list);
      startColourise(cout);
      cout<<"[LOG \""<<getProcessIdentifier()<<"\": "<<msg<<"]"; 
      endColourise(cout);
      cout<<endl;
    }  
  }

  void CASTComponent::log(const string & _s) const {
    log(_s.c_str());
  }




  void CASTComponent::debug(const char * format, ...) const {
    if(m_bDebugOutput) {
      
      char msg[1024];
      va_list arg_list;
      va_start(arg_list, format);
      vsnprintf(msg, 1024, format, arg_list);
      va_end(arg_list);
      startColourise(cout);
      cout<<"[DEBUG \""<<getProcessIdentifier()<<"\": "<< msg <<"]"; 
      endColourise(cout);
      cout<<endl;
    }
  }

  void CASTComponent::debug(const string & _s) const {
    debug(_s.c_str());
  }




  void* CASTComponent::run_undetached(void *arg) {

    //assert that start has been called by the subclass
    assert(m_startCalled);
    //assert that configure has been called by the subclass
    assert(m_configureCalled);

    try {

      runComponent();
      int* rv = new int(0);
      return (void*)rv;
    }
    catch(const BALTException & e) {
      println("Aborting after catching BALTException exception from runComponent()");
      println("what(): %s", e.what());
      std::abort();
    }
    catch(const std::exception &e) {
      println("WorkingMemoryChangeThread::forwardToSubclass std::exception caught");
      println(e.what());
      std::abort();
    }
    catch(const CORBA::BAD_PARAM &e) {
      println("Aborting after catching CORBA::BAD_PARAM exception from runComponent()");
      println("_name(): %s", e._name());
      std::abort();
    }
    catch(const CORBA::Exception &e) {
      println("Aborting after catching CORBA::Exception exception from runComponent()");
      println("_name(): %s", e._name());
      std::abort();
    }
    catch(...) {
      println("Aborting after catching unknown exception from runComponent()");
      std::abort();
    }



  }
  
  void CASTComponent::start() {
    InspectableComponent::start();
    m_startCalled = true;
  }

  void CASTComponent::configure(map<string,string> & _config) {

    //cout<<"CASTProcessingComponent::configure: "<<_config<<endl;
    
    m_configureCalled = true;

    map<string,string>::const_iterator i = _config.find(cdl::LOG_KEY);

    if(i != _config.end()) {
      string logValue = i->second;
      if(logValue == "true") {
	m_bLogOutput = true;
      }
      else if(logValue == "false") {
	m_bLogOutput = false;
      }
      else {
	println(string("config err, unknown value for log") + logValue); 
      }   
    }

    i = _config.find(cdl::DEBUG_KEY);

    if(i != _config.end()) {
      string logValue = i->second;
      if(logValue == "true") {
	m_bDebugOutput = true;
      }
      else if(logValue == "false") {
	m_bDebugOutput = false;
      }
      else {
	println(string("config err, unknown value for log") + logValue); 
      }   
    }

    i = _config.find(cdl::COMPONENT_NUMBER_KEY);

    assert(i != _config.end());
    string numberString = i->second;
    int myNumber = atoi(numberString.c_str());
    int printNumber = (myNumber % 7);
    int bold = (myNumber / 7) % 2;

    if(printNumber == 0) {
      //0 = do nothing
      m_startColourEscape = "\033[0m";
    }
    else {
      //otherwise use the connected colour
      ostringstream outStream;
      outStream<<"\033[3"<<printNumber;
      if(bold != 0) {
	outStream<<";1m";      
      }
      else {
	outStream<<"m";      
      }
      m_startColourEscape = outStream.str();
    }

    //println("MEMEMEMEMEMEMEME");
  }
  /**
   * Acquire the semaphore for access to this component.
   */

  std::ostream & 
  CASTComponent::startColourise(std::ostream &_stream) const {
    _stream<<m_startColourEscape;
    return _stream;
  }
  
  std::ostream & 
  CASTComponent::endColourise(std::ostream &_stream) const {
    _stream<<END_COLOUR_ESCAPE;
    return _stream;
  }


  void CASTComponent::lockProcess() {
    //println("locking");
    //m_semaphore.wait();
    m_mutex.lock();
    //println("locked");
  }

  /**
   * Release the semaphore for access to this component.
   */
  void CASTComponent::unlockProcess() {
    // println("unlocking");
    //m_semaphore.post();
    m_mutex.unlock();

    m_unlockNotificationMutex.lock();
    m_pUnlockNotificationCondition->broadcast();
    m_unlockNotificationMutex.unlock();  
    //  println("unlocked");

    omni_thread::yield();
    //self()->yield();

  }


  void CASTComponent::waitForUnlock() {
    m_unlockNotificationMutex.lock();
    m_pUnlockNotificationCondition->wait();
    m_unlockNotificationMutex.unlock();
  }

  //#include <errno.hpp>

  void CASTComponent::sleepProcess(unsigned long _millis) {
 
    int seconds = _millis / 1000;
    //  cout<<"secs: "<<seconds<<endl;
    _millis -= (seconds * 1000);
    //  int err = 0;
    long nanos = _millis * 1000000;
    //  cout<<"nanos: "<<nanos<<endl;
  
    omni_thread::sleep(seconds,nanos);

    //   timespec ts;
    //   ts.tv_sec = seconds;
    //   ts.tv_nsec = nanos;
    //   int err = nanosleep (&ts, NULL);

    //   if(err != 0) {
    //     throw CASTException(__HERE__, "failed to sleep: %s", strerror(err));
    //   }

  }



} //namespace cast 
