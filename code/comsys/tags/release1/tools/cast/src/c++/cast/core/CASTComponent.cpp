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
using namespace Ice;

namespace cast {


  ComponentRunThread::ComponentRunThread(CASTComponent *_comp) : m_component(_comp) {}
  
  void 
  ComponentRunThread::run() {
    try {
      m_component->runComponent();
    }
    catch(const WMException &e) {
      m_component->println("Aborting after catching a WMException from runComponent()");
      m_component->println("The address involved in this error was: " + 
			   e.wma.id + " in " + 
			   e.wma.subarchitecture);	  		
      m_component->println("what(): %s", e.what());
      m_component->println("message: %s", e.message.c_str());
      std::abort();
    }    
    catch(const cast::CASTException & e) {
      m_component->println("Aborting after catching a CASTException from runComponent()");
      m_component->println("what(): %s", e.what());
      m_component->println("message: %s", e.message.c_str());
      std::abort();
    }
    catch(const Ice::Exception &e) {
      m_component->println("Aborting after catching an Ice::Exception from runComponent()");
      m_component->println("what(): %s", e.what());
      std::abort();
    }
    catch(const std::exception &e) {
      m_component->println("Aborting after catching std::exception from runComponent()");
      m_component->println(e.what());
      std::abort();
    }
    catch(...) {
      m_component->println("Aborting after catching unknown exception from runComponent()");
      std::abort();
    }


  }


  const std::string CASTComponent::END_COLOUR_ESCAPE = "\033[0m";

  CASTComponent::CASTComponent() : 
    m_componentID(""),
    m_startColourEscape(END_COLOUR_ESCAPE),
    m_startCalled(false),
    m_configureCalled(false),
    m_bLogOutput(false),
    m_bDebugOutput(false) {

  }


  CASTComponent::~CASTComponent() {

    //    println("CASTComponent::~CASTComponent()");
//UPGRADE
	  //    m_unlockNotificationMutex.lock();
//    m_pUnlockNotificationCondition->broadcast();
//    m_unlockNotificationMutex.unlock();
//    delete m_pUnlockNotificationCondition;
  }



  void 
  CASTComponent::run(const Ice::Current & _ctx) {
    assert(m_startCalled);
    //create a new thread
    m_runThread = new ComponentRunThread(this);
    //start thread to run component  
    m_runThreadControl = m_runThread->start();
  }

  
  void 
  CASTComponent::stop(const Ice::Current & _ctx)  {

      assert(m_startCalled);
      m_startCalled = false;

      //stop derived class
      stop();

      //stop internal processing
      stopInternal();

      //and remove from adapter
      //doesn't quite work yet
      //       getObjectAdapter()->hold();
      //       println("waiting for hold");
      //       getObjectAdapter()->waitForHold();
      //       println("held");
      
      //     getObjectAdapter()->remove(getIceIdentity());
      // reactivate after hold
      //       println("removed self");
      //       getObjectAdapter()->activate();
      

    }

  void CASTComponent::stop() {
  }

  void CASTComponent::stopInternal() {
    debug("trying to join runComponent()");
    m_runThreadControl.join();

    //UPGRADE
    //    //release sleeping threads
    //    m_unlockNotificationMutex.lock();
    //    m_pUnlockNotificationCondition->broadcast();
    //    m_unlockNotificationMutex.unlock();
  }


  void CASTComponent::println(const char *format, ...) const {
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);
    startColourise(cout);
    cout<<"[\""<<getComponentID()<<"\": "<< msg<<"]"; 
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
      cout<<"[LOG \""<<getComponentID()<<"\": "<<msg<<"]"; 
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
      cout<<"[DEBUG \""<<getComponentID()<<"\": "<< msg <<"]"; 
      endColourise(cout);
      cout<<endl;
    }
  }

  void CASTComponent::debug(const string & _s) const {
    debug(_s.c_str());
  }

  void CASTComponent::start() {
  }

  void CASTComponent::startInternal() {
    debug("CASTComponent::startInternal()");
    assert(!m_startCalled);
    m_startCalled = true;
  }

  void CASTComponent::configure(const map<string,string> & _config) {
  }

  void CASTComponent::configureInternal(const map<string,string> & _config) {

//     cout<<"CASTComponent::configure: "<<endl;

//     for(map<string,string>::const_iterator cf = _config.begin();
// 	cf != _config.end(); ++cf) {
//       cout<<"config: "<<cf->first<<" "<<cf->second<<endl;
//     }
    

    
    m_configureCalled = true;

    map<string,string>::const_iterator i = _config.find(cdl::LOGKEY);

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

    i = _config.find(cdl::DEBUGKEY);

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

    i = _config.find(cdl::COMPONENTNUMBERKEY);

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


  void CASTComponent::lockComponent() {
    m_componentMutex.lock();
  }

  /**
   * Release the semaphore for access to this component.
   */
  void CASTComponent::unlockComponent() {

   //m_semaphore.post();
   m_componentMutex.unlock();

//    m_unlockNotificationMutex.lock();
//    m_pUnlockNotificationCondition->broadcast();
//    m_unlockNotificationMutex.unlock();  

  }


//   void CASTComponent::waitForUnlock() {
// //UPGRADE
// 	  //    m_unlockNotificationMutex.lock();
// //    m_pUnlockNotificationCondition->wait();
// //    m_unlockNotificationMutex.unlock();
//   }

  //#include <errno.hpp>

  void CASTComponent::sleepComponent(unsigned long _millis) {
    IceUtil::Time t = IceUtil::Time::milliSeconds(_millis); 
    IceUtil::ThreadControl::sleep(t);
  }
  
  void 
  CASTComponent::destroy(const Ice::Current & _crt) {
    destroy();
    destroyInternal(_crt);
  }
  
  void 
  CASTComponent::destroyInternal(const Ice::Current& _crt) {
    
    for(vector<Identity>::iterator i = m_serverIdentities.begin();
 	i < m_serverIdentities.end(); ++i) {
      getObjectAdapter()->remove(*i);
    }

    _crt.adapter->remove(_crt.id);
  }

  /**
   * Resolve an Ice server using the given details.
   */
  Ice::ObjectPrx 
  CASTComponent::getIceServer(const std::string & _name, const std::string & _category, 
			      const std::string & _host, unsigned int _port) const {
    
    Ice::Identity id;
    id.name = _name;
    id.category = _category;
    
    std::ostringstream serverAddr;
    serverAddr << getCommunicator()->identityToString(id)
	       << ":default -h " << _host << " -p " << _port;
    
    Ice::ObjectPrx base = getCommunicator()->stringToProxy(serverAddr.str());
    
    return base;
  }



} //namespace cast 
