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

#include "CASTUIComponent.hpp"

using namespace std;
using namespace cast::cdl;
using namespace cast::cdl::ui;

namespace cast {

  CASTUIComponent::CASTUIComponent(const string &_id) : 
    //InspectableComponent(_id),
    CASTComponent(_id) {

    //m_pComponentEventPushConnector = NULL;
    m_pTextOutputConnector = NULL;

    m_bDebugEvents = false;

    m_componentStatus.m_component = CORBA::string_dup(_id.c_str());
    m_componentStatus.m_changeQueue = 0;
    m_componentStatus.m_totalChangeEventsFiltered = 0;
    m_componentStatus.m_totalChangeEventsReceived = 0;
    m_componentStatus.m_totalAdds = 0;
    m_componentStatus.m_totalOverwrites = 0;
    m_componentStatus.m_totalDeletes = 0;
    m_componentStatus.m_totalReads = 0;
    m_componentStatus.m_totalProposals = 0;
    m_componentStatus.m_totalStarts = 0;
    m_componentStatus.m_totalEnds = 0;
 
  }


  void CASTUIComponent::configure(map<string,string> & _config) {

    CASTComponent::configure(_config);

    map<string,string>::const_iterator i 
      = _config.find(cdl::SUBARCH_ID_KEY);

    if(i != _config.end()) {
      m_componentStatus.m_subarchitecture 
	= CORBA::string_dup(i->second.c_str());
    }
    else {
      m_componentStatus.m_subarchitecture 
	= CORBA::string_dup("no subarchitecture");
    }

    i = _config.find(cdl::DEBUG_EVENTS_KEY);

    if(i != _config.end()) {
      m_bDebugEvents = true;
    }


    m_componentStatus.m_log = m_bLogOutput;
    m_componentStatus.m_debug = m_bDebugOutput;

  }

  void CASTUIComponent::receivePullQuery(const FrameworkQuery & _query, 
					 FrameworkLocalData<ComponentStatus> *& _pData) {

    m_componentStatusMutex.lock();
    _pData 
      = new FrameworkLocalData<ComponentStatus>(getProcessIdentifier(),
						new ComponentStatus(m_componentStatus));  
    m_componentStatusMutex.unlock();
  }


  void CASTUIComponent::textToUI(const  char * _s, const OutputType & _type) const {

    TextOutput *pTO = new TextOutput();
    pTO->m_type = _type;
    pTO->m_string = CORBA::string_dup(_s);
  
    FrameworkLocalData<TextOutput> * fld = 
      new FrameworkLocalData<TextOutput>(getProcessIdentifier(),pTO);
  
    m_textOutMutex.lock();
    m_pTextOutputConnector->push(fld);
    m_textOutMutex.unlock();
  }


//   void CASTUIComponent::println(const char *format, ...) const {
//     char msg[1024];
//     va_list arg_list;
//     va_start(arg_list, format);
//     vsnprintf(msg, 1024, format, arg_list);
//     va_end(arg_list);
//     println(msg);
//   }

  void CASTUIComponent::println(const char *format, ...) const {
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);
  
    if(hasUIConnection()) {
      textToUI(msg,PRINT);
    }
    else {
      CASTComponent::println(msg);
    }

  }

  void CASTUIComponent::println(const string & _s) const {
    println(_s.c_str());
  }

  void CASTUIComponent::log(const char * format, ...) const {

    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    if(hasUIConnection()) {
      textToUI(msg,LOG);
    }
    else {
      CASTComponent::log(msg);
    }
  
  }

  void CASTUIComponent::log(const string & _s) const {
    log(_s.c_str());
  }

  
  void CASTUIComponent::debug(const char * format,...) const {
    
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);
    
    if(hasUIConnection()) {
      textToUI(msg,DEBUG);
    }
    else {
      CASTComponent::debug(msg);
    }
    
  }

  void CASTUIComponent::debug(const string & _s) const {
    debug(_s.c_str());
  }


  // void CASTUIComponent::logEvent(ComponentEvent * _pEvent) {
  //   m_componentEventMutex.lock();
  //   m_pComponentEventPushConnector
  //     ->push(new FrameworkLocalData<ComponentEvent>(getProcessIdentifier(),_pEvent));
  //   m_componentEventMutex.unlock();
  // }

  void CASTUIComponent::updateStatus(const ComponentEventType &_event) const {
    switch (_event) {
    case cdl::ui::ADD:
      m_componentStatus.m_totalAdds++;
      break;
    case cdl::ui::OVERWRITE:
      m_componentStatus.m_totalOverwrites++;
      break;
    case cdl::ui::DELETE:
      m_componentStatus.m_totalDeletes++;
      break;
    case cdl::ui::GET:
      m_componentStatus.m_totalReads++;
      break;
    case cdl::ui::PROPOSED:
      m_componentStatus.m_totalProposals++;
      break;
    case cdl::ui::START:
      m_componentStatus.m_totalStarts++;
      break;
    case cdl::ui::END:
      m_componentStatus.m_totalEnds++;
      break;
    default:
      break;
    }
  }

  void CASTUIComponent::logEvent(const ComponentEventType & _event, 
				 const string & _componentID, 
				 const string & _target, 
				 const string & _dataType,  
				 const string & _dataID) const {
  
    //increase count for particular event
    updateStatus(_event);

    if(m_bDebugEvents) {
      //   send this to debug output
      ostringstream outStream;
      outStream<<toString(_event)<<
	" compID: "<<_componentID<<
	" target: "<<_target<<
	" datatype: "<<_dataType<<
	" dataID: "<<_dataID;
      debug(outStream.str());
    }    

    //   if(m_pComponentEventPushConnector != NULL) {
    
    

    //     ComponentEvent *pCE = new ComponentEvent();
    //     pCE->m_event = _event;
    //     pCE->m_componentID = CORBA::string_dup(_componentID.c_str());
    //     pCE->m_target = CORBA::string_dup(_target.c_str());
    //     pCE->m_dataType = CORBA::string_dup(_dataType.c_str());
    //     pCE->m_dataID = CORBA::string_dup(_dataID.c_str());
    //     pCE->m_time = BALTTimer::getBALTTime();

    //     logEvent(pCE);
    //   }
  }


} //namespace cast
