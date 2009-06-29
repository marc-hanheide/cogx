/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Michael Zillich, Nick Hawes
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

#ifndef ABSTRACT_TESTER_HPP
#define ABSTRACT_TESTER_HPP

#include <cast/core.hpp>
#include <cast/architecture.hpp>

#include <queue>
#include <pthread.h>

namespace cast {

  class AbstractTester : public ManagedComponent {

  protected:

    class AbstractTest {
  
    public:
    
      AbstractTest(AbstractTester & _tester);
      virtual ~AbstractTest();
      
      void run();
      bool waitFor();
    
    protected:

      friend  void * runTest( void * _ptr );

      virtual void startTest() = 0;
      void testComplete(bool _passed);    

      //make a bunch of common cast calls available

      std::string newDataID() {
	return m_tester.newDataID();
      }

      void println(const std::string & _s) const {
	m_tester.println(_s);
      }

      void println(const char *format, ...) const {
	
	char msg[1024];
	va_list arg_list;
	va_start(arg_list, format);
	vsnprintf(msg, 1024, format, arg_list);
	va_end(arg_list);
	m_tester.println(msg);
  
      }

      void log(const std::string & _s) const {
	m_tester.log(_s);
      }

      void log(const char *format, ...) const {
	      
	char msg[1024];
	va_list arg_list;
	va_start(arg_list, format);
	vsnprintf(msg, 1024, format, arg_list);
	va_end(arg_list);
	m_tester.log(msg);
  
      }

      void sleepComponent(long _millis) {
	return m_tester.sleepComponent(_millis);
      }

      std::string getSubarchitectureID() {
	return m_tester.getSubarchitectureID();
      }

      std::string getComponentID() {
	return m_tester.getComponentID();
      }

      
      template <class T>
      void addToWorkingMemory(const std::string &_id, 
			      IceInternal::Handle<T> _data) {    	
	m_tester.addToWorkingMemory(_id,_data);
      }

      template <class T>
      void addToWorkingMemory(const std::string &_id, 
			      const std::string &_subarchitecture, 
			      IceInternal::Handle<T> _data) {    
	m_tester.addToWorkingMemory(_id,_subarchitecture,_data);
      }
      
      template <class T>
      void addToWorkingMemory(const cdl::WorkingMemoryAddress &_wma, 
			      IceInternal::Handle<T> _data) {    
	m_tester.addToWorkingMemory(_wma,_data);
      }

      template <class T>
      void overwriteWorkingMemory(const std::string &_id, 
				  IceInternal::Handle<T> _data) {    
	m_tester.overwriteWorkingMemory(_id,_data);
      }

      template <class T>
      void overwriteWorkingMemory(const std::string &_id, 
				  const std::string &_subarchitecture, 
				  IceInternal::Handle<T> _data) {    
	m_tester.overwriteWorkingMemory(_id,_subarchitecture,_data);
      }

      template <class T>
      void overwriteWorkingMemory(const cdl::WorkingMemoryAddress & _wma,
				  IceInternal::Handle<T> _data) {    
	m_tester.overwriteWorkingMemory(_wma,_data);
      }

      

      void deleteFromWorkingMemory(const cdl::WorkingMemoryAddress & _wma) {    
	m_tester.deleteFromWorkingMemory(_wma );
      }


      bool existsOnWorkingMemory(const cdl::WorkingMemoryAddress & _wma) {
	return m_tester.existsOnWorkingMemory(_wma);
      }
      



      template <class T>
      IceInternal::Handle<T> getMemoryEntry(const std::string & _id) {
	return getMemoryEntry<T>(_id,m_tester.getSubarchitectureID());
      }

      template <class T>
      IceInternal::Handle<T> getMemoryEntry(const cdl::WorkingMemoryAddress & _wma) {
	return getMemoryEntry<T>(std::string(_wma.id),std::string(_wma.subarchitecture));
      }

      template <class T>
      IceInternal::Handle<T> getMemoryEntry(const std::string & _id,
					const std::string & _subarch) {     
	return m_tester.getMemoryEntry<T>(_id,_subarch);
      }

      template <class T>
      void  getMemoryEntries(std::vector < IceInternal::Handle<T> > & _results) {
	m_tester.getMemoryEntries(_results);
      }

      template <class T>
      void  getMemoryEntries(const std::string & _subarch,				  
			     const int & _count,
			     std::vector < IceInternal::Handle<T> > & _results) {
	m_tester.getMemoryEntries<T>(_results, _subarch, _count);
      }

      void addChangeFilter(const cdl::WorkingMemoryChangeFilter & _filter,
			   WorkingMemoryChangeReceiver * _pReceiver) {
	m_tester.addChangeFilter(_filter, _pReceiver);
      }

      void removeChangeFilter(const WorkingMemoryChangeReceiver * _pReceiver,
			      const cdl::ReceiverDeleteCondition & _condition = cdl::DONOTDELETERECEIVER) {
	m_tester.removeChangeFilter(_pReceiver, _condition);
      }


      int getFilterCount() const {
	return m_tester.getFilterCount();
      }

      void lockEntry(const cdl::WorkingMemoryAddress & _wma,
		     const cdl::WorkingMemoryPermissions & _permissions)
	throw(DoesNotExistOnWMException) {
	m_tester.lockEntry(_wma,_permissions);
      }

      void lockEntry(const std::string & _id,
		     const std::string & _subarch,
		     const cdl::WorkingMemoryPermissions & _permissions)
	throw(DoesNotExistOnWMException) {
	m_tester.lockEntry(_id,_subarch,_permissions);
      }

      bool tryLockEntry(const cdl::WorkingMemoryAddress & _wma,
			const cdl::WorkingMemoryPermissions & _permissions)
	throw(DoesNotExistOnWMException) {
	return m_tester.tryLockEntry(_wma,_permissions);
      }

      void unlockEntry(const cdl::WorkingMemoryAddress & _wma) {
	m_tester.unlockEntry(_wma);
      
      } 
      
      bool isReadable(const std::string & _id,
		      const std::string & _subarch) {
	return m_tester.isReadable(_id,_subarch);
      }
      
      void unlockEntry(const std::string & _id,
		       const std::string & _subarch)
	throw(DoesNotExistOnWMException) {
	m_tester.unlockEntry(_id,_subarch);
      }


      void lockComponent() {
	m_tester.lockComponent();
      }

      void unlockComponent() {
	m_tester.unlockComponent();
      }
      
      

      AbstractTester & m_tester;

    private:
      bool m_testComplete;			
      bool m_passed;
      pthread_mutex_t m_completionLock;	   

    

    };


    friend class AbstractTest;

    friend  void * runTest( void * _ptr );

    virtual void runComponent();
    virtual void configure(const std::map<std::string,std::string> & _config);
    virtual void taskAdopted(const std::string &_taskID) {};
    virtual void taskRejected(const std::string &_taskID) {};

    void registerTest(std::string _id, boost::shared_ptr<AbstractTest> _test) throw(CASTException);
    void queueTest(std::string _id) throw(CASTException);


  public:
    AbstractTester();
    virtual ~AbstractTester();
    
    
    typedef StringMap<boost::shared_ptr<AbstractTest> >::map TestMap;

  private:

    bool runTests();
    bool performTest(const boost::shared_ptr<AbstractTest> & _test);
    
    TestMap m_tests;    
    std::queue<std::string> m_perform;
    
    bool m_exitOnCompletion;				  
    int m_successValue;
    int m_failureValue;

    pthread_t m_testThread;

          

  };

} //namespace cast

#endif

