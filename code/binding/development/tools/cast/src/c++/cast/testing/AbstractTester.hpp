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

#include <cast/core/CASTCore.hpp>
#include <cast/architecture/PrivilegedManagedProcess.hpp>

#include <queue>
#include <pthread.h>

namespace cast {

  class AbstractTester : public PrivilegedManagedProcess {

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

      void sleepProcess(long _millis) {
	return m_tester.sleepProcess(_millis);
      }

      std::string getSubarchitectureID() {
	return m_tester.m_subarchitectureID;
      }

      std::string getProcessIdentifier() {
	return m_tester.getProcessIdentifier();
      }

      
      template <class T>
      void addToWorkingMemory(const std::string &_id, 
			      T * _data,
			      const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    	
	m_tester.addToWorkingMemory(_id,_data,_sync);
      }

      template <class T>
      void addToWorkingMemory(const std::string &_id, 
			      const std::string &_subarchitecture, 
			      T * _data,
			      const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    
	m_tester.addToWorkingMemory(_id,_subarchitecture,_data,_sync);
      }
      
      template <class T>
      void addToWorkingMemory(const cdl::WorkingMemoryAddress &_wma, 
			      T * _data,
			      const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    
	m_tester.addToWorkingMemory(_wma,_data,_sync);
      }

      template <class T>
      void overwriteWorkingMemory(const std::string &_id, 
				  T * _data,
				  const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    
	m_tester.overwriteWorkingMemory(_id,_data,_sync);
      }

      template <class T>
      void overwriteWorkingMemory(const std::string &_id, 
				  const std::string &_subarchitecture, 
				  T * _data,
				  const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    
	m_tester.overwriteWorkingMemory(_id,_subarchitecture,_data,_sync);
      }

      template <class T>
      void overwriteWorkingMemory(const cdl::WorkingMemoryAddress & _wma,
				  T * _data,
				  const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    
	m_tester.overwriteWorkingMemory(_wma,_data,_sync);
      }

      

      void deleteFromWorkingMemory(const cdl::WorkingMemoryAddress & _wma,
				   const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    
	m_tester.deleteFromWorkingMemory(_wma ,_sync);
      }


      bool existsOnWorkingMemory(const cdl::WorkingMemoryAddress & _wma) {
	return m_tester.existsOnWorkingMemory(_wma);
      }
      



      template <class T>
      boost::shared_ptr< const CASTData<T> > getWorkingMemoryEntry(const std::string & _id) {
	return getWorkingMemoryEntry<T>(_id,m_tester.m_subarchitectureID);
      }

      template <class T>
      boost::shared_ptr< const CASTData<T> > getWorkingMemoryEntry(const cdl::WorkingMemoryAddress & _wma) {
	return getWorkingMemoryEntry<T>(std::string(_wma.m_id),std::string(_wma.m_subarchitecture));
      }

      template <class T>
      boost::shared_ptr< const CASTData<T> > getWorkingMemoryEntry(const std::string & _id,
								   const std::string & _subarch) {     
	return m_tester.getWorkingMemoryEntry<T>(_id,_subarch);
      }

      template <class T>
      void  getWorkingMemoryEntries(std::vector < boost::shared_ptr< const CASTData<T> > > & _results) {
	m_tester.getWorkingMemoryEntries(_results);
      }

      template <class T>
      void  getWorkingMemoryEntries(const std::string & _subarch,				  
				    const int & _count,
				    std::vector < boost::shared_ptr< const CASTData<T> > > & _results) {
	m_tester.getWorkingMemoryEntries(_subarch, _count, _results);
      }

      void addChangeFilter(const cdl::WorkingMemoryChangeFilter & _filter,
			   WorkingMemoryChangeReceiver * _pReceiver) {
	m_tester.addChangeFilter(_filter, _pReceiver);
      }

      void removeChangeFilter(const WorkingMemoryChangeReceiver * _pReceiver,
			      const cdl::ReceiverDeleteCondition & _condition = cdl::DO_NOT_DELETE_RECEIVER) {
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


      void lockProcess() {
	m_tester.lockProcess();
      }

      void unlockProcess() {
	m_tester.unlockProcess();
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
    virtual void configure(std::map<std::string,std::string> & _config);
    virtual void taskAdopted(const std::string &_taskID) {};
    virtual void taskRejected(const std::string &_taskID) {};

    void registerTest(std::string _id, boost::shared_ptr<AbstractTest> _test) throw(CASTException);
    void queueTest(std::string _id) throw(CASTException);


  public:
    AbstractTester(const std::string &_id);
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

