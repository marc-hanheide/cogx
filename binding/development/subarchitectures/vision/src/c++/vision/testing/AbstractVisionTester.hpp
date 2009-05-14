#ifndef ABSTRACT_VISION_TESTER_H
#define ABSTRACT_VISION_TESTER_H

#include <cast/core/CASTCore.hpp>
#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include "vision/idl/Vision.hh"

#include <queue>
#include <pthread.h>

using namespace boost;

namespace cast {

    class AbstractVisionTester : public PrivilegedManagedProcess,
				 public PullSender<Vision::ImageFrame>
{
 protected:

    ////////////////////////////////
    class AbstractVisionTest {
    public:
	AbstractVisionTest(AbstractVisionTester & _tester);
	virtual ~AbstractVisionTest();
	
	void run();
	bool waitFor();
	
    protected:
      friend  void * runTest( void * _ptr );
      
      virtual void startTest() = 0;
      void testComplete(bool _passed);    
      
      //make a bunch of common cast calls available
      std::string newDataID()  {
	return m_tester.newDataID();
      }

      void println(const std::string & _s) const {
	m_tester.println(_s);
      }

      void printfln(const char *format, ...) const {
	
	char msg[1024];
	va_list arg_list;
	va_start(arg_list, format);
	vsnprintf(msg, 1024, format, arg_list);
	va_end(arg_list);
	println(msg);
  
      }

      void log(const std::string & _s) const {
	m_tester.log(_s);
      }

      void logf(const char *format, ...) const {
	
	char msg[1024];
	va_list arg_list;
	va_start(arg_list, format);
	vsnprintf(msg, 1024, format, arg_list);
	va_end(arg_list);
	log(msg);
  
      }

      void sleepProcess(long _millis) {
	return m_tester.sleepProcess(_millis);
      }



      std::string getSubarchitectureID()  {
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


      void deleteFromWorkingMemory(const cdl::WorkingMemoryAddress & _wma,
				   const cdl::OperationMode & _sync = cdl::NON_BLOCKING) {    
	m_tester.deleteFromWorkingMemory(_wma ,_sync);
      }


      bool existsOnWorkingMemory(const cdl::WorkingMemoryAddress & _wma) {
	return m_tester.existsOnWorkingMemory(_wma);
      }
      

      
      template <class T> boost::shared_ptr< const CASTData<T> > 
      getWorkingMemoryEntry(const std::string & _id) {
	return getWorkingMemoryEntry<T>(_id,m_tester.m_subarchitectureID);
      }
      
      template <class T> boost::shared_ptr< const CASTData<T> > 
      getWorkingMemoryEntry(const cdl::WorkingMemoryAddress & _wma)  {
	return getWorkingMemoryEntry<T>(std::string(_wma.m_id),std::string(_wma.m_subarchitecture));
      }
      
      template <class T> boost::shared_ptr< const CASTData<T> > 
      getWorkingMemoryEntry(const std::string & _id, 
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

	/*
      template <class T> void  
      getWorkingMemoryEntries(const std::string & _type, 
			      const int & _count,
			      std::vector < boost::shared_ptr< const CASTData<T> > > & _results)
      {
	return m_tester.getWorkingMemoryEntries<T>(m_tester.m_subarchitectureID,_type,_count,_results);
      }
	*/


	/*
      void addChangeFilter(const std::string &_type, 
			   const cdl::WorkingMemoryOperation & _op,
			   const bool & _local,
			     WorkingMemoryChangeReceiver * _pReceiver) {
	m_tester.addChangeFilter(_type, _op, _local, _pReceiver);
      }
	*/
 
      void addChangeFilter(const cdl::WorkingMemoryChangeFilter & _filter,
			   WorkingMemoryChangeReceiver * _pReceiver) {
	  m_tester.addChangeFilter(_filter, _pReceiver);
      }
      

      void removeChangeFilter(WorkingMemoryChangeReceiver * _pReceiver) {
	m_tester.removeChangeFilter(_pReceiver);
      }


      int getFilterCount() const {
	return m_tester.getFilterCount();
      }


	AbstractVisionTester & m_tester;
      
    private:
      bool m_testComplete;			
      bool m_passed;
      pthread_mutex_t m_completionLock;	   
	//AbstractVisionTester & m_tester;
    };
    /////////////////////////////////////


    friend class AbstractVisionTest;
    friend void * runTest (void * _ptr);

    virtual void runComponent();
    virtual void configure(std::map<std::string,std::string> & _config);
    virtual void taskAdopted(const std::string &_taskID) {};
    virtual void taskRejected(const std::string &_taskID) {};

    void registerTest(std::string _id, 
		      boost::shared_ptr<AbstractVisionTest> _test) throw(CASTException);
    void queueTest(std::string _id) throw(CASTException);
    
    
 public:
    AbstractVisionTester(const std::string &_id);
    virtual ~AbstractVisionTester();
    
    typedef StringMap<boost::shared_ptr<AbstractVisionTest> >::map TestMap;
    
    PullConnectorOut<Vision::ImageFrame> *img_pull;
        
    virtual void setPullConnector(const std::string &_connectionID,
				  PullConnectorOut<Vision::ImageFrame> *gc);
    
    Vision::ImageFrame* GetImage(int camNum);

private:

    bool runTests();
    bool performTest(const boost::shared_ptr<AbstractVisionTest> & _test);
    
    TestMap m_tests;    
    std::queue<std::string> m_perform;
    
    bool m_exitOnCompletion;				  
    int m_successValue;
    int m_failureValue;
    pthread_t m_testThread;

};

}

#endif
