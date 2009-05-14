#include "LockTester.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace boost;
using namespace cast::cdl;
using namespace cast::cdl::testing;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new cast::LockTester(_id);
  }
}



namespace cast {


  void LockTester::LockingWriter::startTest() {
    //sleep a little bit to allow others to get their filters up
    sleepProcess(2000);
    
    LockTester * tester = dynamic_cast<LockTester* >(&m_tester);
    if(tester == NULL) {
      throw(CASTException(__HERE__, "Unable to cast LockTester"));
    }
    string targetSubarch(tester->m_targetSubarch);
    string id(newDataID());
    

    addChangeFilter(createAddressFilter(id,targetSubarch, OVERWRITE), 
		    this);
    try {		
	
      CASTTestStruct wrote;
      wrote.m_count = 0;
      wrote.m_change.m_operation = cdl::ADD;
      wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
      wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
      wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(targetSubarch.c_str());
      wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());	
      addToWorkingMemory(id, targetSubarch, new CASTTestStruct(wrote), cdl::BLOCKING);	
    } 
    catch (const CASTException & e) {
      cout<<"exception: "<<e.what()<<endl;
      testComplete(false);
    }

  }

  void LockTester::LockingWriter::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    try {
      //println(getPermissions(_wmc.m_address));

      println("reading: %s",string(_wmc.m_address.m_id).c_str());
      shared_ptr<const CASTTestStruct> cts = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address)->getData();
      println("read: %s %d",string(_wmc.m_address.m_id).c_str(),cts->m_count);
      //println("read: %s",string(_wmc.m_address.m_id).c_str());

      if (cts->m_count == m_count) {
	println("deleting: " + string(_wmc.m_address.m_id));
	deleteFromWorkingMemory(_wmc.m_address, BLOCKING);
	println("deleted: " + string(_wmc.m_address.m_id));
	
	removeChangeFilter(this);
	testComplete(true);
      } 
      else {
	log("unlocking: " + string(_wmc.m_address.m_id));
	unlockEntry(_wmc.m_address);
	log("unlocked: " + string(_wmc.m_address.m_id));
      }
      
    } 
    catch (const SubarchitectureProcessException & e) {
      //println(e);
      try {
	removeChangeFilter(this);
      } 
      catch (const SubarchitectureProcessException & e1) {
	println(e1.what());
      }
      testComplete(false);
    }
  }


  void LockTester::LockingOverwriter::startTest() {
    try {
      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(cdl::ADD), 
		      this);            
    } 
    catch (CASTException &e) {
      println(e.what());     
      testComplete(false);
    }
  }

  void LockTester::LockingOverwriter::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    // try {
    // println(CASTUtils.toString(getPermissions(_wmc.m_address)));
    // } catch (WMException e) {
    // e.printStackTrace();
    // }

    try {

      // lock the entry
      log("locking: %s",string(_wmc.m_address.m_id).c_str());
      lockEntry(_wmc.m_address, LOCKED_ODR);
      log("locked: %s",string(_wmc.m_address.m_id).c_str());

      
      log("reading: %s",string(_wmc.m_address.m_id).c_str());
      shared_ptr<const CASTTestStruct> ctsData = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address)->getData();
      //create a local copy to increment
      CASTTestStruct cts(*ctsData.get());
      println("read: %s %d",string(_wmc.m_address.m_id).c_str(),cts.m_count);


      for (int i = 0; i < m_count; i++) {
	cts.m_count++;
	CASTTestStruct * newCTS = new CASTTestStruct(cts);	
	overwriteWorkingMemory<CASTTestStruct>(_wmc.m_address, newCTS);
	sleepProcess(100);
      }

      log("unlocking: " + string(_wmc.m_address.m_id));
      unlockEntry(_wmc.m_address);
      log("unlocked: " + string(_wmc.m_address.m_id));

      testComplete(true);
    } 
    catch (const SubarchitectureProcessException & e) {
      println(e.what());
      try {
	removeChangeFilter(this);
      } 
      catch (const SubarchitectureProcessException & e1) {
	println(e1.what());
      }
      testComplete(false);
      return;
    }
      
  }

  void LockTester::TryLockingOverwriter::startTest() {
    try {
      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(cdl::ADD), 
		      this);            
    } 
    catch (CASTException &e) {
      println(e.what());     
      testComplete(false);
    }
  }

  void LockTester::TryLockingOverwriter::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    // try {
    // println(CASTUtils.toString(getPermissions(_wmc.m_address)));
    // } catch (WMException e) {
    // e.printStackTrace();
    // }

    try {

      // lock the entry
      log("try locking: %s",string(_wmc.m_address.m_id).c_str());
      while(!tryLockEntry(_wmc.m_address, LOCKED_ODR)) {
	log("try locking: %s",string(_wmc.m_address.m_id).c_str());      
	sleepProcess(100);
      }
      log("locked: %s",string(_wmc.m_address.m_id).c_str());

      
      log("reading: %s",string(_wmc.m_address.m_id).c_str());
      shared_ptr<const CASTTestStruct> ctsData = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address)->getData();
      //create a local copy to increment
      CASTTestStruct cts(*ctsData.get());
      println("read: %s %d",string(_wmc.m_address.m_id).c_str(),cts.m_count);


      for (int i = 0; i < m_count; i++) {
	cts.m_count++;
	CASTTestStruct * newCTS = new CASTTestStruct(cts);	
	overwriteWorkingMemory<CASTTestStruct>(_wmc.m_address, newCTS);
	sleepProcess(100);
      }

      log("unlocking: " + string(_wmc.m_address.m_id));
      unlockEntry(_wmc.m_address);
      log("unlocked: " + string(_wmc.m_address.m_id));

      testComplete(true);
    } 
    catch (const SubarchitectureProcessException & e) {
      println(e.what());
      try {
	removeChangeFilter(this);
      } 
      catch (const SubarchitectureProcessException & e1) {
	println(e1.what());
      }
      testComplete(false);
      return;
    }
      
  }


  void LockTester::Locker::startTest() {
    
    sleepProcess(2000);	

    LockTester * tester = dynamic_cast<LockTester* >(&m_tester);
    if(tester == NULL) {
      throw(CASTException(__HERE__, "Unable to cast LockTester"));
    }
    string targetSubarch(tester->m_targetSubarch);
    string id(newDataID());
    

    try {			
      CASTTestStruct wrote;
      wrote.m_count = 0;
      wrote.m_change.m_operation = cdl::ADD;
      wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
      wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
      wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(targetSubarch.c_str());
      wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());	
      addToWorkingMemory(id, targetSubarch, new CASTTestStruct(wrote));	
      lockEntry(id, targetSubarch, m_permissions);
      sleepProcess(3000);
      unlockEntry(id, targetSubarch);
      testComplete(true);
    } 
    catch (const CASTException & e) {
      cout<<"exception: "<<e.what()<<endl;
      testComplete(false);
    }

  }


  void LockTester::Sneaker::startTest() {
    try {
      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(cdl::ADD), 
		      this);            
    } 
    catch (CASTException &e) {
      println(e.what());     
      testComplete(false);
    }
  }
  
  void LockTester::Sneaker::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {

    // need to wait a little while
    sleepProcess(500);

    try {
      if (m_op == cdl::OVERWRITE) {
	try {
	  shared_ptr<const CASTTestStruct> ctsData 
	    = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address)->getData();
	  CASTTestStruct cts(*ctsData.get());
		  log("trying to overwrite");
	  overwriteWorkingMemory(_wmc.m_address, 
				 new CASTTestStruct(cts));
	  log("overwrite incorrectly allowed");
	  testComplete(false);
	}
	catch (const PermissionException & p) {
	  testComplete(true);
	}
      }
      else if (m_op == cdl::DELETE) {
	try {
	  shared_ptr<const CASTTestStruct> ctsData 
	    = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address)->getData();
	  CASTTestStruct cts(*ctsData.get());		
	  overwriteWorkingMemory(_wmc.m_address, new CASTTestStruct(cts));
	  testComplete(false);
	}
	catch (const PermissionException & p) {	
	  try {
	    deleteFromWorkingMemory(_wmc.m_address);
	    testComplete(false);
	  }
	  catch (const PermissionException & q) {	
	    testComplete(true);
	  }
	}
      }
      else if (m_op == cdl::GET) {
	try {
	  overwriteWorkingMemory(_wmc.m_address,
				 //warning, if this was actually written
				 //problems would occur as it's
				 //uninitialised
				 new CASTTestStruct()); 
	  log("overwrite incorrectly allowed");
	  testComplete(false);
	}
	catch (const PermissionException & p) {	
	  try {
	    deleteFromWorkingMemory(_wmc.m_address);
	    log("deletion incorrectly allowed");
	    testComplete(false);
	  }
	  catch (const PermissionException & q) {	
	    if (isReadable(std::string(_wmc.m_address.m_id),
			   std::string(_wmc.m_address.m_subarchitecture))) {
	      log("read incorrectly allowed");
	      testComplete(false);
	    }
	    else {
	      testComplete(true);
	    }
	  }
	}
      }
      else {
	assert (true);
      }
    }
    catch (const SubarchitectureProcessException & e) {
      println(e.what());
      testComplete(false);
    }
  
  }


  LockTester::LockTester(const string &_id)
    : WorkingMemoryAttachedComponent(_id),
      AbstractTester(_id)  {
  }
  
  void LockTester::configure(std::map<std::string,std::string> & _config) {

    shared_ptr<LockingWriter> write10(new LockingWriter(*this, 10));
    registerTest("write-10", write10);

    shared_ptr<LockingWriter> write40(new LockingWriter(*this, 40));
    registerTest("write-40", write40);

    shared_ptr<LockingOverwriter> overwrite10(new LockingOverwriter(*this, 10));
    registerTest("overwrite-10", overwrite10);

    shared_ptr<TryLockingOverwriter> tryOverwrite10(new TryLockingOverwriter(*this, 10));
    registerTest("try-overwrite-10", tryOverwrite10);

    shared_ptr<Locker> lockO(new Locker(*this, cdl::LOCKED_O));
    registerTest("lock-o", lockO);
    shared_ptr<Locker> lockOD(new Locker(*this, cdl::LOCKED_OD));
    registerTest("lock-od", lockOD);
    shared_ptr<Locker> lockODR(new Locker(*this, cdl::LOCKED_ODR));
    registerTest("lock-odr", lockODR);

    shared_ptr<Sneaker> sneakO(new Sneaker(*this, cdl::OVERWRITE));
    registerTest("sneak-o", sneakO);
    shared_ptr<Sneaker> sneakOD(new Sneaker(*this, cdl::DELETE));
    registerTest("sneak-d", sneakOD);
    shared_ptr<Sneaker> sneakODR(new Sneaker(*this, cdl::GET));
    registerTest("sneak-r", sneakODR);
    


    AbstractTester::configure(_config);    

    map<string,string>::const_iterator i = _config.find("--subarch");
    if(i != _config.end()) {
      m_targetSubarch = i->second;
    }
    else {
      m_targetSubarch = m_subarchitectureID;
    }
  }

};
