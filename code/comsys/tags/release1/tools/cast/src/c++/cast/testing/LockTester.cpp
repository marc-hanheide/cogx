#include "LockTester.hpp"

#include <ChangeFilterFactory.hpp>

using namespace std;
using namespace boost;
using namespace cast::cdl;
using namespace cast::cdl::testing;

extern "C" {
  cast::interfaces::CASTComponentPtr
  newComponent() {
    return new cast::LockTester();
  }
}



namespace cast {


  void LockTester::LockingWriter::startTest() {
    //sleep a little bit to allow others to get their filters up
    sleepComponent(2000);
    
    LockTester * tester = dynamic_cast<LockTester* >(&m_tester);
    if(tester == NULL) {
      throw(CASTException(exceptionMessage(__HERE__, "Unable to cast LockTester")));
    }
    string targetSubarch(tester->m_targetSubarch);
    string id(newDataID());
    

    addChangeFilter(createAddressFilter(id,targetSubarch, OVERWRITE), 
		    this);
    try {		
	
      CASTTestStructPtr wrote(new CASTTestStruct());
      wrote->count = 0;
      wrote->change.operation = cdl::ADD;
      wrote->change.src = getComponentID();
      wrote->change.address.id = id;
      wrote->change.address.subarchitecture  = targetSubarch;
      wrote->change.type = typeName<CASTTestStruct>();	
      addToWorkingMemory(id, targetSubarch, wrote);	
    } 
    catch (const CASTException & e) {
      cout<<"exception: "<<e.what()<<endl;
      testComplete(false);
    }

  }

  void LockTester::LockingWriter::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    try {
      //println(getPermissions(_wmc.address));

      println("reading: " + _wmc.address.id);
      CASTTestStructPtr cts = getMemoryEntry<CASTTestStruct>(_wmc.address);
      println("read: %s %d",_wmc.address.id.c_str(),cts->count);
      //println("read: %s",string(_wmc.address.id);

      if (cts->count == m_count) {
	println("deleting: " + _wmc.address.id);
	deleteFromWorkingMemory(_wmc.address);
	println("deleted: " + _wmc.address.id);
	
	removeChangeFilter(this);
	testComplete(true);
      } 
      else {
	log("unlocking: " + _wmc.address.id);
	unlockEntry(_wmc.address);
	log("unlocked: " + _wmc.address.id);
      }
      
    } 
    catch (const SubarchitectureComponentException & e) {
      //println(e);
      try {
	removeChangeFilter(this);
      } 
      catch (const SubarchitectureComponentException & e1) {
	println(e1.message);
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
      println(e.message);     
      testComplete(false);
    }
  }

  void LockTester::LockingOverwriter::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    // try {
    // println(CASTUtils.toString(getPermissions(_wmc.address)));
    // } catch (WMException e) {
    // e.printStackTrace();
    // }

    try {

      // lock the entry
      log("locking: %s",_wmc.address.id.c_str());
      lockEntry(_wmc.address, LOCKEDODR);
      log("locked: %s",_wmc.address.id.c_str());

      
      log("reading: %s",_wmc.address.id.c_str());
      CASTTestStructPtr cts = getMemoryEntry<CASTTestStruct>(_wmc.address);
      println("read: %s %d",_wmc.address.id.c_str(),cts->count);


      for (int i = 0; i < m_count; i++) {
	cts->count++;
	overwriteWorkingMemory<CASTTestStruct>(_wmc.address, cts);
	sleepComponent(100);
      }

      log("unlocking: " + _wmc.address.id);
      unlockEntry(_wmc.address);
      log("unlocked: " + _wmc.address.id);

      testComplete(true);
    } 
    catch (const SubarchitectureComponentException & e) {
      println(e.what());
      try {
	removeChangeFilter(this);
      } 
      catch (const SubarchitectureComponentException & e1) {
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
    // println(CASTUtils.toString(getPermissions(_wmc.address)));
    // } catch (WMException e) {
    // e.printStackTrace();
    // }

    try {

      // lock the entry
      log("try locking: %s",_wmc.address.id.c_str());
      while(!tryLockEntry(_wmc.address, LOCKEDODR)) {
	log("try locking: %s",_wmc.address.id.c_str());      
	sleepComponent(100);
      }
      log("locked: %s",_wmc.address.id.c_str());

      
      log("reading: %s",_wmc.address.id.c_str());
      CASTTestStructPtr cts = getMemoryEntry<CASTTestStruct>(_wmc.address);
      println("read: %s %d",_wmc.address.id.c_str(), cts->count);


      for (int i = 0; i < m_count; i++) {
	cts->count++;
	overwriteWorkingMemory<CASTTestStruct>(_wmc.address, cts);
	sleepComponent(100);
      }

      log("unlocking: " + _wmc.address.id);
      unlockEntry(_wmc.address);
      log("unlocked: " + _wmc.address.id);

      testComplete(true);
    } 
    catch (const SubarchitectureComponentException & e) {
      println(e.what());
      try {
	removeChangeFilter(this);
      } 
      catch (const SubarchitectureComponentException & e1) {
	println(e1.what());
      }
      testComplete(false);
      return;
    }
      
  }


  void LockTester::Locker::startTest() {
    
    sleepComponent(2000);	

    LockTester * tester = dynamic_cast<LockTester* >(&m_tester);
    if(tester == NULL) {
      throw(CASTException(exceptionMessage(__HERE__, "Unable to cast LockTester")));
    }
    string targetSubarch(tester->m_targetSubarch);
    string id(newDataID());
    

    try {			
      CASTTestStructPtr wrote(new CASTTestStruct());
      wrote->count = 0;
      wrote->change.operation = cdl::ADD;
      wrote->change.src = getComponentID();
      wrote->change.address.id = id;
      wrote->change.address.subarchitecture = targetSubarch;
      wrote->change.type = typeName<CASTTestStruct>();	
      addToWorkingMemory(id, targetSubarch, wrote);	
      lockEntry(id, targetSubarch, m_permissions);
      sleepComponent(3000);
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
    sleepComponent(500);

    try {
      if (m_op == cdl::OVERWRITE) {
	try {
	  CASTTestStructPtr cts
	    = getMemoryEntry<CASTTestStruct>(_wmc.address);
	  log("trying to overwrite");
	  overwriteWorkingMemory(_wmc.address, cts);
	  log("overwrite incorrectly allowed");
	  testComplete(false);
	}
	catch (const PermissionException & p) {
	  log("overwrite correctly prevented");
	  testComplete(true);
	}
      }
      else if (m_op == cdl::DELETE) {
	try {
	  CASTTestStructPtr cts
	    = getMemoryEntry<CASTTestStruct>(_wmc.address);
	  overwriteWorkingMemory(_wmc.address, cts);
	  testComplete(false);
	}
	catch (const PermissionException & p) {	
	  try {
	    deleteFromWorkingMemory(_wmc.address);
	    testComplete(false);
	  }
	  catch (const PermissionException & q) {	
	    testComplete(true);
	  }
	}
      }
      else if (m_op == cdl::GET) {
	try {
	  CASTTestStructPtr cts(new CASTTestStruct());
	  overwriteWorkingMemory(_wmc.address,
				 //warning, if this was actually written
				 //problems would occur as it's
				 //uninitialised
				 cts); 
	  log("overwrite incorrectly allowed");
	  testComplete(false);
	}
	catch (const PermissionException & p) {	
	  try {
	    deleteFromWorkingMemory(_wmc.address);
	    log("deletion incorrectly allowed");
	    testComplete(false);
	  }
	  catch (const PermissionException & q) {	
	    if (isReadable(std::string(_wmc.address.id),
			   std::string(_wmc.address.subarchitecture))) {
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
    catch (const SubarchitectureComponentException & e) {
      println(e.what());
      testComplete(false);
    }
  
  }

  void LockTester::configure(const std::map<std::string,std::string> & _config) {

    shared_ptr<LockingWriter> write10(new LockingWriter(*this, 10));
    registerTest("write-10", write10);

    shared_ptr<LockingWriter> write40(new LockingWriter(*this, 40));
    registerTest("write-40", write40);

    shared_ptr<LockingOverwriter> overwrite10(new LockingOverwriter(*this, 10));
    registerTest("overwrite-10", overwrite10);

    shared_ptr<TryLockingOverwriter> tryOverwrite10(new TryLockingOverwriter(*this, 10));
    registerTest("try-overwrite-10", tryOverwrite10);

    shared_ptr<Locker> lockO(new Locker(*this, cdl::LOCKEDO));
    registerTest("lock-o", lockO);
    shared_ptr<Locker> lockOD(new Locker(*this, cdl::LOCKEDOD));
    registerTest("lock-od", lockOD);
    shared_ptr<Locker> lockODR(new Locker(*this, cdl::LOCKEDODR));
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
