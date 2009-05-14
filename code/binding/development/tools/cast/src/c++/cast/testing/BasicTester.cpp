#include "BasicTester.hpp"


#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace boost;
using namespace cast::cdl;
using namespace cast::cdl::testing;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new cast::BasicTester(_id);
  }
}



namespace cast {

  void BasicTester::SingleComponentReadWriteTest::startTest() {
    
    try {
      string id(newDataID());      
      
      CASTTestStruct wrote;
      wrote.m_count = 0;
      wrote.m_change.m_operation = cdl::ADD;
      wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
      wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
      wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(getSubarchitectureID().c_str());
      wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());

      addToWorkingMemory(id, new CASTTestStruct(wrote), cdl::BLOCKING);
      
      shared_ptr< const CASTData<CASTTestStruct> > readData = getWorkingMemoryEntry<CASTTestStruct>(id);
      shared_ptr<const CASTTestStruct> read = readData->getData();
    

      //       cout<<wrote.m_change<<endl;
      //       cout<<read->m_change<<endl;
      //       if(wrote == (*read)) {
      // 	cout<<"passed"<<endl;
      //       }

      if(!(wrote == (*read))) {
	testComplete(false);
      }

      //add a few more
      addToWorkingMemory(newDataID(), new CASTTestStruct(wrote), cdl::BLOCKING);
      addToWorkingMemory(newDataID(), new CASTTestStruct(wrote), cdl::BLOCKING);
      addToWorkingMemory(newDataID(), new CASTTestStruct(wrote), cdl::BLOCKING);
      addToWorkingMemory(newDataID(), new CASTTestStruct(wrote), cdl::BLOCKING);
      
      vector< shared_ptr< const CASTData<CASTTestStruct> > > entries;
      getWorkingMemoryEntries(entries);

      if(entries.size() != 5) {
	testComplete(false);
      }

      for(vector< shared_ptr< const CASTData<CASTTestStruct> > >::iterator i = entries.begin();
	  i < entries.end(); ++i) {
	//	CASTTestStruct read
	if(!(wrote == *((*i)->getData()))) {
	  testComplete(false);
	}	
      }


      entries.clear();
      getWorkingMemoryEntries(getSubarchitectureID(), 4, entries);

      if(entries.size() != 4) {
	testComplete(false);
      }

      testComplete(true);


    } catch (CASTException &e) {
      cerr<<e.what()<<endl;
      testComplete(false);
    }
    
  }


  class RemoveMyself: public WorkingMemoryChangeReceiver {
    
  public:
    RemoveMyself(BasicTester::Receiver * _test, const string & _message) : m_test(_test), m_message(_message){};
    virtual ~RemoveMyself() {};
    virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc)  {
      cout<<m_message<<endl;
      m_test->removeChangeFilter(this, cdl::DELETE_RECEIVER);
    }
  private:
    BasicTester::Receiver * m_test;
    string m_message;
  };


  void BasicTester::Receiver::startTest() {
    
    try {
      string id(newDataID());

      log("# filters: %d",getFilterCount());     

      addChangeFilter(createOperationFilter(cdl::ADD), 
		      new RemoveMyself(this, "addChangeFilter: op cdl::LOCAL_SA"));

      log("# filters: %d",getFilterCount());

      addChangeFilter(createOperationFilter(cdl::ADD, cdl::ALL_SA), 
		      new RemoveMyself(this, "addChangeFilter: op cdl::ALL_SA"));      

      log("# filters: %d",getFilterCount());

      addChangeFilter(createOperationFilter(cdl::WILDCARD, cdl::ALL_SA), 
		      new RemoveMyself(this, "addChangeFilter: cdl::ALL_SA"));      

      log("# filters: %d",getFilterCount());
      
      addChangeFilter(createOperationFilter(cdl::WILDCARD), 
		      new RemoveMyself(this, "addChangeFilter: cdl::LOCAL_SA"));      
      
      log("# filters: %d",getFilterCount());
      
      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(), 
		      new RemoveMyself(this, "addChangeFilter: type cdl::LOCAL_SA"));      
		      
      log("# filters: %d",getFilterCount());
      
      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(),
		      new RemoveMyself(this, "addChangeFilter: type cdl::ALL_SA"));      
      
      log("# filters: %d",getFilterCount());

      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(cdl::ADD), 
		      new RemoveMyself(this, "addChangeFilter: type op cdl::LOCAL_SA"));            

      log("# filters: %d",getFilterCount());

      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(cdl::ADD), 
		      new RemoveMyself(this, "addChangeFilter: type op cdl::ALL_SA"));            
	    
      log("# filters: %d",getFilterCount());
      
      addChangeFilter(createIDFilter(id), 
		      new RemoveMyself(this, "addChangeFilter: id"));            
	    
      log("# filters: %d",getFilterCount());

      WorkingMemoryAddress wma;
      wma.m_id = CORBA::string_dup(id.c_str());
      wma.m_subarchitecture = CORBA::string_dup(getSubarchitectureID().c_str());
      
      addChangeFilter(createAddressFilter(wma), 
		      new RemoveMyself(this, "addChangeFilter: id"));            
	    
      log("# filters: %d",getFilterCount());

      addChangeFilter(createSourceFilter(getProcessIdentifier()), 
		      new RemoveMyself(this, "addChangeFilter: source untyped"));            
	              
      log("# filters: %d",getFilterCount());

      addChangeFilter(createSourceFilter<CASTTestStruct>(getProcessIdentifier()), 
		      new RemoveMyself(this, "addChangeFilter: source typed"));            
      
      log("# filters: %d",getFilterCount());


      //sanity check
//       if(24 != getFilterCount()) {
// 	testComplete(false);
//       }
      


      CASTTestStruct wrote;
      wrote.m_count = 0;
      wrote.m_change.m_operation = cdl::ADD;
      wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
      wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
      wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(getSubarchitectureID().c_str());
      wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());
      
      //unlock to allow changes to be received
      unlockProcess();

      addToWorkingMemory(id, new CASTTestStruct(wrote), cdl::BLOCKING);
      
      sleepProcess(1000);

      log("remaining filters: %d",getFilterCount());
      testComplete(getFilterCount() == 0);
      
      //relock to allow test to continue
      lockProcess();

    } catch (CASTException &e) {
      cerr<<e.what()<<endl;
      testComplete(false);
    }
    
  }

  void BasicTester::TwoComponentReadWriteTest::startTest() {
    
    try {
      
      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(cdl::ADD), this);

      //sleep a little bit to allow others to get their filters up
      sleepProcess(1000);


      string id(newDataID());
            
      m_wrote.m_count = 0;
      m_wrote.m_change.m_operation = cdl::ADD;
      m_wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
      m_wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
      m_wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(getSubarchitectureID().c_str());
      m_wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());

      addToWorkingMemory(id, new CASTTestStruct(m_wrote), cdl::BLOCKING);
      
      
    } catch (CASTException &e) {
      cerr<<e.what()<<endl;
      testComplete(false);
    }
    
  }

  void BasicTester::TwoComponentReadWriteTest::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {

    //only if it's not my change
    if(strcmp(_wmc.m_src,getProcessIdentifier().c_str()) != 0) {
      
      try {
	
	
	shared_ptr< const CASTData<CASTTestStruct> > readData = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address);
	shared_ptr<const CASTTestStruct> read = readData->getData();
	
	// 	cout<<m_wrote.m_change<<endl;
	// 	cout<<read->m_change<<endl;
	// 	if(m_wrote == (*read)) {
	// 	  cout<<"passed"<<endl;
	// 	}
	
	testComplete(m_wrote == (*read));
      } catch (CASTException &e) {
	cerr<<e.what()<<endl;
	testComplete(false);
      }
    }
    
  }
  

  void BasicTester::Copier::startTest() {
    
    try {
      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(cdl::ADD), this);            
    } catch (CASTException &e) {
      cerr<<e.what()<<endl;     
      testComplete(false);
    }
    
  }

  void BasicTester::Copier::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    //only if it's not my change
    if(strcmp(_wmc.m_src,getProcessIdentifier().c_str()) != 0) {
      
      try {	
	
	shared_ptr< const CASTData<CASTTestStruct> > readData = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address);
	shared_ptr<const CASTTestStruct> read = readData->getData();
	
	addToWorkingMemory(newDataID(), new CASTTestStruct(*read), cdl::BLOCKING);
	
	testComplete(true);

      } catch (CASTException &e) {
	cerr<<e.what()<<endl;
	testComplete(false);
      }
    }

  }

  void BasicTester::Writer::startTest() {
    //sleep a little bit to allow others to get their filters up
    sleepProcess(1000);
    
    BasicTester * tester = dynamic_cast<BasicTester* >(&m_tester);
    if(tester == NULL) {
      throw(CASTException(__HERE__, "Unable to cast BasicTester"));
    }
    string targetSubarch(tester->m_targetSubarch);


    for (int i = 0; i < m_count; i++) {

      try {	
	string id(newDataID());
      
	CASTTestStruct wrote;
	wrote.m_count = i;
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

    //wait for a while to let everyone else finish
    sleepProcess(3000);
    testComplete(true);

  }

  void BasicTester::Overwriter::startTest() {
    //sleep a little bit to allow others to get their filters up
    sleepProcess(1000);

    string id(newDataID());
      

    BasicTester * tester = dynamic_cast<BasicTester* >(&m_tester);
    if(tester == NULL) {
      throw(CASTException(__HERE__, "Unable to cast BasicTester"));
    }
    string targetSubarch(tester->m_targetSubarch);

    CASTTestStruct wrote;
    wrote.m_count = 0;
    wrote.m_change.m_operation = cdl::ADD;
    wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
    wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
    wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(targetSubarch.c_str());
    wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());

    try {	
      addToWorkingMemory(id, targetSubarch, new CASTTestStruct(wrote), cdl::BLOCKING);
    } 
    catch (const CASTException & e) {
      cout<<"exception: "<<e.what()<<endl;
      testComplete(false);
    }
    
    for (int i = 0; i < m_count; i++) {

      // sleep to allow others to interfere
      sleepProcess(200);
      try {

	// if safe, then read in before overwrite
	if (m_safe) {
	  //now this should prevent consistency errors
	  getWorkingMemoryEntry<CASTTestStruct>(id, targetSubarch);
	}

	wrote.m_count = i;

	try {
	  overwriteWorkingMemory(id, targetSubarch, new CASTTestStruct(wrote), cdl::BLOCKING);

	}
	catch (const ConsistencyException & e) {
	  if(m_safe) {
	    log("consistency check failed");
	    log(e.what());
	    testComplete(false);
	    return;
	  }
	}


      } 
      catch (const CASTException & e) {
	println("exception: %s",e.what());
	testComplete(false);
	return;
      }



	
    }
    testComplete(true);

  }


  void BasicTester::Replacer::startTest() {
    
    try {
      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(cdl::ADD), this);
    } catch (CASTException &e) {
      cerr<<e.what()<<endl;     
      testComplete(false);
    }
    
  }

  void BasicTester::Replacer::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    //only if it's not my change
    if(strcmp(_wmc.m_src,getProcessIdentifier().c_str()) != 0) {
      
      try {	
	
	shared_ptr< const CASTData<CASTTestStruct> > readData = getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address);

	string id(_wmc.m_address.m_id);
	CASTTestStruct wrote;
	wrote.m_count = 0;
	wrote.m_change.m_operation = cdl::ADD;
	wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
	wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
	wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(getSubarchitectureID().c_str());
	wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());
	
	overwriteWorkingMemory(id, new CASTTestStruct(wrote), cdl::BLOCKING); 
      } catch (CASTException &e) {
	cerr<<e.what()<<endl;
	testComplete(false);
      }
    }

  }

  void BasicTester::Deleter::startTest() {
    
    try {
      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(cdl::ADD), this);
      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(cdl::OVERWRITE), this);            
    } catch (CASTException &e) {
      cerr<<e.what()<<endl;     
      testComplete(false);
    }
    
  }

  void BasicTester::Deleter::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    //only if it's not my change
    if(strcmp(_wmc.m_src,getProcessIdentifier().c_str()) != 0) {
      
	
	cdl::WorkingMemoryAddress wma = _wmc.m_address;

	
	// get it to check the value
      try {	
	
	shared_ptr< const CASTData<CASTTestStruct> > readData = getWorkingMemoryEntry<CASTTestStruct>(wma);
	shared_ptr<const CASTTestStruct> read = readData->getData();
	
	try {	
	
	
	  log("deletion count: %d", read->m_count);
	  
	  if(read->m_count % m_modulo == 0) {
	    log("deleted on count: %d");
	    deleteFromWorkingMemory(wma, cdl::BLOCKING);       
	  }
	  
	} catch (CASTException &e) {
	  println(e.what());
	  testComplete(false);
	  assert(false);
	}
      } 
      catch (CASTException &e) {
	println("ignoring read exception: %s",e.what());	
      }

    }

  }

  
  void BasicTester::Counter::startTest() {
    
    try {
      addChangeFilter(createGlobalTypeFilter<CASTTestStruct>(cdl::ADD), this);        
    }
    catch (const CASTException & e) {
      cout<<"exception: "<<e.what()<<endl;
      testComplete(false);
    }
    
  }
  
  void BasicTester::Counter::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {

    try {
      //read int
      shared_ptr< const CASTData<CASTTestStruct> > readData 
	= getWorkingMemoryEntry<CASTTestStruct>(_wmc.m_address);
      shared_ptr<const CASTTestStruct> read 
	= readData->getData();
      
      string src(read->m_change.m_src);
      int count = read->m_count;
      
      //cout<<src<<" "<<count<<endl;

      CountMap::iterator i = m_counts.find(src);

      // if we've seen changes from here before
      if (i != m_counts.end()) {

	// get the existing count
	int currentCount = m_counts[src];

	// if this isn't true then we're out of step
	if (currentCount != (count - 1)) {
	  cout<<"failed on order"<<endl;
	  testComplete(false);
	}
	// if this is true then we've received all the ones we need
	else if (count == (m_expecting - 1)) {
	  m_counts.erase(i);
	}
	// else just store the count
	else {
	  m_counts[src] = count;
	}

      } else {
	// the first count should be 0
	if (count != 0) {
	  cout<<"failed on first"<<endl;
	  testComplete(false);
	}
	m_counts[src]=count;
      }
      
    } 
    catch (const CASTException & e) {
      cout<<"exception: "<<e.what()<<endl;
      testComplete(false);
    }

    // if we're empty, then we're complete!
    if (m_counts.size() == 0) {
      //just in case
      sleepProcess(10000);
      testComplete(true);
    }
    
  }









  void BasicTester::ReAdder::startTest() {
    
    sleepProcess(1000);

    try {      
      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(cdl::ADD), this);
      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(cdl::OVERWRITE), this);
      addChangeFilter(createLocalTypeFilter<CASTTestStruct>(cdl::DELETE), this);
    } catch (CASTException &e) {
      cerr<<e.what()<<endl;     
      testComplete(false);
      return;
    }

    
    string id(newDataID());
    m_wrote.m_count = 0;
    m_wrote.m_change.m_operation = cdl::ADD;
    m_wrote.m_change.m_src = CORBA::string_dup(getProcessIdentifier().c_str());
    m_wrote.m_change.m_address.m_id = CORBA::string_dup(id.c_str());
    m_wrote.m_change.m_address.m_subarchitecture = CORBA::string_dup(getSubarchitectureID().c_str());
    m_wrote.m_change.m_type = CORBA::string_dup(typeName<CASTTestStruct>().c_str());
    
    addToWorkingMemory(id,new CASTTestStruct(m_wrote));
    

  }

  void BasicTester::ReAdder::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    
    if(++m_wrote.m_count == 100) {
      testComplete(true);
      return;
    } 

    if(m_wrote.m_count % 5 == 2) {
      log("adding on %d", m_wrote.m_count);
      addToWorkingMemory(_wmc.m_address,new CASTTestStruct(m_wrote));
      lockEntry(_wmc.m_address,cdl::LOCKED_O);      
    }
    else if(m_wrote.m_count % 5 == 1) {
      log("nothing on %d", m_wrote.m_count);
    }
    else  {
      log("overwrite on %d", m_wrote.m_count);
      overwriteWorkingMemory(_wmc.m_address,new CASTTestStruct(m_wrote));
    }

    
  }


  BasicTester::BasicTester(const string &_id)
    : WorkingMemoryAttachedComponent(_id),
      AbstractTester(_id)  {
  }
  
  void BasicTester::configure(std::map<std::string,std::string> & _config) {

    shared_ptr<SingleComponentReadWriteTest> write(new SingleComponentReadWriteTest(*this));
    registerTest("single-write", write);
    shared_ptr<TwoComponentReadWriteTest> readWrite(new TwoComponentReadWriteTest(*this));
    registerTest("read-write", readWrite);
    shared_ptr<Copier> copy(new Copier(*this));
    registerTest("copy", copy);

    shared_ptr<Writer> write10(new Writer(*this, 10));
    registerTest("write-10", write10);
    shared_ptr<Counter> count10(new Counter(*this, 10));
    registerTest("count-10", count10);

    shared_ptr<Writer> write100(new Writer(*this, 100));
    registerTest("write-100", write100);
    shared_ptr<Counter> count100(new Counter(*this, 100));
    registerTest("count-100", count100);

    shared_ptr<Writer> write1000(new Writer(*this, 1000));
    registerTest("write-1000", write1000);
    shared_ptr<Counter> count1000(new Counter(*this, 1000));
    registerTest("count-1000", count1000);

    shared_ptr<Overwriter> overwrite10(new Overwriter(*this, 10, true));
    registerTest("overwrite", overwrite10);

    shared_ptr<Overwriter> unsafeOverwrite10(new Overwriter(*this, 10, false));
    registerTest("unsafe-overwrite", unsafeOverwrite10);

    shared_ptr<Replacer> replace(new Replacer(*this));
    registerTest("replace", replace);

    shared_ptr<Deleter> deleter(new Deleter(*this,1));
    registerTest("delete", deleter);

    shared_ptr<Deleter> deleter5(new Deleter(*this,5));
    registerTest("delete-5", deleter5);

    shared_ptr<ReAdder> readder(new ReAdder(*this));
    registerTest("readd", readder);

    shared_ptr<Receiver> receiver(new Receiver(*this));
    registerTest("receive", receiver);

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
