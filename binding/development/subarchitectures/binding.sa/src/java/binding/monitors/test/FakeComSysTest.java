/**
 * 
 */
package binding.monitors.test;

import java.util.*;


import binding.monitors.fakecomsys.FakeComSysBindingMonitor;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import binding.common.BindingComponentException;
import BindingData.BinderStatus;
import BindingData.BindingProxy;
import cast.cdl.*;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

/**
 * @author henrikj
 */
public class FakeComSysTest extends FakeComSysBindingMonitor {

    /// true when all test sentences have been processed
    boolean m_testReady;
    int m_testNumber;

    public FakeComSysTest(String _id) {
        super(_id);
	m_testReady = false;
    }

    
    @Override
    public void configure(Properties _config) {
        super.configure(_config);
	try {
	    m_testNumber = Integer.parseInt(_config.getProperty("--test"));
	    assert(m_testNumber >= 0);
	} catch (NumberFormatException e) {
	    System.out.println("usage in FakeComSysTest: --test N where N is an integer. But the problem is now:" + e.getMessage());
	    System.exit(CAST_TEST_FAIL.value);
	}
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();
	WorkingMemoryChangeReceiver binding_status_receiver =
	    new WorkingMemoryChangeReceiver() {
		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
		    bindingStatusUpdated(_wmc);
		}
	    };
	try {
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(BinderStatus.class, WorkingMemoryOperation.OVERWRITE),
			    //BindingOntology.BINDER_STATUS_TYPE,
			    //WorkingMemoryOperation.OVERWRITE,
			    //FilterRestriction.ALL_SA, // not local
			    binding_status_receiver);
	}
	catch(SubarchitectureProcessException e) {
	    e.printStackTrace();
	    System.exit(CAST_TEST_FAIL.value);
	}
    }
    
    @Override
	public void runComponent() {
	sleepProcess(3000);
	BindingProxy prox = new BindingProxy();
	log("Typename of Proxies: " + CASTUtils.typeName(prox));

	try {
	    switch(m_testNumber) {
	    case 0:
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		m_testReady = true;
		break;
	    case 1:
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		m_testReady = true;
		break;
	    case 2:
		createAndStoreBindingProxiesFromPhonString("red balls");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		m_testReady = true;
		break;
	    case 3:
		createAndStoreBindingProxiesFromPhonString("red ball on blue box");
		bindNewProxies();
		m_testReady = true;
		break;
	    case 4:
		createAndStoreBindingProxiesFromPhonString("red ball on blue box");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("red balls on blue box");
		bindNewProxies();		
		m_testReady = true;
		break;
	    case 5:
		createAndStoreBindingProxiesFromPhonString("red balls on blue boxes");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();		
		createAndStoreBindingProxiesFromPhonString("box");
		bindNewProxies();		
		createAndStoreBindingProxiesFromPhonString("boxes");
		bindNewProxies();		
		createAndStoreBindingProxiesFromPhonString("big pen");
		bindNewProxies();		
		m_testReady = true;
		break;
	    case 6:
		createAndStoreBindingProxiesFromPhonString("red pen on blue on green on big on yellow");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("pen on box on mug on ball on banana");
		bindNewProxies();		
		m_testReady = true;
		break;
	    case 7:
		createAndStoreBindingProxiesFromPhonString("red boxes");
		bindNewProxies();
		sleepProcess(1000);
		createAndStoreBindingProxiesFromPhonString("blue boxes");
		bindNewProxies();
		sleepProcess(1000);
		createAndStoreBindingProxiesFromPhonString("boxes");
		bindNewProxies();
		m_testReady = true;
		break;
	    case 8:
		createAndStoreBindingProxiesFromPhonString("red boxes");
		createAndStoreBindingProxiesFromPhonString("blue boxes");
		createAndStoreBindingProxiesFromPhonString("boxes");
		bindNewProxies();		
		m_testReady = true;
		break;
	    case 9:
		createAndStoreBindingProxiesFromPhonString("boxes");
		createAndStoreBindingProxiesFromPhonString("red box");
		createAndStoreBindingProxiesFromPhonString("blue box");
		bindNewProxies();		
		m_testReady = true;
		break;
	    case 10:
		createAndStoreBindingProxiesFromPhonString("boxes");
		createAndStoreBindingProxiesFromPhonString("red boxes");
		createAndStoreBindingProxiesFromPhonString("blue boxes");
		createAndStoreBindingProxiesFromPhonString("red box");
		createAndStoreBindingProxiesFromPhonString("blue box");
		bindNewProxies();
		m_testReady = true;
		break;
	    default:
		System.out.println("test number too large: " + m_testNumber);
		System.exit(CAST_TEST_FAIL.value);
	    }
	}
	catch(SubarchitectureProcessException e) {
	    e.printStackTrace();
	    System.exit(CAST_TEST_FAIL.value);
	}
    }
    
    protected void bindingStatusUpdated(WorkingMemoryChange _wmc) {
	BinderStatus status = null;
	try{
	    CASTData<?> _status = getWorkingMemoryEntry(_wmc.m_address.m_id);
	    status = (BinderStatus) _status.getData();
	}
	catch(SubarchitectureProcessException e) {
	    e.printStackTrace();
	    System.exit(CAST_TEST_FAIL.value);
	}

	if(m_testReady && status.m_stable) {
	    // let the dotviewer do its work...
	    sleepProcess(5000);
	    System.exit(CAST_TEST_PASS.value);
	}
    }

    /**
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _goalID) {    }

    /**
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) { }

}
