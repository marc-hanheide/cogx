/**
 * 
 */
package binding.monitors.test;

import java.util.*;


import binding.monitors.fakecomsys.FakeComSysBindingMonitor;
import cast.SubarchitectureComponentException;
import cast.cdl.testing.CASTTESTFAIL;
import cast.cdl.testing.CASTTESTPASS;
import binding.common.BindingComponentException;
import BindingData.BinderStatus;
import BindingData.BindingProxy;
import cast.cdl.*;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ChangeFilterFactory;
import cast.core.CASTUtils;
import cast.core.CASTData;

/**
 * @author henrikj
 */
public class FakeComSysTest extends FakeComSysBindingMonitor {

    /// true when all test sentences have been processed
    boolean testReady;
    int testNumber;

    public FakeComSysTest(String id) {
        super(id);
	testReady = false;
    }

    
    @Override
    public void configure(Map<String,String> config) {
        super.configure(config);
	try {
	    testNumber = Integer.parseInt(config.get("--test"));
	    assert(testNumber >= 0);
	} catch (NumberFormatException e) {
	    System.out.println("usage in FakeComSysTest: --test N where N is an integer. But the problem is now:" + e.getMessage());
	    System.exit((int)CASTTESTFAIL.value);
	}
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderComponent#start()
     */
    @Override
    public void start() {
        super.start();
	WorkingMemoryChangeReceiver bindingstatusreceiver =
	    new WorkingMemoryChangeReceiver() {
		public void workingMemoryChanged(WorkingMemoryChange wmc) {
		    bindingStatusUpdated(wmc);
		}
	    };
	try {
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(BinderStatus.class, WorkingMemoryOperation.OVERWRITE),
			    //BindingOntology.BINDERSTATUSTYPE,
			    //WorkingMemoryOperation.OVERWRITE,
			    //FilterRestriction.ALLSA, // not local
			    bindingstatusreceiver);
	}
	catch(Exception e) {
	    e.printStackTrace();
	    System.exit((int)CASTTESTFAIL.value);
	}
    }
    
    @Override
	public void runComponent() {
	sleepComponent(3000);
	BindingProxy prox = new BindingProxy();
	log("Typename of Proxies: " + CASTUtils.typeName(prox));

	try {
	    switch(testNumber) {
	    case 0:
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		testReady = true;
		break;
	    case 1:
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		testReady = true;
		break;
	    case 2:
		createAndStoreBindingProxiesFromPhonString("red balls");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("red ball");
		bindNewProxies();
		testReady = true;
		break;
	    case 3:
		createAndStoreBindingProxiesFromPhonString("red ball on blue box");
		bindNewProxies();
		testReady = true;
		break;
	    case 4:
		createAndStoreBindingProxiesFromPhonString("red ball on blue box");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("red balls on blue box");
		bindNewProxies();		
		testReady = true;
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
		testReady = true;
		break;
	    case 6:
		createAndStoreBindingProxiesFromPhonString("red pen on blue on green on big on yellow");
		bindNewProxies();
		createAndStoreBindingProxiesFromPhonString("pen on box on mug on ball on banana");
		bindNewProxies();		
		testReady = true;
		break;
	    case 7:
		createAndStoreBindingProxiesFromPhonString("red boxes");
		bindNewProxies();
		sleepComponent(1000);
		createAndStoreBindingProxiesFromPhonString("blue boxes");
		bindNewProxies();
		sleepComponent(1000);
		createAndStoreBindingProxiesFromPhonString("boxes");
		bindNewProxies();
		testReady = true;
		break;
	    case 8:
		createAndStoreBindingProxiesFromPhonString("red boxes");
		createAndStoreBindingProxiesFromPhonString("blue boxes");
		createAndStoreBindingProxiesFromPhonString("boxes");
		bindNewProxies();		
		testReady = true;
		break;
	    case 9:
		createAndStoreBindingProxiesFromPhonString("boxes");
		createAndStoreBindingProxiesFromPhonString("red box");
		createAndStoreBindingProxiesFromPhonString("blue box");
		bindNewProxies();		
		testReady = true;
		break;
	    case 10:
		createAndStoreBindingProxiesFromPhonString("boxes");
		createAndStoreBindingProxiesFromPhonString("red boxes");
		createAndStoreBindingProxiesFromPhonString("blue boxes");
		createAndStoreBindingProxiesFromPhonString("red box");
		createAndStoreBindingProxiesFromPhonString("blue box");
		bindNewProxies();
		testReady = true;
		break;
	    default:
		System.out.println("test number too large: " + testNumber);
		System.exit((int)CASTTESTFAIL.value);
	    }
	}
	catch(SubarchitectureComponentException e) {
	    e.printStackTrace();
	    System.exit((int)CASTTESTFAIL.value);
	}
    }
    
    protected void bindingStatusUpdated(WorkingMemoryChange wmc) {
    BinderStatus status2 = null;
    try{
	    CASTData<?> status = getWorkingMemoryEntry(wmc.address.id);
	    status2 = (BinderStatus) status.getData();
	}
	catch(SubarchitectureComponentException e) {
	    e.printStackTrace();
	    System.exit((int)CASTTESTFAIL.value);
	}

	if(testReady && status2.stable) {
	    // let the dotviewer do its work...
	    sleepComponent(5000);
	    System.exit((int)CASTTESTPASS.value);
	}
    }
}

