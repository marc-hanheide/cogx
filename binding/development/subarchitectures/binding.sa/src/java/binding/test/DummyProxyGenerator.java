package binding.test;

import java.util.Random;

import BindingData.FeaturePointer;
import BindingData.BindingProxy;
import BindingFeatures.Concept;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.core.CASTUtils;

/**
 * @author henrikj and maria, 
 */

public class DummyProxyGenerator extends ManagedProcess {
    // will be used to generate some random contents for the
    // progies
    private Random random;
    

    /**
     * @param _id
     */
    public DummyProxyGenerator(String _id) {
        super(_id);
	random = new Random();

        // m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
        // setReceiveXarchChangeNotifications(true);
		
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _goalID) {

    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) {

    }

    private String m_sceneObjectSubarchitecture = null;

 

    private
    BindingProxy  generateRandomProxy() throws SubarchitectureProcessException {
	BindingProxy ret = new BindingProxy();
	ret.m_proxyFeatures = new FeaturePointer[0]; // Corba does not initialize it...

	if(true) { //random.nextInt(2) == 0) {
	    Concept concept = new Concept();
	    concept.m_concept = new String("concept_" + random.nextInt(2));
	    
	    log(concept.m_concept);
	    
	    FeaturePointer feature = new FeaturePointer();
	    feature.m_type = CASTUtils.typeName(Concept.class);
	    feature.m_address = newDataID();
	    
	    log("feature.m_type: " + feature.m_type);
	    log("feature.m_address: " + feature.m_address);
	    addToWorkingMemory(feature.m_address.toString(),
			       //feature.m_type.toString(),
			       concept);
	    
	    FeaturePointer[] newFeats = new FeaturePointer[ret.m_proxyFeatures.length + 1];
	    
	    System.arraycopy(ret.m_proxyFeatures,0,newFeats,0,ret.m_proxyFeatures.length);
	    
	    newFeats[ret.m_proxyFeatures.length] = feature;
	    
	    ret.m_proxyFeatures = newFeats;
	    log(ret.m_proxyFeatures[0].m_address);
	}
	log("Proxy generated");
	return ret;
    }

    
    @Override
    protected void runComponent() {

	try {
	    Thread.sleep(2000);
	    
	    while(m_status == ProcessStatus.RUN) {
		
		lockProcess();

		//sleep for a random amount of time
		Thread.sleep(1000);
		// must check that we're still running after sleep
		if(m_status == ProcessStatus.RUN) {
		    log("generating a binding proxy");
		    
		    BindingProxy proxy = generateRandomProxy();  
		    
		    /*		    addToWorkingMemory(newDataID(),
		      BindingOntology.BINDING_PROXY_TYPE,
		      proxy);
		    */	    
		}
		unlockProcess();

	    }
	}
        catch (InterruptedException e) {
            e.printStackTrace();
        }
	catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	}
    }

    

}
