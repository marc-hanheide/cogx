package binding.test;

import java.util.Random;

import BindingData.FeaturePointer;
import BindingData.BindingProxy;
import BindingFeatures.Concept;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.core.CASTUtils;

/**
 * @author henrikj and maria, 
 */

public class DummyProxyGenerator extends ManagedComponent {
    // will be used to generate some random contents for the
    // progies
    private Random random;
    

    /**
     * @param _id
     */
    public DummyProxyGenerator(String _id) {
        super();
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
    BindingProxy  generateRandomProxy() throws SubarchitectureComponentException {
	BindingProxy ret = new BindingProxy();
	ret.proxyFeatures = new FeaturePointer[0]; // Corba does not initialize it...

	if(true) { //random.nextInt(2) == 0) {
	    Concept concept = new Concept();
	    concept.conceptstr = new String("concept_" + random.nextInt(2));
	    
	    log(concept.conceptstr);
	    
	    FeaturePointer feature = new FeaturePointer();
	 //   feature.types = CASTUtils.typeName(concept);
	    feature.address = newDataID();
	    
	//    log("feature.type: " + feature.type);
	    log("feature.address: " + feature.address);
	    addToWorkingMemory(feature.address.toString(),
			       //feature.type.toString(),
			       concept);
	    
	    FeaturePointer[] newFeats = new FeaturePointer[ret.proxyFeatures.length + 1];
	    
	    System.arraycopy(ret.proxyFeatures,0,newFeats,0,ret.proxyFeatures.length);
	    
	    newFeats[ret.proxyFeatures.length] = feature;
	    
	    ret.proxyFeatures = newFeats;
	    log(ret.proxyFeatures[0].address);
	}
	log("Proxy generated");
	return ret;
    }

    
    @Override
    protected void runComponent() {

    	while (isRunning()) {

			try {
				// must check we're still running after sleep!
				if (isRunning()) {

					// lock from external access
					lockComponent();

		//sleep for a random amount of time
		Thread.sleep(1000);
		// must check that we're still running after sleep
		if(isRunning()) {
		    log("generating a binding proxy");
		    
		    BindingProxy proxy = generateRandomProxy();  
		    
		    /*		    addToWorkingMemory(newDataID(),
		      BindingOntology.BINDING_PROXY_TYPE,
		      proxy);
		    */	    
		}
		unlockComponent();

	    }
	}
        catch (InterruptedException e) {
            e.printStackTrace();
        }
	catch (SubarchitectureComponentException e) {
	    e.printStackTrace();
	}
    }
    }
    

}
