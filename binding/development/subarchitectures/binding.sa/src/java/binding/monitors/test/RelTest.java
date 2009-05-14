package binding.monitors.test;

import java.util.Properties;

import org.cognitivesystems.common.autogen.Math.Vector3D;

import BindingData.BinderStatus;
import BindingFeatures.Concept;
import BindingFeatures.Location;
import BindingFeaturesCommon.TemporalFrameType;
import binding.abstr.AbstractMonitor;
import binding.common.BindingComponentException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.data.CASTData;

public class RelTest extends AbstractMonitor {
    
    public RelTest(String _id) {
	super(_id);
    }
    
    @Override
	protected void taskAdopted(String _taskID) {
	
    }
    
    @Override
	protected void taskRejected(String _taskID) {
	
    }
    
    @Override
	public void start() {
	super.start();
	
    }
    
    private static enum MonitorMode {
	VISION, SPACE
	    };
    
    private MonitorMode m_mode;
    
    @Override
	public void configure(Properties _config) {
	super.configure(_config);
	
	m_sourceID = m_subarchitectureID;
	
	if (_config.containsKey("--vision")) {
	    m_mode = MonitorMode.VISION;
	}
	else if (_config.containsKey("--space")) {
	    m_mode = MonitorMode.SPACE;
	}
	else {
	    throw new RuntimeException("set monitor mode");
	}
    }
    
    @Override
	protected void runComponent() {
	
	try {
	    if (m_mode == MonitorMode.SPACE) {
		sleepProcess(1000);
		
		startNewBasicProxy();
		Location loc1 = new Location();
		loc1.m_location = new Vector3D(0.2f, 0.2f, 0.2f);
		addFeatureToCurrentProxy(loc1);
		String location1 = storeCurrentProxy();
		bindNewProxies();
		
		sleepProcess(5000);
		
		startNewBasicProxy();
		Location loc2 = new Location();
		loc2.m_location = new Vector3D(0.3f, 0.3f, 0.3f);
		addFeatureToCurrentProxy(loc2);
		String location2 = storeCurrentProxy();
		bindNewProxies();
		
		sleepProcess(2000);
		
		String rel1 = addSimpleRelation(location1, location2, "near", TemporalFrameType.PERCEIVED);
		String rel2 = addSimpleRelation(location2, location1, "near", TemporalFrameType.PERCEIVED);
		bindNewProxies();
		
		sleepProcess(3000);
		
		
		deleteExistingProxy(rel1);
		deleteExistingProxy(rel2);
		
		startNewBasicProxy();
		Location loc3 = new Location();
		loc3.m_location = new Vector3D(0.4f, 0.4f, 0.4f);
		addFeatureToCurrentProxy(loc3);
		String location3 = storeCurrentProxy();
		
		bindNewProxies();
		sleepProcess(1000);
		
		rel1 = addSimpleRelation(location1, location2, "near", TemporalFrameType.PERCEIVED);
		rel2 = addSimpleRelation(location2, location1, "near", TemporalFrameType.PERCEIVED);
		bindNewProxies();
		
		//just wait a while
		sleepProcess(3000);
		addToWorkingMemory(newDataID(), getBindingSA(),new BindingData.TriggerDotViewer());
		//just wait a while
		sleepProcess(3000);
		
		CASTData<BinderStatus>[] workingMemoryEntries = getWorkingMemoryEntries(getBindingSA(), BinderStatus.class);
		BinderStatus status = workingMemoryEntries[0].getData();
		
		if(status.m_unboundProxies > 0) {
		    System.exit(CAST_TEST_FAIL.value);
		}
		else {
		    System.exit(CAST_TEST_PASS.value);
		}
		
	    }
	    else if (m_mode == MonitorMode.VISION) {
		
		
		sleepProcess(1000);
		
		startNewBasicProxy();
		Concept con1 = new Concept();
		con1.m_concept = "thing";
		addFeatureToCurrentProxy(con1);
		String thing1 = storeCurrentProxy();
		startNewBasicProxy();
		Location loc1 = new Location();
		loc1.m_location = new Vector3D(0.2f, 0.2f, 0.2f);
		addFeatureToCurrentProxy(loc1);
		String location1 = storeCurrentProxy();
		addSimpleRelation(thing1, location1, "pos", TemporalFrameType.PERCEIVED);
		
		
		bindNewProxies();
		sleepProcess(5000);
		
		
		startNewBasicProxy();
		Concept con2 = new Concept();
		con1.m_concept = "thing";
		addFeatureToCurrentProxy(con1);
		String thing2 = storeCurrentProxy();
		startNewBasicProxy();
		Location loc2 = new Location();
		loc2.m_location = new Vector3D(0.3f, 0.3f, 0.3f);
		addFeatureToCurrentProxy(loc2);
		String location2 = storeCurrentProxy();
		addSimpleRelation(thing2, location2, "pos", TemporalFrameType.PERCEIVED);
		
		bindNewProxies();
		sleepProcess(5000);
		
		startNewBasicProxy();
		Concept con3 = new Concept();
		con1.m_concept = "thing";
		addFeatureToCurrentProxy(con1);
		String thing3 = storeCurrentProxy();
		startNewBasicProxy();
		Location loc3 = new Location();
		loc3.m_location = new Vector3D(0.4f, 0.4f, 0.4f);
		addFeatureToCurrentProxy(loc3);
		String location3 = storeCurrentProxy();
		addSimpleRelation(thing3, location3, "pos", TemporalFrameType.PERCEIVED);
		
		bindNewProxies();
		
	    }
	}
	catch (BindingComponentException e) {
	    e.printStackTrace();
	}
	catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	}
	
    }
    
}
