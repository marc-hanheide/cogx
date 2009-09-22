package binder.tests;

import java.util.HashMap;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.utils.FeatureValueUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;

public class Test1 extends AbstractTester {
	
	protected static int testNumber = 1;
	protected static String task = "Correct production of a binding union for a single proxy";
	
	
	static Proxy lastProxy = new Proxy();
	static HashMap<String, Union> predictedUnions = new HashMap<String, Union>();

	
	public Test1()
	{
		super(testNumber, task);
	}
	

	public boolean performTest() {
		
		Proxy proxy = createProxy();
		Union union = getBindingUnionForProxy(proxy);
		
		boolean check = isBindingUnionCorrect(union);
		return check;
	}
	
	
	protected Proxy createProxy() {
		WorkingMemoryPointer origin = createWorkingMemoryPointer("subarch1", "localDataID", "dataType");
		Proxy newProxy = createNewProxy(origin, 0.8f);
		
		FeatureValue featvalue1a = createStringValue("featvalue1", 0.7f);
		Feature feat1 = createFeatureWithUniqueFeatureValue("featlabel1", featvalue1a);
		addFeatureToProxy(newProxy, feat1);

		FeatureValue featvalue2a = createIntegerValue(3, 0.7f);
		FeatureValue featvalue2b = createIntegerValue(4, 0.2f);
		Feature feat2 = createFeature("featlabel2");
		addFeatureValueToFeature(feat2, featvalue2a);
		addFeatureValueToFeature(feat2, featvalue2b);
		
		addFeatureToProxy(newProxy, feat2);

		return newProxy;
	}

	protected boolean isBindingUnionCorrect(Union union) {
		if (union != null) {
			if ((union.includedProxies.length == 1) && 
					union.includedProxies[0] ==lastProxy) {
			
				 if (union.probExists > 0.0f) {
					
					 if (union.timeStamp < System.currentTimeMillis()) {
						 
						 if (union.features != null && 
								 union.features.length == lastProxy.features.length) {
							
							 for (int i = 0 ; i < union.features.length ; i++) {
							 if (union.features[i].featlabel.equals(lastProxy.features[i].featlabel)) {
								 
								 if (union.features[i].alternativeValues != null && 
										 union.features[i].alternativeValues.length == 
											 lastProxy.features[i].alternativeValues.length) {
									 for (int j = 0; j < union.features[i].alternativeValues.length ; j++) {
										 
										 if (union.features[i].alternativeValues[j].getClass().equals(
												 lastProxy.features[i].alternativeValues[j].getClass())) {
											 
											 if (!FeatureValueUtils.haveEqualValue(union.features[i].alternativeValues[j], lastProxy.features[i].alternativeValues[j])) {
													 log("FAILED, feature values in the proxy and the union are different");
											 }
										 }
										 else {
											 log("FAILED, feature value subclasses are not similar");
											 return false;
										 }
											 
									 }
								 }
								 else {
									 log("FAILED, problem with the feature values of feature " + i);
									 if (union.features[i].alternativeValues != null) {
										 log("number of feature values in proxy: " + 
												 lastProxy.features[i].alternativeValues.length +
												 " != number of feature values in union: " + 
												 union.features[i].alternativeValues.length );	 
									 }
									 else {
										 log("alternative feature values is null");
									 }
									 return false;
								 }
							 }
							 else {
								 log("FAILED, feature labels do not match");
							 }
							 }
						 }
						 
						 else {
							 log("FAILED, wrong number of features in proxy");
						 }
						 
							if ((union.entityID == null)) {
								log("FAILED, entity ID is null");
								return false;
							}
							
					 }
					 else {
						 log("FAILED, union timestamp not correctly specified");
						 return false;
					 }
					 
				
				 }
				 else {
					 log("FAILED, probExists is not > 0");
					 return false;
				 }
			}
			else {
				log("FAILED, wrong number of included proxies in union");
				return false;
			}
		}
		else {
			log("FAILED, union is null");
			return false;
		}
		
		// Passed all tests!
		return true;
	}
	
	protected Union getBindingUnionForProxy (Proxy proxy) {
		try {
			
			lastProxy = proxy;
			activateChangeFilter(proxy);
			addProxyToWM (proxy);

			while (!predictedUnions.containsKey(proxy.entityID)) {
				sleepComponent(20);
			}
			log("Predicted union for proxy is sucessfully retrieved");
			
			return predictedUnions.get(proxy.entityID);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}



	private void activateChangeFilter (Proxy proxy) {


		WorkingMemoryChangeReceiver receiverForPhantomProxies = 
			new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					UnionConfiguration config = 
						getMemoryEntry(_wmc.address, UnionConfiguration.class);
					for (int i = 0 ; i < config.includedUnions.length ; i++) {
						Union u = config.includedUnions[i];
						for (int j = 0 ; j < u.includedProxies.length ; j++) {
							if (u.includedProxies[j].equals(lastProxy)) {
								predictedUnions.put(lastProxy.entityID, u);
							}
						}
					}
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		};
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), receiverForPhantomProxies);
	}

}
