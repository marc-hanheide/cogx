// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

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


/**
 * Test 1: create a single proxy, and verify whether (1) an union has been produced for 
 * this proxy, and (2) whether this union is well-formed and contains all the information
 * from the proxy
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 15/09/2009)
 */

public class Test1 extends AbstractTester {
	
	// test number
	protected static int testNumber = 1;
	
	// task description
	protected static String task = "Correct production of a binding union for a single proxy";
	
	// pointer to the proxy inserted into the working memory
	static Proxy lastProxy;
	
	static HashMap<String, Union> unions = new HashMap<String, Union>();

	
	/**
	 * Initialise the test
	 */
	public Test1()
	{
		super(testNumber, task);
	}
	
	
	
	/**
	 * Perform the test: create a proxy, insert it into the working memory, wait for the union(s) 
	 * containing this proxy, and verify that the created union is well-formed and contain all
	 * necessary information
	 * 
	 * @return true if the test was successful, false otherwise
	 */
	
	public boolean performTest() {
		
		Proxy proxy = createProxy();
		Union union = getBindingUnionForProxy(proxy);
		
		boolean check = isBindingUnionCorrect(proxy, union);
		return check;
	}
	
	
	/**
	 * Create the proxy (simple object with 2 features, one with one feature value, the second one with two)
	 * 
	 * @return the created proxy
	 */
	
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

	/**
	 * Check if the union is correct and well-formed relative to the proxy
	 * 
	 * @param union the union
	 * @param proxy the proxy
	 * @return true if the union is correct, false otherwise
	 */
	protected boolean isBindingUnionCorrect(Proxy proxy, Union union) {
		
		// check if union is not null
		if (union != null) {
			
			// check if union contains only one proxy, which is the one inserted
			if ((union.includedProxies.length == 1) && 
					union.includedProxies[0] ==proxy) {
			
				// check whether the existence probability of the union > 1
				 if (union.probExists > 0.0f) {
					
					 // check whether the time stamp of the union is between the timestamp of the proxy and the current time
					 if (union.timeStamp.s >= proxy.timeStamp.s && union.timeStamp.s <=getCASTTime().s) {
						 
						 // check whether the number of features of the union == the number of features of the proxy
						 if (union.features != null && 
								 union.features.length == proxy.features.length) {
							
							 // loop on each feature
							 for (int i = 0 ; i < union.features.length ; i++) {
							 if (union.features[i].featlabel.equals(proxy.features[i].featlabel)) {
								 
								 // check whether the number of alternative values on the union == number 
								 // of alternative values on the proxy
								 if (union.features[i].alternativeValues != null && 
										 union.features[i].alternativeValues.length == 
											 proxy.features[i].alternativeValues.length) {
									 
									 // loop on the feature values
									 for (int j = 0; j < union.features[i].alternativeValues.length ; j++) {
										 
										 // check whether the type of the feature values are the same in the proxy and the union
										 if (union.features[i].alternativeValues[j].getClass().equals(
												 proxy.features[i].alternativeValues[j].getClass())) {
											 
											 if (!FeatureValueUtils.haveEqualValue(
													 union.features[i].alternativeValues[j], proxy.features[i].alternativeValues[j])) {
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
												 proxy.features[i].alternativeValues.length +
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
	
	
	/**
	 * Get the binding union for the proxy
	 * 
	 * @param proxy the proxy
	 * @return the union
	 */
	
	protected Union getBindingUnionForProxy (Proxy proxy) {
		try {
			
			activateChangeFilter(proxy);
			addProxyToWM (proxy);

			while (!unions.containsKey(proxy.entityID)) {
				sleepComponent(20);
			}
			log("Predicted union for proxy is sucessfully retrieved");
			
			return unions.get(proxy.entityID);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}


	/**
	 * Activate a change filter for the proxy, monitoring binding unions containing it.
	 * When such union is found, it is added to the unions vector
	 * 
	 * @param proxy
	 */
	private void activateChangeFilter (Proxy proxy) {

		lastProxy = proxy;
		
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
								unions.put(lastProxy.entityID, u);
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
