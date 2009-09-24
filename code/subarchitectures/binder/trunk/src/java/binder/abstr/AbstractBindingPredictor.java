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

package binder.abstr;

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.specialentities.PhantomProxy;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;

/**
 * Abstract class for inserting phantom proxies onto the binder working memory,
 * and retrieving (ranked) lists of predicted binding unions for these proxies,
 * based on the Bayesian network
 * 
 * @author Pierre Lison
 * @version 22/09/2009 (tarted 10/09/2009)
 */

public class AbstractBindingPredictor extends BindingWorkingMemoryWriter {

	//Last phantom proxy inserted onto the binder WM
	static PhantomProxy lastPhantomProxy = new PhantomProxy();
	
	// Change receiver for phantom proxies 
	static WorkingMemoryChangeReceiver receiverForPhantomProxies;	
	
	// List of predicted unions for lastPhantomProxy
	static Vector<Union> predictedUnions = new Vector<Union>();


	// =================================================================
	// METHODS FOR CREATING NEW PHANTOM PROXIES
	// =================================================================
	

	/**
	 * Create a new phantom proxy, given the pointer to the originating object
	 * and the existence probability
	 * 
	 * @param origin
	 *            origin information
	 * @param probExists
	 *            the probability of the proxy
	 * @return the new phantom
	 */
	public PhantomProxy createNewPhantomProxy(WorkingMemoryPointer origin,
			float probExists) {

		PhantomProxy newProxy = new PhantomProxy();

		newProxy.entityID = newDataID();
		newProxy.origin = origin;
		newProxy.probExists = probExists;
		newProxy.features = new Feature[0];

		return newProxy;
	}

	

	/**
	 * Create a new phantom proxy, given a pointer to the data in the local working 
	 * memory, its existence probability, and a list of features
	 * 
	 * @param origin
	 *            origin information
	 * @param probExists
	 *            the probability of the proxy
	 * @return the new phantom proxy
	 */
	
	public PhantomProxy createNewPhantomProxy(WorkingMemoryPointer origin,
			float probExists, Feature[] features) {

		PhantomProxy newProxy = createNewPhantomProxy(origin, probExists);
		newProxy.features = features;

		return newProxy;
	}
	
	
	
	// =================================================================
	// METHODS FOR PREDICTIONS POSSIBLE UNIONS FOR PHANTOM PROXIES
	// =================================================================
	
	
	
	/**
	 * Get a list of predicted possible unions for the phantom proxy
	 * 
	 * @param phantomProxy the phantom proxy
	 * @param deleteProxyAfterBinding true if the phantom proxy is to be deleted once the prediction
	 *                                is finished, false otherwise
	 * @return list of predicted unions
	 */
	
	protected Vector<Union> getPredictedUnions (PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		try {

			// Reinitialising the predicted unions
			predictedUnions = new Vector<Union>();

			// Adding the phantom proxy into the working memory
			addPhantomProxyToWM (phantomProxy);

			// Wait for the predicted unions to be computed by the binder
			while (predictedUnions.size() == 0) {
				sleepComponent(20);
			}
			
			log("Predicted unions for phantom proxy is sucessfully retrieved");

			// If deleteProxyAfterBinding==true, delete the phantom proxy
			if (deleteProxyAfterBinding) {
				deleteEntityInWM(phantomProxy);
			}

			// Return the predicted unions
			return predictedUnions;
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}


	/**
	 * Retrieve the predicted union with the highest probability of existence for the
	 * given phantom proxy
	 * 
	 * @param phantomProxy the phantom proxy
	 * @param deleteProxyAfterBinding true if the phantom proxy is to be deleted once the prediction
	 *                                is finished, false otherwise
	 * @return list of predicted unions
	 */
	
	protected Union getBestPredictedUnion 
	(PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		
		return getMaximum(getPredictedUnions(phantomProxy, deleteProxyAfterBinding));
	}
	
	
	/**
	 * Retrieve the union with the hightest probability of existence amongst a list of 
	 * possible unions
	 * 
	 * @param unions the vector of unions
	 * @return the max-likelihood union
	 */
	
	private Union getMaximum (Vector<Union> unions) {

		Union maxUnion = new Union();
		float maxValue = -1.0f;

		// Loop on the union vector
		for (Enumeration<Union> e = unions.elements(); e.hasMoreElements() ; ) {
			Union curU = e.nextElement();

			// If the existence probability of current union is higher than the 
			// temporary maximum, update the maximum
			if (curU.probExists > maxValue) {
				maxValue = curU.probExists;
				maxUnion = curU;
			}
		}
		return maxUnion;
	}

	
	
	// =================================================================
	// METHODS FOR INSERTING PHANTOM PROXIES INTO THE WM
	// =================================================================
	
	
	
	/**
	 * Add the phantom proxy to the binding working memory, and specify a new change filter
	 * to detect possible unions for this proxy
	 * 
	 * @param phantomProxy the phantom proxy
	 */
	private void addPhantomProxyToWM (PhantomProxy phantomProxy) {

		// Set the last phantom proxy to be the current one
		lastPhantomProxy = phantomProxy;

		// Construct the change filter to detect new unions including the phantom proxy
		receiverForPhantomProxies =  new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					AlternativeUnionConfigurations configs = 
						getMemoryEntry(_wmc.address, AlternativeUnionConfigurations.class);
					
					Vector<Union> newPredictedUnions = new Vector<Union>();
					
					// Loop on the possible union configurations
					for (int k = 0 ; k < configs.alterconfigs.length ; k++) {
						UnionConfiguration curConfig = configs.alterconfigs[k];
						
						// Loop on the included unions in the config
						for (int i = 0 ; i < curConfig.includedUnions.length ; i++) {
							Union u = curConfig.includedUnions[i];
							
							// Loop on the included proxies in the union
							for (int j = 0 ; j < u.includedProxies.length ; j++) {
								if (u.includedProxies[j].equals(lastPhantomProxy)) {
									newPredictedUnions.add(u);
								}
							}
						}
					}
					// Set the predicted unions
					predictedUnions = newPredictedUnions;
					
					// Remove the change filter
					if (predictedUnions.size() > 0) {
						removeChangeFilter(receiverForPhantomProxies);
					}
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		};
		
		// Add the change filter to the system
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AlternativeUnionConfigurations.class,
				WorkingMemoryOperation.WILDCARD), receiverForPhantomProxies);
		
		// And finally, add the phantom proxy into the working memory
		addProxyToWM (phantomProxy);
	}

}
