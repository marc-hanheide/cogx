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

import beliefmodels.domainmodel.cogx.BoundPhantomProxyProperty;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import binder.autogen.core.Feature;
import binder.autogen.specialentities.PhantomProxy;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTData;

/**
 * Abstract class for inserting phantom proxies onto the binder working memory,
 * and retrieving (ranked) lists of predicted belief model formula for these proxies,
 * based on the generated binding unions 
 * 
 * @author Pierre Lison
 * @version 22/09/2009 (started 10/09/2009)
 */

public class BindingPredictor extends ProxyWriter {

	//Last phantom proxy inserted onto the binder WM
	static PhantomProxy lastPhantomProxy = new PhantomProxy();

	// Change receiver for phantom proxies 
	private static WorkingMemoryChangeReceiver receiverForPhantomProxies;	


	// List of predicted beliefs for lastPhantomProxy
	static boolean foundPossibleBindings = false;


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

		log("new phantom proxy successfully created!");
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
	 * Get a list of predicted possible belief bindings for the phantom proxy
	 * 
	 * @param phantomProxy the phantom proxy
	 * @param deleteProxyAfterBinding true if the phantom proxy is to be deleted once the prediction
	 *                                is finished, false otherwise
	 * @return list of predicted belief bindings
	 */

	protected Vector<UncertainSuperFormula> getPredictedBindings 
	(PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		try {

			// Reinitialising the predicted unions
			foundPossibleBindings = false;

			// Adding the phantom proxy into the working memory
			addPhantomProxyToWM (phantomProxy);

			// Wait for the predicted unions to be computed by the binder
			while (!foundPossibleBindings) {
				sleepComponent(20);
			}

			log("Predicted beliefs for phantom proxy is sucessfully retrieved");

			Vector<UncertainSuperFormula> possibleBindings = new Vector<UncertainSuperFormula>();

			// If deleteProxyAfterBinding==true, delete the phantom proxy, and also update 
			if (deleteProxyAfterBinding) {
				log("now deleting phantom proxy...");
				deleteEntityInWM(phantomProxy);
				
				foundPossibleBindings = false;
				addChangeDetectorOnWM();
				
				while (!foundPossibleBindings) {
					sleepComponent(20);
				}
			}
						
			CASTData<ComplexFormula>[] formulae = getWorkingMemoryEntries(ComplexFormula.class);
			
			for (int i = 0; i < formulae.length; i++) {
				possibleBindings.addAll(getAllFormulaeIncludingProxy(formulae[i].getData(), lastPhantomProxy));
			}
			return possibleBindings;
			
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}


	/**
	 * Retrieve the predicted belief binding with the highest probability of existence for the
	 * given phantom proxy
	 * 
	 * @param phantomProxy the phantom proxy
	 * @param deleteProxyAfterBinding true if the phantom proxy is to be deleted once the prediction
	 *                                is finished, false otherwise
	 * @return list of predicted belief binding 
	 */

	protected UncertainSuperFormula getBestPredictedBinding
	(PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {

		return getMaximum(getPredictedBindings(phantomProxy, deleteProxyAfterBinding));
	}

	
	/**
	 * Retrieve the union with the hightest probability of existence amongst a list of 
	 * possible unions
	 * 
	 * @param unions the vector of unions
	 * @return the max-likelihood union
	 */

	private UncertainSuperFormula getMaximum (Vector<UncertainSuperFormula> beliefs) {

		UncertainSuperFormula maxFormula = new UncertainSuperFormula();
		float maxValue = -1.0f;

		// Loop on the union vector
		for (Enumeration<UncertainSuperFormula> e = beliefs.elements(); e.hasMoreElements() ; ) {
			SuperFormula cuf = e.nextElement();

			// If the existence probability of current union is higher than the 
			// temporary maximum, update the maximum
			if (cuf instanceof UncertainSuperFormula) {
			if (((UncertainSuperFormula)cuf).prob > maxValue) {
				maxValue = ((UncertainSuperFormula)cuf).prob;
				maxFormula = ((UncertainSuperFormula)cuf);
			}
			}
		}
		return maxFormula;
	}



	// =================================================================
	// METHODS FOR INSERTING PHANTOM PROXIES INTO THE WM
	// =================================================================


	public static Vector<UncertainSuperFormula> getAllFormulaeIncludingProxy (UncertainSuperFormula formula, PhantomProxy proxy) {

		Vector<UncertainSuperFormula> buFormulae = new Vector<UncertainSuperFormula>();

		if (formula instanceof ComplexFormula) {
			for (int i = 0; i < ((ComplexFormula)formula).formulae.length ; i++) {

				UncertainSuperFormula subformula = (UncertainSuperFormula) ((ComplexFormula)formula).formulae[i];
				
				if (subformula instanceof BoundPhantomProxyProperty) {
					buFormulae.add (formula);
				}
				
				else {
					Vector<UncertainSuperFormula> partialResult = 
					getAllFormulaeIncludingProxy ((UncertainSuperFormula)((ComplexFormula)formula).formulae[i], proxy);

					buFormulae.addAll(partialResult);
				}
			}
		}

		return buFormulae;
	}

	/**
	 * Add the phantom proxy to the binding working memory, and specify a new change filter
	 * to detect possible unions for this proxy
	 * 
	 * @param phantomProxy the phantom proxy
	 */
	private void addPhantomProxyToWM (PhantomProxy phantomProxy) {

		// Set the last phantom proxy to be the current one
		lastPhantomProxy = phantomProxy;

		addChangeDetectorOnWM();

		// And finally, add the phantom proxy into the working memory
		addProxyToWM (phantomProxy);
	}

	
	
	private void addChangeDetectorOnWM () {
		// Construct the change filter to detect new unions including the phantom proxy
		receiverForPhantomProxies =  new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					UncertainSuperFormula initFormula = getMemoryEntry(_wmc.address, UncertainSuperFormula.class);

					Vector<UncertainSuperFormula> possibleBindings = 
						getAllFormulaeIncludingProxy(initFormula, lastPhantomProxy);


					// Remove the change filter
					if (possibleBindings.size() > 0) {
						foundPossibleBindings = true;
					}
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		};

		// Add the change filter to the system
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(SuperFormula.class,
				WorkingMemoryOperation.WILDCARD), receiverForPhantomProxies);
	}
	
}
