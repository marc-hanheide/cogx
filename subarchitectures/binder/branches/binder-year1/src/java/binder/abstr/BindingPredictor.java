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

import beliefmodels.adl.Belief;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import binder.autogen.specialentities.PhantomProxy;
import binder.components.Binder;
import binder.utils.BeliefModelUtils;

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
	PhantomProxy lastPhantomProxy = new PhantomProxy();

	public static boolean lastPhantomProxyToDelete = false;
	
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

	protected Vector<Belief> getPredictedBindings 
	(PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		try {

			lastPhantomProxyToDelete = deleteProxyAfterBinding;
			
			// Adding the phantom proxy into the working memory
			addPhantomProxyToWM (phantomProxy);

			log("OK, just added phantom proxy onto the WM");
			
			Vector<UncertainSuperFormula> constraints = getPhantomProxyConstraints(phantomProxy);

			log("retrieving possible beliefs including the constraints imposed in the phantom proxy...");

			log("First try...");
			Vector<Belief> bindings = getPossibleBindings(constraints);
			
			// Wait for the predicted unions to be computed by the binder
			int count = 0;
			while ((bindings.size() == 0) && (count < 5)) {
				sleepComponent(20);
				log("Try " + (count+2));
				bindings = getPossibleBindings(constraints);
				count++;
			}

			if (count < 5) {
				log("Predicted beliefs for phantom proxy is sucessfully retrieved!");
			}
			else {
				log("WARNING: *no* predicted beliefs found");
			}

			// If deleteProxyAfterBinding==true, delete the phantom proxy, and also update 
			if (deleteProxyAfterBinding) {
				log("now deleting phantom proxy...");
				deleteEntityInWM(phantomProxy);
			}
			
			
						
			return bindings;
			
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}


	private Vector<Belief> getPossibleBindings(Vector<UncertainSuperFormula> constraints) {
		
		Vector<Belief> matchingBeliefs = new Vector<Belief>(); 

		try {
		CASTData<Belief>[] beliefs = getWorkingMemoryEntries(Binder.BINDER_SA, Belief.class);
		
		Vector<Belief> allBeliefs = new Vector<Belief>();

		for (int i = 0 ; i <beliefs.length; i++) {
			allBeliefs.add(beliefs[i].getData());
		}
		
		matchingBeliefs = getAllBeliefsSatisfyingConstraints(allBeliefs, constraints);
	
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return matchingBeliefs;
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

	protected Belief getBestPredictedBinding
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

	private Belief getMaximum (Vector<Belief> beliefs) {

		Belief maxFormula = null;
		float maxValue = -1.0f;

		// Loop on the union vector
		for (Enumeration<Belief> e = beliefs.elements(); e.hasMoreElements() ; ) {
			Belief b = e.nextElement();

			// If the existence probability of current union is higher than the 
			// temporary maximum, update the maximum
			if (b.phi instanceof UncertainSuperFormula) {
			if (((UncertainSuperFormula)b.phi).prob > maxValue) {
				maxValue = ((UncertainSuperFormula)b.phi).prob;
				maxFormula = ((Belief)b);
			}
			}
		}
		return maxFormula;
	}



	// =================================================================
	// METHODS FOR INSERTING PHANTOM PROXIES INTO THE WM
	// =================================================================

	
	private Vector<UncertainSuperFormula> getPhantomProxyConstraints (PhantomProxy proxy) {
		
		Vector<UncertainSuperFormula> propertiesInPhantom = new Vector<UncertainSuperFormula>();
		for (int i = 0 ; i < proxy.features.length ; i++) {
			for (int j = 0 ; j < proxy.features[i].alternativeValues.length ; j++) {
				UncertainSuperFormula newProp = 
					BeliefModelUtils.createNewProperty
					(proxy.features[i].featlabel, proxy.features[i].alternativeValues[j]);
				propertiesInPhantom.add(newProp);
			}
		}
		return propertiesInPhantom;
	}

	
	public Vector<Belief> getAllBeliefsSatisfyingConstraints 
			(Vector<Belief> allBeliefs, Vector<UncertainSuperFormula> constraints) {
		
	/**	log("constraints: " + constraints.size());
		for (int z = 0; z < constraints.size(); z++) {
			log("C" + (z+1) + ": "  + BeliefModelUtils.getFormulaPrettyPrint(constraints.elementAt(z)));
		}
		
		log("total number of beliefs: " + allBeliefs.size());  */
		
		Vector<Belief> boundBeliefs = new Vector<Belief>();
	


		// looping on all beliefs
		for (Enumeration<Belief> e = allBeliefs.elements(); e.hasMoreElements();) {

			Belief curBelief = e.nextElement();

			boolean allConstraintsSatisfied = true;

			// looping on the constraints, verifying if each of them is satisfied
			for (int j = 0 ; j < constraints.size(); j++) {

				UncertainSuperFormula constraint = constraints.elementAt(j);

				boolean constraintSatisfied = false;
				if (curBelief.phi instanceof ComplexFormula) {

					for (int i = 0; i < ((ComplexFormula)curBelief.phi).formulae.length ; i++) {

						UncertainSuperFormula formula = 
							(UncertainSuperFormula) ((ComplexFormula)curBelief.phi).formulae[i];

						if (formula instanceof ComplexFormula) {
							
							for (int k = 0; k < ((ComplexFormula)formula).formulae.length ; k++) {
								
								UncertainSuperFormula subformula = 
									(UncertainSuperFormula) ((ComplexFormula)formula).formulae[k];
								if (BeliefModelUtils.arePropertiesEqual(subformula, constraint)) {
									constraintSatisfied = true;
								}
							}
						}
						else {
							if (BeliefModelUtils.arePropertiesEqual(formula, constraint)) {
								constraintSatisfied = true;
							}
						}
						
					}
				}
				
				if (!constraintSatisfied) {
					allConstraintsSatisfied = false;
				}
			}
			
			if (allConstraintsSatisfied && !containsBelief(boundBeliefs, curBelief)) {
				boundBeliefs.add(curBelief);
			}
		}
		
		return boundBeliefs;
	}
	

	private boolean containsBelief (Vector<Belief> beliefs, Belief belief) {
		
		for (Enumeration<Belief> e = beliefs.elements(); e.hasMoreElements(); ) {
			Belief curBelief = e.nextElement();
			// TODO: correct this
			if (belief.id.equals(curBelief.id) && 
					((ComplexFormula)belief.phi).formulae.length == ((ComplexFormula)curBelief.phi).formulae.length) {
				return true;
			}
	}
		return false;
	
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

		// And finally, add the phantom proxy into the working memory
		addProxyToWM (phantomProxy);
	}

	
}
