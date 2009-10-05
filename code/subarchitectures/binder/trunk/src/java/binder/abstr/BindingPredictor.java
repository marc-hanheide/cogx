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
import beliefmodels.domainmodel.cogx.LinguisticLabelProperty;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import binder.autogen.core.Feature;
import binder.autogen.featvalues.StringValue;
import binder.autogen.specialentities.PhantomProxy;
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

	protected Vector<ComplexFormula> getPredictedBindings 
	(PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		try {

			// Adding the phantom proxy into the working memory
			addPhantomProxyToWM (phantomProxy);

			log("OK, just added phantom proxy onto the WM");
			
			Vector<ComplexFormula> bindings = getPossibleBindings();
			
			// Wait for the predicted unions to be computed by the binder
			while (bindings.size() == 0) {
				sleepComponent(20);
				bindings = getPossibleBindings();
			}

			log("Predicted beliefs for phantom proxy is sucessfully retrieved");

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


	private Vector<ComplexFormula> getPossibleBindings() {
		
		Vector<ComplexFormula> matchingFormulae = new Vector<ComplexFormula>(); 

		try {
		CASTData<ComplexFormula>[] formulae = getWorkingMemoryEntries("binder", ComplexFormula.class);
		
		for (int i = 0 ; i <formulae.length; i++) {
			Vector<ComplexFormula> matchingFormula = getAllFormulaeIncludingProxy((ComplexFormula)formulae[i].getData(), lastPhantomProxy);
			matchingFormulae.addAll(matchingFormula);
		}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return matchingFormulae;
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

	protected ComplexFormula getBestPredictedBinding
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

	private ComplexFormula getMaximum (Vector<ComplexFormula> beliefs) {

		ComplexFormula maxFormula = new ComplexFormula();
		float maxValue = -1.0f;

		// Loop on the union vector
		for (Enumeration<ComplexFormula> e = beliefs.elements(); e.hasMoreElements() ; ) {
			SuperFormula cuf = e.nextElement();

			// If the existence probability of current union is higher than the 
			// temporary maximum, update the maximum
			if (cuf instanceof UncertainSuperFormula) {
			if (((UncertainSuperFormula)cuf).prob > maxValue) {
				maxValue = ((UncertainSuperFormula)cuf).prob;
				maxFormula = ((ComplexFormula)cuf);
			}
			}
		}
		return maxFormula;
	}



	// =================================================================
	// METHODS FOR INSERTING PHANTOM PROXIES INTO THE WM
	// =================================================================


	public static Vector<ComplexFormula> getAllFormulaeIncludingProxy (ComplexFormula formula, PhantomProxy proxy) {

		Vector<ComplexFormula> buFormulae = new Vector<ComplexFormula>();

		
		Vector<UncertainSuperFormula> propertiesInPhantom = new Vector<UncertainSuperFormula>();
		for (int i = 0 ; i < proxy.features.length ; i++) {
			for (int j = 0 ; j < proxy.features[i].alternativeValues.length ; j++) {
				UncertainSuperFormula newProp = 
					BeliefModelUtils.createNewProperty
					(proxy.features[i].featlabel, proxy.features[i].alternativeValues[j]);
				propertiesInPhantom.add(newProp);
			}
		}
		

		if (formula instanceof ComplexFormula) {
			for (int i = 0; i < ((ComplexFormula)formula).formulae.length ; i++) {

				UncertainSuperFormula subformula = (UncertainSuperFormula) ((ComplexFormula)formula).formulae[i];
				
				if (! (subformula instanceof ComplexFormula)) {
				
					// TODO: fix this
					for (int j = 0 ; j < propertiesInPhantom.size(); j++) {
					UncertainSuperFormula curProp = propertiesInPhantom.elementAt(j);
					if (BeliefModelUtils.arePropertiesEqual(subformula, curProp)) {
						buFormulae.add (formula);	
					}
				}
				
				}
				else {
					Vector<ComplexFormula> partialResult = 
					getAllFormulaeIncludingProxy ((ComplexFormula) subformula, proxy);
					
					for (Enumeration<ComplexFormula> e = partialResult.elements(); e.hasMoreElements(); ) {
						ComplexFormula form = e.nextElement();
						if (!containsFormula(buFormulae, form)) {
							buFormulae.add(form);
						}
					}
				}
			}
		}

		return buFormulae;
	}

	private static boolean containsFormula (Vector<ComplexFormula> formulae, ComplexFormula form) {
		
		for (Enumeration<ComplexFormula> e = formulae.elements(); e.hasMoreElements(); ) {
			ComplexFormula curForm = e.nextElement();
			// TODO: correct this
			if (curForm.id.equals(form.id) && curForm.formulae.length == form.formulae.length) {
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
