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


package binder.fakeproxygenerators;

import java.util.Map;
import java.util.Random;

import beliefmodels.adl.Belief;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import binder.abstr.BindingPredictor;
import binder.autogen.core.Union;
import binder.autogen.specialentities.PhantomProxy;
import binder.utils.BeliefModelUtils;


/**
 * Abstract class to generate and insert phantom proxies into the binder
 * 
 * @author Pierre Lison
 * @version 22/09/2009 (started 05/09/2009)
 */

abstract public class AbstractPhantomProxyGenerator extends BindingPredictor {

	// Number of proxies to create
	protected int nbOfProxiesToCreate = 0;
	
	// Whether to create the proxies in the standard or inverted order
	protected boolean reverted = false;

	// Whether to make pauses (of about 1 s.) between the proxy insertions
	protected boolean pauses = false;

	// Whether to delete the phantom proxies after binding occurred
	protected boolean deleteProxiesAfterBinding = true;
	
	
	/**
	 * Parameter configuration
	 */
	@Override
	public void configure(Map<String, String> _config) {

		if (_config.containsKey("--nbproxies")) {
			nbOfProxiesToCreate = Integer.parseInt(_config.get("--nbproxies"));
		} 
		if (_config.containsKey("--reverted")) {
			reverted = Boolean.parseBoolean(_config.get("--reverted"));
		} 
		if (_config.containsKey("--pauses")) {
			pauses = Boolean.parseBoolean(_config.get("--pauses"));
		} 
		if (_config.containsKey("--deleteproxies")) {
			deleteProxiesAfterBinding = Boolean.parseBoolean(_config.get("--deleteproxies"));
		}
	}
	
	
	
	/**
	 * Create and inserts the phantom proxies, and output the best predicted union
	 * for each
	 * 
	 */
  
	protected void randomPrediction() {

		Random rand = new Random();
		if (nbOfProxiesToCreate > 0) {
			
			// Loop on the proxies to crate
			for (int i = 1 ; i < (nbOfProxiesToCreate +1) ; i++) {
				
				// Make a short pause
				if (pauses) {
				sleepComponent(2000 + rand.nextInt(1000));
				}
				 
				// Create the phantom proxy
				PhantomProxy p;
				if (!reverted) {
					p = createPhantomProxy(i);
				}
				else {
					p = createPhantomProxy(nbOfProxiesToCreate-i+1);
				}
				   
				// Get the best predicted union for the phantom proxy
				Belief belief = getBestPredictedBinding(p, deleteProxiesAfterBinding);
				if (belief != null) {
					log("PREDICTED BELIEF: \n" + BeliefModelUtils.getBeliefPrettyPrint(belief, 1));
				}
			}	
		}
	}
	  
	
	/**
	 *  Create the proxy nb (nb being an integer between 1 and nbOfProxiesToCreate)
	 *  
	 * @param nb index of the proxy to create
	 * @return the created phantom proxy
	 */
	
	abstract public PhantomProxy createPhantomProxy(int nb);
	
}
