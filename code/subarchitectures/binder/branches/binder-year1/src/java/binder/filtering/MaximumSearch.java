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

package binder.filtering;


import java.util.HashMap;
import java.util.Iterator;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.constructors.DistributionGeneration;
import binder.utils.BinderUtils;
import binder.utils.ProbabilityUtils;


/**
 * Utility library to search for maximum values in probability distributions
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 10/08/2009)
 */

public class MaximumSearch {

	// flag to activate error logging
	public static boolean ERRLOGGING = true;

	// flag to activate logging
	public static boolean LOGGING = false;

	private static Logger logger = ComponentLogger.getLogger(MaximumSearch.class);

	
	// Cache of already computed maximum values for perceived entities
	public static HashMap<PerceivedEntity,Float> maxForEntities = new HashMap<PerceivedEntity,Float>();

	
	public static HashMap<ProbabilityDistribution,Float> maxForDistribs = new HashMap<ProbabilityDistribution,Float>();

	
	// =================================================================
	// METHODS TO SEARCH FOR MAXIMUM VALUES IN ENTITIES
	// =================================================================
	
	

	/**
	 * Get the maximum probability value for the given entity
	 * 
	 * @param entity the perceived entity
	 * @return the maximum probability value
	 */

	public static float getMaximum (PerceivedEntity entity) {

		// check if entity == null
		if (entity == null) {
			errlog("WARNING: entity == null, returning 0.0f");
			return 0.0f;
		}

		// check if distribution is already generated
		else if (entity.distribution == null || 
				(!(entity.distribution instanceof DiscreteProbabilityDistribution) && 
				!(entity.distribution instanceof CombinedProbabilityDistribution))) {
			errlog("WARNING: distribution == null, regenerating");
			BinderUtils.addUnknownFeatureValues(entity.features);
			entity.distribution = DistributionGeneration.generateProbabilityDistribution(entity);
		}

		// if the distribution for the entity has already been computed and included in 
		// the cache, just return its value
		if (alreadyComputed(entity)) {
			return maxForEntities.get(entity);
		}

		// else, start computing, and fill the cache with the value
		else {
			float max = getMaximum(entity.distribution);
			maxForEntities.put(entity, max);
			return max;
		}
	}


	// =================================================================
	// METHODS TO SEARCH FOR MAXIMUM VALUES IN DISTRIBUTIONS
	// =================================================================
	
	
	/**
	 * Get the maximum probability value for a given distribution
	 * 
	 * @param distrib the probability distribution
	 * @return the probability value
	 */

	public static float getMaximum (ProbabilityDistribution distrib) {

		float result = 0.0f;
		
		if (false && maxForDistribs.containsKey(distrib)) {
			result = maxForDistribs.get(distrib);
		}
		else {
		// Case 1: distribution is discrete
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			result = getMaximum((DiscreteProbabilityDistribution) distrib);
			maxForDistribs.put(distrib, result);
		}

		// Case 2: distribution is a combined one
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			result = getMaximum((CombinedProbabilityDistribution) distrib);
			maxForDistribs.put(distrib, result);
		}
		else {
			errlog("Sorry, only discrete or combined feature distributions are handled right now");
			log("Used class: " + distrib.getClass());
		}
		}
		
		return result;
		// and, "Houston we have a problem" 
		
	}


	/**
	 * Get the maximum probability value for a given discrete distribution
	 * 
	 * @param distrib the discrete probability distribution
	 * @return the probability value
	 */

	private static float getMaximum (DiscreteProbabilityDistribution distrib) {

		float maxProb = 0.0f;
		if (distrib.assignments != null) {

			// Loop on all assignments
			for (int i = 0 ; i <distrib.assignments.length ; i++) {
				DiscreteProbabilityAssignment assignment = distrib.assignments[i];

				// if assignment probability higher than the current best, update
				if (assignment.prob > maxProb) {
					maxProb = assignment.prob;
				}
			}
		}
		return maxProb;
	}



	/**
	 * Get the maximum probability value for a given combined distribution
	 * 
	 * @param distrib the combined probability distribution
	 * @return the probability value
	 */

	public static float getMaximum (CombinedProbabilityDistribution distrib) {

		float maxProb = 0.0f;

		/** Here, we assume for the moment that the (1) the first distribution inside the combined 
		 * distribution is discrete, and (2) that it contains the complete list of possible assignments 
		 * for the combined distribution 
		 * TODO: find a nicer specification for the assignment in the combined distribution */

		if (distrib.distributions[0] instanceof DiscreteProbabilityDistribution) {
			DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];

			// loop on the possible assignments
			for (int i = 0; i < firstDistrib.assignments.length; i++) {

				DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];

				// Extract the probability value for the assignment
				float probValue = ProbabilityUtils.getProbabilityValue(distrib, assignment);		

				// And if necessary, update the maximum probability
				if (probValue > maxProb) {
					maxProb = probValue;			
				}
			}
		}
		else {
			errlog("WARNING: first distribution insided the combined distribution is not discrete!");
		}

		return maxProb;
	}




	// =================================================================
	// METHODS TO SEARCH FOR BEST ASSIGNMENTS IN DISTRIBUTIONS
	// =================================================================


	public static DiscreteProbabilityAssignment getBestAssignment (ProbabilityDistribution distrib) {

		// discrete probability distribution
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getBestAssignment((DiscreteProbabilityDistribution)distrib);
		}

		// combined probability distribution
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return getBestAssignment((CombinedProbabilityDistribution)distrib);
		}
		
		// else ...
		else {
			errlog("WARNING: type of probability distribution currently not supported");
			return new DiscreteProbabilityAssignment();
		}
	}

	
	/**
	 * Get the best (maximum-probability) assignment for a given combined distribution
	 * 
	 * @param distrib the combined probability distribution
	 * @return the best assignment
	 */

	public static DiscreteProbabilityAssignment getBestAssignment 
	(CombinedProbabilityDistribution distrib) {

		log("Searching maximum value for a combined probability distribution...");
		float maxProb = 0.0f;

		DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;

		// Cf. note in previous method!
		if (distrib.distributions[0] instanceof DiscreteProbabilityDistribution) {

			DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];

			// Loop on the assignments
			for (int i = 0; i < firstDistrib.assignments.length; i++) {

				DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
				
				// Extract the probability value for the assignment
				float probValue = ProbabilityUtils.getProbabilityValue(distrib, assignment);

				// And if necessary, update the best assignment
				if (probValue > maxProb) {
					maxProb = probValue;
					bestAssign = assignment;	
				}
			}
		}
		log("Best assignment: " + BinderUtils.getPrettyPrintProbabilityAssignment(bestAssign));

		return bestAssign;

	}


	/**
	 * Get the best (maximum-probability) assignment for a given discrete distribution
	 * 
	 * @param distrib the discrete probability distribution
	 * @return the best assignment
	 */
	public static DiscreteProbabilityAssignment getBestAssignment  
		(DiscreteProbabilityDistribution distrib) {

		float maxProb = 0.0f;
		DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;
		if (distrib.assignments != null) {
			
			// loop on the assignments
			for (int i = 0 ; i <distrib.assignments.length ; i++) {
				DiscreteProbabilityAssignment assignment = distrib.assignments[i];
				
				// If necessary, update the best assignment
				if (assignment.prob > maxProb) {
					maxProb = assignment.prob;
					bestAssign = assignment;
				}
			}
		}
		return bestAssign;
	}

	

	
	// =================================================================
	// UTILITY METHODS
	// =================================================================
	
	
	/**
	 * Check if the entity has already been computed in the cache
	 * 
	 * TODO: is this still necessary?
	 * 
	 * @param entity the entity
	 * @return true if the max for the entity has already been cached, false otherwise
	 */
	synchronized public static boolean alreadyComputed (PerceivedEntity entity) {
		for (Iterator<PerceivedEntity> i = maxForEntities.keySet().iterator() ; i.hasNext() ; ) {
			PerceivedEntity u = i.next();
			if (u.equals(entity) && u.timeStamp == entity.timeStamp) {
				return true;
			}
		}
		return false;  
	}



	public static void log(String s) {
		if (LOGGING)
			logger.debug("[MaximumSearch] " + s);
	}

	public static void errlog(String s) {
		if (ERRLOGGING)
			logger.debug("[MaximumSearch] " + s);
	}
}
