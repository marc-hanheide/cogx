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

package binder.utils;

import java.util.Enumeration;
import java.util.Vector;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.components.Binder;
import binder.utils.FeatureValueUtils;

/**
 * Library for manipulation of probability values and assignments
 * 
 * @author Pierre Lison
 * @version 24/09/2009 (started 10/08/2009)
 */

public class ProbabilityUtils {

	// flag to activate error logging
	public static boolean ERRLOGGING = true;

	// flag to activate logging
	public static boolean LOGGING = true;

	private static Logger logger = ComponentLogger.getLogger(ProbabilityUtils.class);

	
	// ================================================================= 
	// MARGINAL PROBABILITY VALUES METHODS   
	// ================================================================= 
	

	/**
	 * Get the marginal probability value of a feature pair given the probability distribution.
	 * At the moment, the only supported probability distributions are discrete or combined.
	 * 
	 * @param distrib the probability distribution
	 * @param pair the feature pair
	 * @return the probability value
	 */

	public static float getMarginalProbabilityValue
	(ProbabilityDistribution distrib, FeatureValuePair pair) {
				
		// discrete distribution
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getMarginalProbabilityValue((DiscreteProbabilityDistribution) distrib, pair);
		}

		// combined distribution
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return getMarginalProbabilityValue((CombinedProbabilityDistribution) distrib, pair);
		}

		// houston, we have a problem
		else {
			errlog("WARNING: type of probability distribution not supported yet!");
		}

		return 0.0f;
	}


	/**
	 * Get the marginal probability value of a feature pair given the discrete probability 
	 * distribution.
	 * 
	 * @param distrib the discrete probability distribution
	 * @param pair the feature pair
	 * @return the probability value
	 */

	private static float getMarginalProbabilityValue
	(DiscreteProbabilityDistribution distrib, FeatureValuePair pair) {

		float result = 0.0f;

		// get the list of assignments including the feature pair
		Vector<DiscreteProbabilityAssignment> assignments = 
			getAssignmentsIncludingFeaturePair(distrib, pair);

		// loop on these assignments
		for (Enumeration<DiscreteProbabilityAssignment> enu = 
			assignments.elements() ; enu.hasMoreElements() ; ) {

			// and sum up their probabilities
			result += enu.nextElement().prob;
		}

		return result;
	}

	/**
	 * Get the marginal probability value of a feature pair given the combined probability 
	 * distribution.
	 * 
	 * @param distrib the combined probability distribution
	 * @param pair the feature pair
	 * @return the probability value
	 */

	private static float getMarginalProbabilityValue
	(CombinedProbabilityDistribution distrib, FeatureValuePair pair) {

		float result = 0.0f;
		
		if (distrib.distributions.length > 0) {

			/** Here, we assume for the moment that the (1) the first distribution inside the combined 
			 * distribution is discrete, and (2) that it contains the complete list of possible assignments 
			 * for the combined distribution  */

			DiscreteProbabilityDistribution firstdistrib = 
				(DiscreteProbabilityDistribution) distrib.distributions[0];

			// get the list of assignments including the feature pair
			Vector<DiscreteProbabilityAssignment> assignments = 
				getAssignmentsIncludingFeaturePair(firstdistrib, pair);

			for (Enumeration<DiscreteProbabilityAssignment> enu = 
				assignments.elements() ; enu.hasMoreElements() ; ) {
				DiscreteProbabilityAssignment assign = enu.nextElement();

				// get the probability value for this particular assignment
				float partialresult = getProbabilityValue(distrib, assign);

				// and sum the resulting probability
				result += partialresult;
			}	
		}

		return result;
	}

	/**
	 * Get the list of assignments containing the particular feature pair
	 * 
	 * @param distrib the discrete probability distribution
	 * @param pair the feature pair
	 * @return the list of assignments
	 */
	private static Vector<DiscreteProbabilityAssignment> getAssignmentsIncludingFeaturePair(
			DiscreteProbabilityDistribution distrib, FeatureValuePair pair) {

		Vector<DiscreteProbabilityAssignment> assignments = new Vector<DiscreteProbabilityAssignment>();

		// loop on the assignments
		for (int i =0; i < distrib.assignments.length; i++) {

			DiscreteProbabilityAssignment assign = distrib.assignments[i];

			// if the assignment contains the pair, add it to the list
			if (containsFeatureValuePair(assign, pair)) {
				assignments.add(assign);
			}
		}

		return assignments;
	}


	/**
	 * Returns true if the assignment contains the feature pair, false otherwise
	 * 
	 * @param assign the assignment
	 * @param pair the feature pair
	 * @return true if it contains the pair, false otherwise
	 */

	public static boolean containsFeatureValuePair
	(DiscreteProbabilityAssignment assign, FeatureValuePair pair) {

		// Construct a list of feature value pairs in the assignment
		Vector<FeatureValuePair> pairsList = new Vector<FeatureValuePair>();
		for (int j = 0; j < assign.featurepairs.length; j++) {
			pairsList.add(assign.featurepairs[j]);
		}

		// loop on these pairs
		for (Enumeration<FeatureValuePair> f = pairsList.elements() ; f.hasMoreElements() ; ) {

			FeatureValuePair pairInList = f.nextElement();

			// check if the pair in the list is equivalent to the specific pair
			if (pairInList.featlabel.equals(pair.featlabel) 
					&& (FeatureValueUtils.haveEqualValue(pairInList.featvalue, pair.featvalue))) {
				return true;
			}
		}
		return false;
	}


	// ================================================================= 
	// PROBABILITY VALUE EXTRACTION METHODS   
	// ================================================================= 
	
	
	/**
	 * Get the probability value of a particular assignment given the distribution
	 * 
	 * @param distrib the distribution
	 * @param assignment the assignment
	 * @return the probability value
	 */
	
	public static float getProbabilityValue
	(ProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {

		if (distrib == null) {
			errlog("ERROR, distribution is null");
		}

		// discrete distribution
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getProbabilityValue((DiscreteProbabilityDistribution)distrib, assignment);
		}
		
		// combined distribution
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return getProbabilityValue((CombinedProbabilityDistribution)distrib, assignment);
		}
		
		// houston we have a problem
		else {
			errlog("WARNING: sorry, only discrete and combined probability distribution are supported at the moment!");
			return 0.0f;
		}
	}

	
	/**
	 * Get the probability value of a particular assignment given the combined distribution
	 * 
	 * @param distrib the combined distribution
	 * @param assignment the assignment
	 * @return the probability value
	 */
	
	private static float getProbabilityValue 
	(CombinedProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
	
		float result = 1.0f;

		// loop on the subdistributions in the combined distrib
		for (int i = 0; i < distrib.distributions.length; i++) {
			
			ProbabilityDistribution subdistrib = distrib.distributions[i];
		
			if (subdistrib == null) {
				log("ERROR, distribution is null");
			}
			else if (distrib.opType.equals(OperationType.MULTIPLIED)) {
				
				// get the probability value for the assignment in the subdistribution
				float subdistribresult = getProbabilityValue(subdistrib, assignment); 
				
				// and multiply the result by it
				result = result * subdistribresult;
			}
			else {
				errlog("WARNING: operation type in combined distribution currently not supported");
			}
		}

		return result;
	}

	
	/**
	 * Get the probability value of a particular assignment given the discrete distribution
	 * 
	 * @param distrib the discrete distribution
	 * @param assignment the assignment
	 * @return the probability value
	 */
	
	private static float getProbabilityValue 
	(DiscreteProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		
		float result = 0.0f;

		if (distrib.assignments != null) {
			
			// loop on the assignments
			for (int i = 0; i < distrib.assignments.length; i++) {
				
				DiscreteProbabilityAssignment curAssignment = distrib.assignments[i];
				
				/** If the currently considered assignment contains all feature value pairs in
				 * "assignment", simply returns the result (this assumes that the discrete 
				 * distribution is well-formed, i.e. that all assignments in it are mutually 
				 * exclusive */
				if (containsAll(assignment, curAssignment)) {
					return curAssignment.prob;
				}
			}
		}
		else {
			errlog("ERROR, distribution contains no assignment");
		}

		if (Binder.proxyDistribFilter == 0) {
			errlog("WARNING, no probability value found for assignment: " + 
				BinderUtils.getPrettyPrintProbabilityAssignment(assignment));
			errlog("distrib: " + BinderUtils.getPrettyPrintProbabilityDistribution(distrib));
		}
		return result;
	}


	/**
	 * Returns true is bigassign contains all the feature pairs included in smallassign,
	 * false otherwise
	 * 
	 * @param bigassign the big assignment
	 * @param smallassign the small assignment
	 * @return true if bigassign contains the feat pairs in smallassign, false otherwise
	 */
	
	public static boolean containsAll
	(DiscreteProbabilityAssignment bigassign, DiscreteProbabilityAssignment smallassign) {

		// construct the first list of feature pairs
		Vector<FeatureValuePair> pairs1 = new Vector<FeatureValuePair>();
		for (int j = 0; j < bigassign.featurepairs.length; j++) {
			pairs1.add(bigassign.featurepairs[j]);
		}

		// construct the second list of feature pairs
		Vector<FeatureValuePair> pairs2 = new Vector<FeatureValuePair>();
		for (int j = 0; j < smallassign.featurepairs.length; j++) {
			pairs2.add(smallassign.featurepairs[j]);
		}

		// and check if the second is contained in the first
		return containsAll(pairs1, pairs2);	
	}

	
	/**
	 * Returns true if the elements in pairs2 are contained in pairs1, false otherwise
	 * 
	 * @param pairs1 first feature pair
	 * @param pairs2 second feature pair
	 * @return true if contained, false if not
	 */
	 
	public static boolean containsAll(Vector<FeatureValuePair> pairs1, 
			Vector<FeatureValuePair> pairs2) {
	
		boolean result = true;

		// loop on the elements in pairs2
		for (Enumeration<FeatureValuePair> e = pairs2.elements() ; e.hasMoreElements(); ) {
			FeatureValuePair pair2 = e.nextElement();
			boolean foundMatch = false;
			
			// loop on the elements in pairs2
			for (Enumeration<FeatureValuePair> f = pairs1.elements() ; 
			f.hasMoreElements()  && !foundMatch; ) {
			
				FeatureValuePair pair1 = f.nextElement();
				
				// and check equivalence
				if (pair1.featlabel.equals(pair2.featlabel) 
						&& (FeatureValueUtils.haveEqualValue(pair1.featvalue, pair2.featvalue))) {
					foundMatch = true;
				}
			}
			if (!foundMatch) {
				return false;
			}
		}
		return result;
	}

	
	// ================================================================= 
	// PROBABILITY DISTRIBUTION MANIPULATION METHODS   
	// ================================================================= 
	
	
	
	/**
	 * Multiply the probability values in the distribution by a constant value
	 * 
	 * @param distrib the discrete probability distribution
	 * @param constantValue the value
	 * @return a new discrete probability distribution, with multiplied probability values
	 */
	
	public static DiscreteProbabilityDistribution multiplyDistributionWithConstantValue (
			DiscreteProbabilityDistribution distrib, float constantValue) {

		// create a new probability distribution
		DiscreteProbabilityDistribution newDistrib = new DiscreteProbabilityDistribution();
		newDistrib.assignments = new DiscreteProbabilityAssignment[distrib.assignments.length];

		// loop on each assignment
		for (int i = 0; i < distrib.assignments.length; i++) {
			newDistrib.assignments[i] = new DiscreteProbabilityAssignment();
			newDistrib.assignments[i].featurepairs = distrib.assignments[i].featurepairs;
			
			// multiply the probability by the constant value
			newDistrib.assignments[i].prob = distrib.assignments[i].prob * constantValue;
		}

		return newDistrib;
	}

	/**
	 * Invert the probability values of each assignment in the distribution
	 * --> i.e., apply the function P2 (x) = 1 / P1(x)
	 * 
	 * @param distrib the discrete probability distribution
	 * @return the new, inverted discrete probability distribution
	 */
	
	public static DiscreteProbabilityDistribution invertDistribution (
			DiscreteProbabilityDistribution distrib) {

		// create a new distribution
		DiscreteProbabilityDistribution newDistrib = new DiscreteProbabilityDistribution();
		newDistrib.assignments = new DiscreteProbabilityAssignment[distrib.assignments.length];

		// loop on the assignments
		for (int i = 0; i < distrib.assignments.length; i++) {
			
			// verify the probability is greated than 0 (to avoid division by zero problems)
			if (distrib.assignments[i].prob > 0) {
				newDistrib.assignments[i] = new DiscreteProbabilityAssignment();
				newDistrib.assignments[i].featurepairs = distrib.assignments[i].featurepairs;
				
				// and invert the probability value
				newDistrib.assignments[i].prob = 1 / distrib.assignments[i].prob ;
			}
		}

		return newDistrib;
	}


	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 
	
	

	public static void log(String s) {
		if (LOGGING)
			logger.debug("[ProbabilityUtils] " + s);
	}

	public static void errlog(String s) {
		if (ERRLOGGING)
			logger.debug("[ProbabilityUtils] " + s);
	}

}
