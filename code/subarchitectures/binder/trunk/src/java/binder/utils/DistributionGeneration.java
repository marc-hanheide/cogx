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

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;


/**
 * Library for generating and normalising probability distributions from feature lists
 * associated with independent probabilities
 * 
 * @author Pierre Lison
 * @version 23/09/2009
 * @started 20/08/2009
 */

public class DistributionGeneration {

	// whether to normalise probability distributions (to sum up to 1.0) 
	// after generating them
	public static boolean normaliseDistributions = true;
	

	
	/**
	 * Generate a probability distribution for the given entity
	 * 
	 * @param entity the entity
	 * @return the probability distribution
	 */
	
	public static ProbabilityDistribution generateProbabilityDistribution (PerceivedEntity entity) {
		
		// Initialise the distribution
		DiscreteProbabilityDistribution distrib = new DiscreteProbabilityDistribution();

		// usual case: we have at least one feature in the entity
		if (entity.features.length > 0) {
			
			// construct the features list
			Vector<Feature> features = new Vector<Feature>();
			for (int i = 0; i < entity.features.length ; i++) {
				features.add(entity.features[i]);
			}

			// and generate the set of assignment functions from it
			Vector<DiscreteProbabilityAssignment> assignments = 
				generateProbabilityDistribution (features);
			
			// define the distribution based on these assignments
			distrib.assignments = new DiscreteProbabilityAssignment[assignments.size()];
			distrib.assignments = assignments.toArray(distrib.assignments);

		}
		
		// if we have no features, create a dummy distribution with only one empty assignment
		// with probability 1.0f
		else {
			distrib.assignments = new DiscreteProbabilityAssignment[1];
			distrib.assignments[0] = new DiscreteProbabilityAssignment();
			distrib.assignments[0].featurepairs = new FeatureValuePair[0];
			distrib.assignments[0].prob = 1.0f;
		}
		
		// If necessary, normalise the probability distribution to sum all assignment probabilities
		// to 1.0f
		if (normaliseDistributions) {
			distrib = normaliseDistribution(distrib, 1.0f);
		}
		
		// and finally, multiply all assignment probabilities with the existence probability
		// of the entity
		for (int i = 0 ; i < distrib.assignments.length ; i++) {
			distrib.assignments[i].prob = distrib.assignments[i].prob * entity.probExists;
		}
		
		return distrib;
	}

	
	/**
	 * Generate a list of probability assignments from the features list
	 * 
	 * @param features the feature list
	 * @return the list of probability assignments
	 */
	
	public static Vector<DiscreteProbabilityAssignment> generateProbabilityDistribution
		(Vector<Feature> features) {
		
		// usual case: the feature list contains at least one element
		if (features.size() > 0 ) {
			
			// call the recursive function "generateProbabilityDistribution" with an empty
			// list of assignments
			return generateProbabilityDistribution(features, 
					new Vector<DiscreteProbabilityAssignment>());
		}
		
		// if the list is empty
		else {
			Vector<DiscreteProbabilityAssignment> assignments = 
				new Vector<DiscreteProbabilityAssignment>();
			DiscreteProbabilityAssignment assignment = new DiscreteProbabilityAssignment();
			assignment.featurepairs = new FeatureValuePair[0];
			assignment.prob = 1.0f;
			assignments.add(assignment);
			return assignments;
		} 
	}
	
	
	/**
	 * Given a list of features and a list of previously computed assignments, generate
	 * a new list of assignments integrating both the previously computed assignments and
	 * the probabilities contained in the list of features
	 * 
	 * Example: if we have an input assignment {(colour, blue), (shape,spherical)}=0.6 and a 
	 * feature "location" with two possible values P(on_table) = 0.7 and P(on_shelf) = 0.2,
	 * the method will return the two following assignments:
	 * - {(colour, blue), (shape,spherical), (location, on_table) }=0.42
	 * - {(colour, blue), (shape,spherical), (location, on_shelf) }=0.14
	 * 
	 * @param features the list of features
	 * @param prevAssignments  the previously computed assignments
	 * @return the new assignments
	 */
	
	public static Vector<DiscreteProbabilityAssignment> generateProbabilityDistribution 
	(Vector<Feature> features, Vector<DiscreteProbabilityAssignment> prevAssignments) {

		Vector<DiscreteProbabilityAssignment> newAssignments = 
			new Vector<DiscreteProbabilityAssignment>();

		// remove one feature from the features list
		Feature feat = features.remove(0);

		if (prevAssignments.size() == 0) {

			for (int i = 0; i < feat.alternativeValues.length ; i++) {
				FeatureValue fv = feat.alternativeValues[i];
				DiscreteProbabilityAssignment newAss = new DiscreteProbabilityAssignment();
				newAss.featurepairs = new FeatureValuePair[1];
				newAss.featurepairs[0] = new FeatureValuePair(feat.featlabel, fv);
				newAss.prob = fv.independentProb;
				newAssignments.add(newAss);
			}
		}

		else {
			for (int i = 0; i < feat.alternativeValues.length ; i++) {
				FeatureValue fv = feat.alternativeValues[i];

				for (Enumeration<DiscreteProbabilityAssignment> e = 
					prevAssignments.elements() ; e.hasMoreElements(); ) {
					DiscreteProbabilityAssignment ass = e.nextElement();

					DiscreteProbabilityAssignment newAss = new DiscreteProbabilityAssignment();
					Vector<FeatureValuePair> featpairs = new Vector<FeatureValuePair>();

					for (int j = 0 ; j <ass.featurepairs.length ; j++) {
						featpairs.add(ass.featurepairs[j]);
					}
					FeatureValuePair newPair = new FeatureValuePair(feat.featlabel, fv);
					featpairs.add(newPair);
					newAss.featurepairs = new FeatureValuePair[featpairs.size()];
					newAss.featurepairs = featpairs.toArray(newAss.featurepairs);
					newAss.prob = ass.prob * fv.independentProb;
					newAssignments.add(newAss);
				}
			}
		}

		if (features.size() > 0) {
			return generateProbabilityDistribution(features, newAssignments);
		}
		else  {
			return newAssignments;
		}
	}


	
	public static CombinedProbabilityDistribution normaliseDistribution
	(CombinedProbabilityDistribution distrib, float probExists) {

		DiscreteProbabilityDistribution firstDistrib = 
			(DiscreteProbabilityDistribution) distrib.distributions[0];
		float total = 0.0f;
		for (int i = 0; i < firstDistrib.assignments.length; i++) {
			
			DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
					
			float probValue = ProbabilityUtils.getProbabilityValue(distrib, assignment);
			total += probValue;
		}
		
		for (int i = 0; i < firstDistrib.assignments.length; i++) {
			((DiscreteProbabilityDistribution)distrib.distributions[0]).assignments[i].prob = 
				firstDistrib.assignments[i].prob / (total * probExists);
		}
		
		return distrib;
	}
	
	
	public static ProbabilityDistribution normaliseDistribution 
		(ProbabilityDistribution distrib, float probExists) {
		
		if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return normaliseDistribution((CombinedProbabilityDistribution)distrib, probExists);
		}
		else if  (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return normaliseDistribution((DiscreteProbabilityDistribution)distrib, probExists);
		}
		
		return distrib;
	}
	
	

	public static DiscreteProbabilityDistribution normaliseDistribution
		(DiscreteProbabilityDistribution distrib, float probExists) {

		float total = 0.0f;
		if (distrib.assignments != null) {
			for (int i = 0; i < distrib.assignments.length; i++) {
				total += distrib.assignments[i].prob;
			}

			for (int i = 0; i < distrib.assignments.length; i++) {
				distrib.assignments[i].prob = distrib.assignments[i].prob / (total * probExists);
			}
		}
		return distrib;
	}




}
