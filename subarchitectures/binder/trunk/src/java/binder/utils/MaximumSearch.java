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
import java.util.HashMap;
import java.util.Iterator;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.specialentities.RelationUnion;
import binder.utils.ProbabilityUtils;

public class MaximumSearch {


	public static boolean ERRLOGGING = true;
	public static boolean LOGGING = false;
	
	public static HashMap<PerceivedEntity,Float> maxForEntities = new HashMap<PerceivedEntity,Float>();

	
	public static float getMaximum (PerceivedEntity entity) {

		
		if (entity == null) {
			errlog("WARNING: entity == null, returning 0.0f");
			return 0.0f;
		}
		
		else if (entity.distribution == null) {
			errlog("WARNING: distribution == null, regenerating");
			BinderUtils.addUnknownFeatureValues(entity.features);
			entity.distribution = ProbabilityUtils.generateProbabilityDistribution(entity);

		}
		
		
		if (alreadyComputed(entity)) {
			return maxForEntities.get(entity);
		}
		
		else {
			float max = getMaximum(entity.distribution);
			maxForEntities.put(entity, max);
		}
		
		return getMaximum (entity.distribution);
		
	}

	public static float getMaximum (ProbabilityDistribution distrib) {
		float result = 0.0f;
		
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getMaximum((DiscreteProbabilityDistribution) distrib);
		}
 
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			
			return getMaximum((CombinedProbabilityDistribution) distrib);
		}

		else {
			errlog("Sorry, only discrete or combined feature distributions are handled right now");
			log("Used class: " + distrib.getClass());
		}

		return result;
	}

	public static float getMaximum (DiscreteProbabilityDistribution distrib) {

		float maxProb = 0.0f;
		if (distrib.assignments != null) {
			for (int i = 0 ; i <distrib.assignments.length ; i++) {
				DiscreteProbabilityAssignment assignment = distrib.assignments[i];
				log(BinderUtils.getPrettyPrintProbabilityAssignment(assignment));
				if (assignment.prob > maxProb) {
					maxProb = assignment.prob;
				}
			}
		}
		return maxProb;
	}




	public static float getMaximum (CombinedProbabilityDistribution distrib) {

		//	log("Searching maximum value for a combined probability distribution...");
		float maxProb = 0.0f;

		DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;

		DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];
		float total = 0.0f;
		
		for (int i = 0; i < firstDistrib.assignments.length; i++) {

			DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
			float probValue = ProbabilityUtils.getProbabilityValue(distrib, assignment);
			//	log("ass: " + ProbabilityUtils.getDiscreteProbabilityAssignmentPrettyPrint(assignment) + " --> " + probValue);

			total += probValue;
			if (probValue > maxProb) {
				maxProb = probValue;			
			}

		}

		return maxProb;

	}


	public static DiscreteProbabilityAssignment 
		getBestAssignment (CombinedProbabilityDistribution distrib) {


		log("Searching maximum value for a combined probability distribution...");
		float maxProb = 0.0f;

		DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;

		DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];
		float total = 0.0f;
		for (int i = 0; i < firstDistrib.assignments.length; i++) {

			DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
			float probValue = ProbabilityUtils.getProbabilityValue(distrib, assignment);
			//	log("ass: " + ProbabilityUtils.getDiscreteProbabilityAssignmentPrettyPrint(assignment) + " --> " + probValue);

			total += probValue;
			if (probValue > maxProb) {
				maxProb = probValue;
				bestAssign = assignment;	
			}
		}

		log("Best assignment: " + BinderUtils.getPrettyPrintProbabilityAssignment(bestAssign));

		return bestAssign;

	}



	public static DiscreteProbabilityAssignment getBestAssignment  
	(DiscreteProbabilityDistribution distrib) {

		float maxProb = 0.0f;
		DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;
		if (distrib.assignments != null) {
			for (int i = 0 ; i <distrib.assignments.length ; i++) {
				DiscreteProbabilityAssignment assignment = distrib.assignments[i];
				if (assignment.prob > maxProb) {
					maxProb = assignment.prob;
					bestAssign = assignment;
				}
			}
		}
		return bestAssign;
	}
	

	private static boolean isFeatValuePairInAssignment 
	(FeatureValuePair pair, DiscreteProbabilityAssignment assign) {
		for (int i = 0; i < assign.featurepairs.length ; i++) {
			FeatureValuePair pair2 = assign.featurepairs[i];

			if (pair.featlabel.equals(pair2.featlabel) && 
					(FeatureValueUtils.haveEqualValue(pair.featvalue, pair2.featvalue))) {
				return true;
			}
		}
		return false;
	}



	synchronized public static boolean alreadyComputed (PerceivedEntity entity) {
		for (Iterator<PerceivedEntity> i = maxForEntities.keySet().iterator() ; i.hasNext() ; ) {
			PerceivedEntity u = i.next();
			if (u.equals(entity) && u.timeStamp == entity.timeStamp) {
				return true;
			}
		}
		return false;  
	}
	
	
	
	

	public static FeatureValue getBestFeatureValue 
		(Feature feat, DiscreteProbabilityAssignment bestAssign) {

		boolean isFound = false;

		for (int j = 0 ; j < feat.alternativeValues.length ; j++) {
			FeatureValuePair pair =  new FeatureValuePair (feat.featlabel,feat.alternativeValues[j]);
			if (isFeatValuePairInAssignment (pair, bestAssign)) {
				isFound = true;
				return feat.alternativeValues[j];
			}
		}
		if (!isFound) {
			log("WARNING: best assignment NOT found in the feature structure: " +
					BinderUtils.getPrettyPrintProbabilityAssignment(bestAssign));
		}
		return new FeatureValue();
	}


	public static void log(String s) {
		if (LOGGING)
		System.out.println("[GradientDescent] " + s);
	}
	
	public static void errlog(String s) {
		if (ERRLOGGING)
		System.err.println("[GradientDescent] " + s);
	}
}
