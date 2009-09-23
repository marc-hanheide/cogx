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
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.utils.FeatureValueUtils;

public class ProbabilityUtils {

	


	public static Vector<DiscreteProbabilityAssignment> getAssignmentsIncludingSmallerAssignment(
			DiscreteProbabilityDistribution distrib, DiscreteProbabilityAssignment smallassign) {

		Vector<DiscreteProbabilityAssignment> assignments = new Vector<DiscreteProbabilityAssignment>();

		for (int i =0; i < distrib.assignments.length; i++) {

			DiscreteProbabilityAssignment bigassign = distrib.assignments[i];
			
			if (containsAll(bigassign, smallassign)) {
				assignments.add(bigassign);
			}
		}

		return assignments;
	}



	

	public static float getMarginalProbabilityValue
		(ProbabilityDistribution distrib, FeatureValuePair pair) {
		float result = 0.0f;
		
		DiscreteProbabilityAssignment smallassignment = new DiscreteProbabilityAssignment();
		smallassignment.featurepairs = new FeatureValuePair[1];
		smallassignment.featurepairs[0] = pair;
		
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			DiscreteProbabilityDistribution distrib2 = (DiscreteProbabilityDistribution) distrib;
			
			Vector<DiscreteProbabilityAssignment> assignments = 
				getAssignmentsIncludingSmallerAssignment(distrib2, smallassignment);
			
			for (Enumeration<DiscreteProbabilityAssignment> enu = 
				assignments.elements() ; enu.hasMoreElements() ; ) {
				result += enu.nextElement().prob;
			}
		}
		
		if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			CombinedProbabilityDistribution distrib2 = (CombinedProbabilityDistribution) distrib;
			
			
			if (distrib2.distributions.length > 0) {
			DiscreteProbabilityDistribution firstdistrib = 
				(DiscreteProbabilityDistribution) distrib2.distributions[0];
			
			Vector<DiscreteProbabilityAssignment> assignments = 
				getAssignmentsIncludingSmallerAssignment(firstdistrib, smallassignment);
			
			if (assignments.size() > 0) {
				
				for (Enumeration<DiscreteProbabilityAssignment> enu = 
					assignments.elements() ; enu.hasMoreElements() ; ) {
					DiscreteProbabilityAssignment assign = enu.nextElement();
				//	log("ass: " + getDiscreteProbabilityAssignmentPrettyPrint(assign));
					float partialresult = getProbabilityValue(distrib2, assign);
					result += partialresult;
				}
			}
			
			}
		}
		
		return result;
	}
	


	public static float getProbabilityValue
		(ProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		
		if (distrib == null) {
			log("ERROR, distribution is null");
		}
		
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getProbabilityValue((DiscreteProbabilityDistribution)distrib, assignment);
		}
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return getProbabilityValue((CombinedProbabilityDistribution)distrib, assignment);
		}
		else {
			log("sorry, only discrete and combined probability distribution are supported at the moment!");
			return 0.0f;
		}
	}

	public static float getProbabilityValue 
		(CombinedProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		float result = 1.0f;

		for (int i = 0; i < distrib.distributions.length; i++) {
			ProbabilityDistribution subdistrib = distrib.distributions[i];
			if (subdistrib == null) {
				log("ERROR, distribution is null");
			}
			else if (distrib.opType.equals(OperationType.MULTIPLIED)) {
				float subdistribresult = getProbabilityValue(subdistrib, assignment); 
				result = result * subdistribresult;
			}
		}

		return result;
	}

	public static DiscreteProbabilityDistribution multiplyDistributionWithConstantValue (
			DiscreteProbabilityDistribution distrib, float constantValue) {

		DiscreteProbabilityDistribution newDistrib = new DiscreteProbabilityDistribution();
		newDistrib.assignments = new DiscreteProbabilityAssignment[distrib.assignments.length];

		for (int i = 0; i < distrib.assignments.length; i++) {
			newDistrib.assignments[i] = new DiscreteProbabilityAssignment();
			newDistrib.assignments[i].featurepairs = distrib.assignments[i].featurepairs;
			newDistrib.assignments[i].prob = distrib.assignments[i].prob * constantValue;
		}

		return newDistrib;
	}


	public static DiscreteProbabilityDistribution invertDistribution (
			DiscreteProbabilityDistribution distrib) {

		DiscreteProbabilityDistribution newDistrib = new DiscreteProbabilityDistribution();
		newDistrib.assignments = new DiscreteProbabilityAssignment[distrib.assignments.length];
		
		for (int i = 0; i < distrib.assignments.length; i++) {
			if (distrib.assignments[i].prob > 0) {
				newDistrib.assignments[i] = new DiscreteProbabilityAssignment();
				newDistrib.assignments[i].featurepairs = distrib.assignments[i].featurepairs;
				newDistrib.assignments[i].prob = 1 / distrib.assignments[i].prob ;
			}
		}

		return newDistrib;
	}

	public static float getProbabilityValue 
	(DiscreteProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		float result = 0.0f;

		if (distrib.assignments != null) {
		for (int i = 0; i < distrib.assignments.length; i++) {
			DiscreteProbabilityAssignment curAssignment = distrib.assignments[i];
			if (containsAll(assignment, curAssignment)) {
				return curAssignment.prob;
			}
		}
		}
		else {
			log("ERROR, distribution contains no assignment");
		}

		log("WARNING, no probability value found for assignment: " + 
				BinderUtils.getPrettyPrintProbabilityAssignment(assignment));
		log("distrib: " + BinderUtils.getPrettyPrintProbabilityDistribution(distrib));
		return result;
	}


	public static boolean containsAll
	(DiscreteProbabilityAssignment bigassign, DiscreteProbabilityAssignment smallassign) {
		
		Vector<FeatureValuePair> pairs1 = new Vector<FeatureValuePair>();
		for (int j = 0; j < bigassign.featurepairs.length; j++) {
			pairs1.add(bigassign.featurepairs[j]);
		}
		
		Vector<FeatureValuePair> pairs2 = new Vector<FeatureValuePair>();
		for (int j = 0; j < smallassign.featurepairs.length; j++) {
			pairs2.add(smallassign.featurepairs[j]);
		}
		
		return containsAll(pairs1, pairs2);	
	}
	
	

	public static boolean containsAll(Vector<FeatureValuePair> pairs1, 
			Vector<FeatureValuePair> pairs2) {
		boolean result = true;
		
		for (Enumeration<FeatureValuePair> e = pairs2.elements() ; e.hasMoreElements(); ) {
			FeatureValuePair pair2 = e.nextElement();
			boolean foundMatch = false;
			for (Enumeration<FeatureValuePair> f = pairs1.elements() ; 
			f.hasMoreElements()  && !foundMatch; ) {
				FeatureValuePair pair1 = f.nextElement();
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

	public static void log(String s) {
		System.out.println("[ProbabilityUtils] " + s);
	}

}
