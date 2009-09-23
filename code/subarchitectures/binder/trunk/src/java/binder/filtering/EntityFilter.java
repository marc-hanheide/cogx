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

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Union;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.specialentities.RelationUnion;
import binder.utils.BinderUtils;
import binder.utils.FeatureValueUtils;


/**
 * Utility library to filter/collapse probability distributions in perceived entities
 * 
 * @author Pierre Lison
 * @version 23/09/2009
 * @started 05/09/2009
 */

public class EntityFilter {

	// flag to activate error logging
	public static boolean ERRLOGGING = true;

	// flag to activate logging
	public static boolean LOGGING = false;
	
	

	// ===================================================================
	// METHODS FOR FILTERING MAX PROBABILITY UNIONS
	// =================================================================== 

	
	// TODO: implement methods for filtering probability distributions in proxies and unions!
	
	
	// ===================================================================
	// METHODS FOR SELECTING MAX PROBABILITY UNIONS
	// =================================================================== 

	
	/**
	 * Select union instance with maximum probability -- i.e. collapse the probability
	 * distribution in the union onto a single-point union instance
	 * 
	 * @param union the union
	 * @return the collapsed union
	 */
	
	public static Union getUnionWithMaximumProbability (Union union) {

		// if the union is actually a relation union
		if (union instanceof RelationUnion) {
			return getRelationUnionWithMaximumProbability((RelationUnion)union);
		}
		
		// else, if it is a usual union
		else {
			return getBasicUnionWithMaximumProbability(union);
		}
	}

	
	/**
	 * Get union instance with maximum probability -- i.e. collapse the probability
	 * distribution in the union onto a single-point union instance
	 * 
	 * @param union the union
	 * @return the collapsed union
	 */
	
	public static Union getBasicUnionWithMaximumProbability (Union union) {

		// create a new union
		Union newUnion = new Union();
		newUnion.entityID = union.entityID;
		newUnion.features = new Feature[union.features.length];
		newUnion.timeStamp = union.timeStamp;

		// verify the distribution exists
		if (union.distribution == null) {
			errlog("ERROR: distribution == null, aborting");
			return newUnion;
		}
			
		// Extract the best assignment in the union distribution
		DiscreteProbabilityAssignment bestAssign = 
			MaximumSearch.getBestAssignment(union.distribution);

		// loop on the union features
		for (int i = 0; i < union.features.length ; i++) {
			newUnion.features[i] = new Feature();
			newUnion.features[i].featlabel = union.features[i].featlabel;
			
			// create a single feature value containing the best value 
			// (specified in the assignment)
			newUnion.features[i].alternativeValues = new FeatureValue[1];
			newUnion.features[i].alternativeValues[0] = 
					getFeatureValueIncludedInAssignment(union.features[i], bestAssign);
		}

		// finish specifiying the new union
		newUnion.includedProxies = union.includedProxies;
		newUnion.probExists = union.probExists;
		newUnion.distribution = union.distribution;

		log("OK, extracted a new union with maximum probability");
		return newUnion;
	}


	/**
	 * Select relation union instance with maximum probability -- i.e. collapse the probability
	 * distribution in the union onto a single-point union instance
	 * 
	 * TODO: integrate source and targets in the search for best assignments
	 * 
	 * @param initRUnion the relation union
	 * @return the collapsed relation union
	 */
	
	public static RelationUnion getRelationUnionWithMaximumProbability (RelationUnion initRUnion) {

		// create a collapsed basic union, and convert it into a relation union
		Union bunion = getBasicUnionWithMaximumProbability(initRUnion);
		RelationUnion newRUnion = BinderUtils.convertIntoRelationUnion(bunion);

		// specify the source of the relation union
		newRUnion.source = new Feature();
		newRUnion.source.featlabel = initRUnion.source.featlabel;	
		newRUnion.source.alternativeValues = new FeatureValue[1];
		
		newRUnion.source.alternativeValues[0] = initRUnion.source.alternativeValues[0];

		// specify the target of the relation union
		newRUnion.target = new Feature();
		newRUnion.target.featlabel = initRUnion.target.featlabel;	
		newRUnion.target.alternativeValues = new FeatureValue[1];

		newRUnion.target.alternativeValues[0] = initRUnion.target.alternativeValues[0];

		return newRUnion;
	}
	
	
	// ===================================================================
	// UTILITY METHODS
	// =================================================================== 

	
	/**
	 * return true if the feature value pair is included in the assignment, false otherwise
	 * 
	 * @param pair the feature value pair
	 * @param assign the assigment
	 * @return true if the pair is included, false otherwise
	 */

	private static boolean isFeatValuePairInAssignment 
	(FeatureValuePair pair, DiscreteProbabilityAssignment assign) {
		
		// loop on the feat-value pairs in the assignment
		for (int i = 0; i < assign.featurepairs.length ; i++) {
			FeatureValuePair pair2 = assign.featurepairs[i];

			if (pair.featlabel.equals(pair2.featlabel) && 
					(FeatureValueUtils.haveEqualValue(pair.featvalue, pair2.featvalue))) {
				return true;
			}
		}
		return false;
	}

 
	
	/**
	 * Given a feature and an assignment function, extract the specific feature value in the
	 * feature which is included in the assignment
	 * 
	 * If no feature value in the feature is to be found in the assignment, return an empty
	 * new feature value.
	 * 
	 * @param feat the feature
	 * @param bestAssign the assignment
	 * @return the feature value included in the feature (if present)
	 */
	
	private static FeatureValue getFeatureValueIncludedInAssignment 
	(Feature feat, DiscreteProbabilityAssignment bestAssign) {

		boolean isFound = false;

		// loop on the feature values in the feature
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
			System.out.println("[MaximumSearch] " + s);
	}

	public static void errlog(String s) {
		if (ERRLOGGING)
			System.err.println("[MaximumSearch] " + s);
	}
	
}
