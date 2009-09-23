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

public class GradientDescent {


	public static boolean ERRLOGGING = true;
	public static boolean LOGGING = false;
	
	public static HashMap<Union,Float> maxForUnions = new HashMap<Union,Float>();

	
	public static float getMaximum (PerceivedEntity entity) {

		float result = 0.0f;
		
		if (entity == null) {
			errlog("WARNING: entity == null, returning 0.0f");
			return 0.0f;
		}
		
		if (entity.distribution == null) {
			errlog("WARNING: distribution == null, regenerating");
			BinderUtils.addUnknownFeatureValues(entity.features);
			entity.distribution = ProbabilityUtils.generateProbabilityDistribution(entity);

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

	/**
public static Union getBestUnion(UnionDistribution distribution) {

	float maxValue = 0.0f;
	Union bestUnion = null;

	for (int i = 0; i < distribution.alternativeUnions.length; i++) {

		Union curUnion = distribution.alternativeUnions[i];
		float val = getMaximum(curUnion.distribution);
		if (val > maxValue) {
			maxValue = val;
			bestUnion = curUnion;
		}	
	}

	return bestUnion;
}
	 */


	synchronized public static boolean alreadyComputed(Union union) {
		for (Iterator<Union> i = maxForUnions.keySet().iterator() ; i.hasNext() ; ) {
			Union u = i.next();
			if (u.equals(union) && u.timeStamp == union.timeStamp) {
				return true;
			}
		}
		return false;  
	}
	
	
	

	public static void computeConfidenceScoresForUnionConfigurations
	(Vector<UnionConfiguration> configs) {

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

			double multiplication = 1.0f;
			for (int i = 0; i < config.includedUnions.length ; i++) {	
				Union union = config.includedUnions[i];
				float max = 0.0f;
				if (alreadyComputed(union)) {
					max = maxForUnions.get(union);
				}
				else {
					max = getMaximum(union);
					maxForUnions.put(union, max);
				}
				multiplication = multiplication * max;
				
			} 
			
			config.configProb = multiplication;
			log("configProb: " + config.configProb);
		}
		
	}
	
	
	public static Vector<UnionConfiguration> getNBestUnionConfigurations
	(Vector<UnionConfiguration> configs, int nb_nbests) {

		double threshold = 0.0f;
		Vector<UnionConfiguration> nbestConfigs = new Vector<UnionConfiguration>();
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();
			if (nbestConfigs.size() < nb_nbests) {

				nbestConfigs.add(config);

				if (config.configProb < threshold) {
					threshold = config.configProb;
				}
			}

			else {
				if (config.configProb > threshold) {
					UnionConfiguration worstinNBests = getWorstUnionConfiguration(nbestConfigs);
					nbestConfigs.remove(worstinNBests);
					nbestConfigs.add(config);
					UnionConfiguration secondworst = getWorstUnionConfiguration(nbestConfigs);
					threshold = secondworst.configProb;
				}
			}
		}

		return nbestConfigs;
	}

	
	public static UnionConfiguration getBestUnionConfiguration(Vector<UnionConfiguration> configs) {

		double maxAverage = -1.0f;
		UnionConfiguration bestConfig = null;

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

			if (config.configProb > maxAverage) {
				maxAverage = config.configProb;
				bestConfig = config;
			}
		}

		return bestConfig;
	}



	public static UnionConfiguration getWorstUnionConfiguration(Vector<UnionConfiguration> configs) {

		double minAverage = 99999.0f;
		UnionConfiguration worstConfig = null;

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

				if (config.configProb < minAverage) {
					minAverage = config.configProb;
					worstConfig = config;
				}
		}

		return worstConfig;
	}


	public static UnionConfiguration getBestUnionConfiguration
		(AlternativeUnionConfigurations configs) {
		Vector<UnionConfiguration> unionconfigsV = new Vector<UnionConfiguration>();
		for (int i = 0 ; i < configs.alterconfigs.length ; i++) {
			unionconfigsV.add(configs.alterconfigs[i]);
		}
		return getBestUnionConfiguration (unionconfigsV);
	}


	public static Union getUnionWithMaximumProbability (Union union) {

		if (union instanceof RelationUnion) {
			return getRelationUnionWithMaximumProbability((RelationUnion)union);
		}
		else {
			return getBasicUnionWithMaximumProbability(union);
		}
	}


	public static Union getBasicUnionWithMaximumProbability (Union union) {

		Union newUnion = new Union();
		newUnion.entityID = union.entityID;
		newUnion.features = new Feature[union.features.length];
		newUnion.timeStamp = union.timeStamp;
		
		DiscreteProbabilityAssignment bestAssign = null;

		if (union.distribution == null) {
			log("ERROR: distribution == null, aborting");
		}
		if (union.distribution.getClass().equals(DiscreteProbabilityDistribution.class)) {
			bestAssign = getBestAssignment((DiscreteProbabilityDistribution)union.distribution);

		}

		else if (union.distribution.getClass().equals(CombinedProbabilityDistribution.class)) {
			bestAssign = getBestAssignment((CombinedProbabilityDistribution)union.distribution);
		}

		else {
			log("Sorry, only discrete or combined feature distributions are handled right now");
			log("Used class: " + union.distribution.getClass());
		}

		for (int i = 0; i < union.features.length ; i++) {
			newUnion.features[i] = new Feature();
			newUnion.features[i].featlabel = union.features[i].featlabel;
			newUnion.features[i].alternativeValues = new FeatureValue[1];
			newUnion.features[i].alternativeValues[0] = 
				getBestFeatureValue(union.features[i], bestAssign);
		}

		newUnion.includedProxies = union.includedProxies;
		newUnion.probExists = union.probExists;
		newUnion.distribution = union.distribution;

		log("OK, extracted a new union with maximum probability");
		return newUnion;
	}


	private static FeatureValue getBestFeatureValue 
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


	public static RelationUnion getRelationUnionWithMaximumProbability (RelationUnion initRUnion) {

		Union bunion = getBasicUnionWithMaximumProbability(initRUnion);
		RelationUnion newRUnion = BinderUtils.convertIntoRelationUnion(bunion);

		newRUnion.source = new Feature();
		newRUnion.source.featlabel = initRUnion.source.featlabel;	
		newRUnion.source.alternativeValues = new FeatureValue[1];
		// INCORRECT - SHOULD CHANGE THIS
		newRUnion.source.alternativeValues[0] = initRUnion.source.alternativeValues[0];

		newRUnion.target = new Feature();
		newRUnion.target.featlabel = initRUnion.target.featlabel;	
		newRUnion.target.alternativeValues = new FeatureValue[1];
		// INCORRECT - SHOULD CHANGE THIS
		newRUnion.target.alternativeValues[0] = initRUnion.target.alternativeValues[0];
		
		return newRUnion;
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
