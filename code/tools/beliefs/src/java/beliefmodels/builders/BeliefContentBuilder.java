
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


package beliefmodels.builders;


import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DiscreteDistribution;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FormulaProbPair;
import beliefmodels.autogen.distribs.NormalDistribution;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.UnknownValue;
import beliefmodels.autogen.logicalcontent.ElementaryFormula;
import beliefmodels.autogen.logicalcontent.Formula;
import beliefmodels.autogen.logicalcontent.NegatedFormula;
import beliefmodels.builders.FormulaBuilder;

public class BeliefContentBuilder {
	
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
			
	
	// =================================================
	// GENERAL METHODS FOR BELIEF CONTENT CONSTRUCTION
	// =================================================
 

	/**
	 * Create a belief content defined as a distribution with "exist" dependency (cf. slice specs)
	 * 
	 * @param probExist 
	 * 			the probability of existence of the belief
	 * @param contentDistrib 
	 * 			the distribution on the rest
	 * 
	 * @return a probability distribution with exist dependency
	 * @throws BeliefException 
	 * 			exception thrown is probExist=0 or contentDistrib is a null pointer
	 */
	public static DistributionWithExistDep createNewDistributionWithExistDep(
			float probExist, ProbDistribution contentDistrib) throws BeliefException {

		// checking the distribution is not null
		if (contentDistrib == null) {
			throw new BeliefException("error, distribution is a null pointer");
		}
		
		// building the existence distribution
		DiscreteDistribution existDistrib = createExistDistribution(probExist);
		
		// constructing the full distribution
		return createNewDistributionWithExistDep (existDistrib, contentDistrib);
	}

	
	
	
	/**
	 * Create a belief content defined as a distribution with "exist" dependency (cf. slice specs)
	 * 
	 * @param existDistrib 
	 * 			the distribution on the existence probability
	 * @param contentDistrib 
	 * 			the distribution on the rest
	 * 
	 * @return a probability distribution with exist dependency
	 * @throws BeliefException 
	 * 			exception thrown if one distribution is a null pointer
	 */
	public static DistributionWithExistDep createNewDistributionWithExistDep (
			ProbDistribution existDistrib, ProbDistribution contentDistrib) throws BeliefException {
	
		// checking the distributions are not null
		if (existDistrib == null || contentDistrib == null) {
			throw new BeliefException("error, distribution is a null pointer");
		}
		
		// creating the new distribution
		DistributionWithExistDep newDistrib = 
			new DistributionWithExistDep(existDistrib, contentDistrib);
		
		return newDistrib;
	}

	

	/**
	 * Create a new set of conditionally independent distributions
	 * 
	 * @return a new CondIndependentDistribs object
	 */
	public static CondIndependentDistribs createNewCondIndependentDistribs () {
		return new CondIndependentDistribs(new HashMap<String, ProbDistribution>());
	}
	
	
	/**
	 * Insert a new feature (feat label + set of alternative feature values) into a belief content
	 * 
	 * @param beliefcontent 
	 * 			the belief content, expressed as a probability distribution with existence dependency
	 * @param key
	 * 			the feature label 
	 * @param featDistrib
	 * 			the set of alternative feature values
	 * @throws BeliefException 
	 * 			exception thrown if distribution is a null pointer, 
	 * 			or if content distribution is not conditionally independent
	 * @post the belief content is updated with the new feature
	 */
	public static void putNewFeatureInBeliefContent(DistributionWithExistDep beliefcontent, String key, ProbDistribution featDistrib) throws BeliefException {
		
		if (beliefcontent == null) {
			throw new BeliefException("error, belief content is null");
		}
		else if (beliefcontent.Pc == null) {
			throw new BeliefException("error, content distribution in belief content is null");
		}
		else if (!(beliefcontent.Pc instanceof CondIndependentDistribs)) {
			throw new BeliefException("error, content distribution is not set to be conditionally independent");
		}
		putNewCondIndependentDistrib(((CondIndependentDistribs)beliefcontent.Pc), key, featDistrib);
	}
	
	
	/**
	 * Insert a new distribution to a set of conditionally independent distributions
	 * 
	 * @param distribs 
	 * 			the existing set of conditionally independent distributions
	 * @param newDistrib 
	 * 			the next distribution to add
	 * @param key
	 * 			the key of the distribution (i.e. the feature label)
	 * @throws BeliefException 
	 * 			exception thrown if distribs or newDistrib is a null pointer
	 * @post distribs now contains newDistrib
	 */
	public static void putNewCondIndependentDistrib(CondIndependentDistribs distribs, String key,
			ProbDistribution newDistrib) throws BeliefException {
		
		if (distribs == null || newDistrib == null) {
			throw new BeliefException("error, distribution is a null pointer");
		}
		else if (distribs.distribs == null) {
			throw new BeliefException ("error, distribution in distribs is a null pointer");
		}
		
		
		distribs.distribs.put(key, newDistrib);
//		ProbDistribution[] newDistribs = new ProbDistribution[distribs.distribs.length +1];
//		for (int i = 0; i < distribs.distribs.length ; i++) {
//			newDistribs[i] = distribs.distribs[i];
//		}
//		newDistribs[distribs.distribs.length] = newDistrib;
//		distribs.distribs = newDistribs;
	}

	
	/**
	 * Create a new discrete probability distribution out of a sequence of <formula,prob> pairs
	 * 
	 * @param pairs
	 * 			array of <form,prob> pairs
	 * @return a new, well-formed discrete probability distribution
	 * @throws BeliefException 
	 * 			if the <form,prob> pairs are not well-formed
	 */
	public static DiscreteDistribution createNewDiscreteDistribution (List<FormulaProbPair> pairs) throws BeliefException {
		
		if (pairs == null) {
			throw new BeliefException ("error, pairs is null");
		}
		
		else if (pairs.size() == 0) {
			throw new BeliefException ("error, no <form,prob> pair is provided");
		}
		else {
			float total = 0.0f;
			for (FormulaProbPair pair : pairs) {
				if (pair == null) {
					throw new BeliefException("error, pair is null");
				}
				else if (pair.form == null) {
					throw new BeliefException("error, form of pair is null");
				}
				total+= pair.prob;
			}
			
			if (total > 1.01) {
				throw new BeliefException("error, probabilities of discrete distribution sum up to: " + total);
			}
			else if (total < 0.99) {
				log("warning, probabilities sum up to: " + total);
			}
		}
		
		return new DiscreteDistribution(pairs);
	}
	
	
	
	public static FeatureValueDistribution createNewFeatureValueDistribution 
			(List<FeatureValueProbPair> values, boolean addUnknownValue) throws BeliefException {
		
		
		// Vector<FormulaProbPair> modalPairs = createModalFormulaPairs(feat, values);
		 
		if (values == null) {
			throw new BeliefException("error, values is null");
		}
		if (values.size() == 0) {
			throw new BeliefException("error, values.lengh == 0");
		}
		
		float total = 0.0f;

		for (FeatureValueProbPair value : values) {
			total += value.prob;
		}
		
		if (total > 1.01) {
			throw new BeliefException("error, probabilities of feature distribution sum up to: " + total);
		}
		
		else if (total < 0.99) {
			if (addUnknownValue) {
				debug("sum of probs is: " + total +", adding unknown value");
				FeatureValueProbPair uval = new FeatureValueProbPair(new UnknownValue(), 1- total);
				values.add(uval);
			}
			else {
				log("warning, probabilities sum up to: " + total);
			}
		}
		
		return new FeatureValueDistribution (values);
	}
	
	
	
	public static FormulaProbPair createNewFormulaProbPair(Formula form, float prob) throws BeliefException {
		if (form == null) {
			throw new BeliefException("error, form is null");
		}
		if (prob < 0 || prob > 1) {
			throw new BeliefException("error, prob < 0 or > 1");
		}
		return new FormulaProbPair(form, prob);
	}
	
		
	/**
	 * Create new discrete probability distribution with a unique <form, prob> pair
	 * 
	 * @param form
	 * 			the formula
	 * @param probForm
	 * 			the probability of the formula
	 * @return a new discrete probability distribution with a unique pair
	 * @throws BeliefException 
	 * 			if 
	 */
	public static DiscreteDistribution createNewDiscreteDistributionWithUniquePair (Formula form, float probForm) throws BeliefException {
		
		List<FormulaProbPair> pairs = new LinkedList<FormulaProbPair>();
		pairs.add(new FormulaProbPair(form, probForm));
		
		return createNewDiscreteDistribution(pairs);
	}

	
	/**
	 * Create a new continuous normal distribution (i.e. a Gaussian) with a particular feature, plus 
	 * the mean and variance parameter of the Gaussian
	 * @param feat 
	 * 			the feature for which the distribution applies
	 * @param mean
	 * 			the mean of the Gaussian
	 * @param variance
	 * 			the variance of the Gaussian
	 * @return a new, well-formed normal distribution
	 * @throws BeliefException 
	 */
	public static NormalDistribution createNewNormalDistribution (double mean, double variance) throws BeliefException {
		
		return new NormalDistribution(mean, variance);
	}
	
	
	
	// =================================================
	// PRIVATE METHODS
	// =================================================

	
	
	
	/**
	 * Create a distribution over existence property, based on the probability that it exists
	 * 
	 * @param probExist 
	 * 			probability >= 0 and <= 1
	 * @return the discrete distrbution
	 * @throws BeliefException
	 */
	private static DiscreteDistribution createExistDistribution (float probExist) throws BeliefException {
		
		if (probExist < 0.0 || probExist > 1) {
			throw new BeliefException("error, probExist is < 0 or > 1");
		}
		
		List<FormulaProbPair> existPairs = new LinkedList<FormulaProbPair>();
		
		ElementaryFormula existForm = FormulaBuilder.createNewExistFormula();
		existPairs.add(createNewFormulaProbPair(existForm, probExist));
		
		NegatedFormula notExistForm = FormulaBuilder.createNewNegatedFormula(existForm);
		existPairs.add(createNewFormulaProbPair(notExistForm, 1- probExist));
		
		return createNewDiscreteDistribution(existPairs);
	}
	
	
/**
	private static Vector<FormulaProbPair> createModalFormulaPairs (Feature feat, FormulaProbPair[] elpairs) throws BeliefException {
		if (feat == null) {
			throw new BeliefException("error, feat is null");
		}
		if (elpairs == null) {
			throw new BeliefException("error, elpairs is null");
		}
		if (elpairs.length == 0) {
			throw new BeliefException("error, elpairs.lengh == 0");
		}
		
		Vector<FormulaProbPair> modalPairs = new Vector<FormulaProbPair>();
		
		for (int i = 0; i < elpairs.length; i++) {
			FormulaProbPair pair = elpairs[i];
			
			if (pair == null) {
				throw new BeliefException ("error, form is null");
			}
			if (pair.form == null) {
				throw new BeliefException("error, pair.form is null");
			}
			if (pair.prob < 0 || pair.prob > 1) {
				throw new BeliefException("error, probability of pair < 0 or > 1");
			}
			
			ModalFormula newMForm = FormulaBuilder.createNewModalFormula(feat, pair.form);
			FormulaProbPair newPair = createNewFormulaProbPair(newMForm, pair.prob);
			modalPairs.add(newPair);
		}
		
		return modalPairs;
	}
	*/

	
	
//	/**
//	 * Add a new pair to the array
//	 * 
//	 * @param array the existing array
//	 * @param newEl the new element to add
//	 * @return the next, extended array
//	 */
//	private static FeatureValueProbPair[] addNewPairToArray (FeatureValueProbPair[] array, FeatureValueProbPair newEl) {
//		FeatureValueProbPair[] newArray = new FeatureValueProbPair[array.length + 1];
//		for (int i = 0 ; i < array.length ; i++) {
//			newArray[i] = array[i];
//		}
//		newArray[array.length] = newEl;
//		return newArray;
//	}
//	
	
	
	public static void log(String s) {
		if (LOGGING) {
			System.out.println("[BeliefContentBuilder] " + s);
		}
	}
	
	public static void debug(String s) {
		if (DEBUG) {
			System.out.println("[BeliefContentBuilder] " + s);
		}
	}

}
