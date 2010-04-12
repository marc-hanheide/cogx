
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
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.FormulaProbPair;
import beliefmodels.autogen.distribs.FormulaValues;
import beliefmodels.autogen.distribs.NormalValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.UnknownValue;
import beliefmodels.autogen.logicalcontent.Formula;

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
		
		// constructing the full distribution
		return createNewDistributionWithExistDep (probExist, contentDistrib);
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
	public static void putNewFeatureInBeliefContent(DistributionWithExistDep beliefcontent, String key, BasicProbDistribution featDistrib) throws BeliefException {
		
		if (beliefcontent == null) {
			throw new BeliefException("error, belief content is null");
		}
		else if (beliefcontent.Pc == null) {
			throw new BeliefException("error, content distribution in belief content is null");
		}
		else if (!(beliefcontent.Pc instanceof CondIndependentDistribs)) {
			throw new BeliefException("error, content distribution is not set to be conditionally independent");
		}
		putNewCondIndependentDistrib(((CondIndependentDistribs)beliefcontent.Pc), featDistrib);
	}
	
	
	
	/**
	 * Insert a new distribution to a set of conditionally independent distributions
	 * NB: the key/identifier to the distribution must be set in newDistrib
	 * 
	 * @param distribs 
	 * 			the existing set of conditionally independent distributions
	 * @param newDistrib 
	 * 			the next distribution to add
	 * @throws BeliefException 
	 * 			exception thrown if distribs or newDistrib is a null pointer
	 * @post distribs now contains newDistrib
	 */
	public static void putNewCondIndependentDistrib(CondIndependentDistribs distribs, BasicProbDistribution newDistrib) throws BeliefException {
		
		if (distribs == null || newDistrib == null) {
			throw new BeliefException("error, distribution is a null pointer");
		}
		else if (distribs.distribs == null) {
			throw new BeliefException ("error, distribution in distribs is a null pointer");
		}
		
		else if (newDistrib.key == null || newDistrib.key == "") {
			throw new BeliefException ("error, no key specified in the distribution");
		}
		
		distribs.distribs.put(newDistrib.key, newDistrib);

	}

	
	/**
	 * Create a new discrete probability distribution out of a sequence of <formula,prob> 
	 * pairs and a distribution identifier
	 * 
	 * @param pairs
	 * 			array of <form,prob> pairs
	 * @param id
	 * 			the distribution identifier (e.g. "content")
	 * @return a new, well-formed discrete probability distribution
	 * @throws BeliefException 
	 * 			if the <form,prob> pairs are not well-formed
	 */
	public static BasicProbDistribution createNewFormulaDistribution (String distribId, List<FormulaProbPair> pairs) throws BeliefException {
		
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
		
		FormulaValues formValues = new FormulaValues(pairs);
		
		return new BasicProbDistribution(distribId, formValues);
	}
	
	
	/**
	 * Create a new probability distribution out of a distribution identifier and a list of 
	 * <featvalue,prob> pairs
	 * 
	 * @param distribId
	 * 			the distribution identifier (i.e. the feature label)
	 * @param values
	 * 			set of <featvalue, prob> pairs
	 * @return the resulting probability distribution -- if the sum of probabilities is lower than
	 * 			1.0, the value "unknown" is added to the set of pairs to provide a well-formed distribution
	 * @pre	each probability must be >=0 and <= 1, and the sum of all probabilities must be <= 1
	 * @throws BeliefException
	 * 			if the inputs are not well-formed
	 */
	public static BasicProbDistribution createNewFeatureDistribution 
			(String distribId, List<FeatureValueProbPair> values) throws BeliefException {
		
				 
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
				debug("sum of probs is: " + total +", adding unknown value");
				FeatureValueProbPair uval = new FeatureValueProbPair(new UnknownValue(), 1- total);
				values.add(uval);
		}
		
		FeatureValues featvalues = new FeatureValues(values);
		
		return new BasicProbDistribution (distribId, featvalues);
	}
	
	
	/**
	 * Create a new <formula,prob> pair
	 * 
	 * @param form the formula
	 * @param prob its probability
	 * @pre the probability must be < 0 and > 1
	 * @return
	 * @throws BeliefException
	 * 			if formula is null or probability ill-formed
	 */
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
	 * @param distribId
	 * 			the distribution identifier (e.g. "content")
	 * @param probForm
	 * 			the probability of the formula
	 * @return a new discrete probability distribution with a unique pair
	 * @throws BeliefException 
	 * 			if 
	 */
	public static BasicProbDistribution createNewFormulaDistributionWithUniquePair (String distribId, Formula form, float probForm) throws BeliefException {
		
		List<FormulaProbPair> pairs = new LinkedList<FormulaProbPair>();
		pairs.add(new FormulaProbPair(form, probForm));
				
		return createNewFormulaDistribution(distribId, pairs);
	}

	
	/**
	 * Create a new continuous normal distribution (i.e. a Gaussian) with a particular feature, plus 
	 * the mean and variance parameter of the Gaussian
	 * @param distribId 
	 * 			the distribution identifier (i.e. the feature label)
	 * @param mean
	 * 			the mean of the Gaussian
	 * @param variance
	 * 			the variance of the Gaussian
	 * @return a new, well-formed normal distribution
	 * @throws BeliefException 
	 */
	public static BasicProbDistribution createNewNormalDistribution (String distribId, double mean, double variance) throws BeliefException {
		
		NormalValues nvalues = new NormalValues(mean, variance);
		
		return new BasicProbDistribution(distribId, nvalues);
	}
	
	
	
	// =================================================
	// UTILITY METHODS
	// =================================================

	
	
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
