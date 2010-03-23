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


package binder.builders;

import java.util.Enumeration;
import java.util.Vector;

import binder.arch.BinderException;
import binder.autogen.Feature;
import binder.autogen.distribs.CondIndependentDistribs;
import binder.autogen.distribs.DiscreteDistribution;
import binder.autogen.distribs.DistributionWithExistDep;
import binder.autogen.distribs.FormulaProbPair;
import binder.autogen.distribs.NormalDistribution;
import binder.autogen.distribs.ProbDistribution;
import binder.autogen.formulae.ElementaryFormula;
import binder.autogen.formulae.Formula;
import binder.autogen.formulae.ModalFormula;
import binder.autogen.formulae.NegatedFormula;


public class BeliefContentBuilder {

	
	public static boolean LOGGING_HIGHPRIORITY = true;
	
	public static boolean LOGGING_LOWPRIORITY = true;
		
	
	/**
	 * Create a belief content defined as a distribution with "exist" dependency (cf. slice specs)
	 * 
	 * @param existDistrib 
	 * 			the distribution on the existence probability
	 * @param contentDistrib 
	 * 			the distribution on the rest
	 * 
	 * @return a probability distribution with exist dependency
	 * @throws BinderException 
	 * 			exception thrown if one distribution is a null pointer
	 */
	public DistributionWithExistDep createBeliefContentWithExistDep(
			ProbDistribution existDistrib, ProbDistribution contentDistrib) throws BinderException {
	
		if (existDistrib == null || contentDistrib == null) {
			throw new BinderException("error, distribution is a null pointer");
		}
		
			DistributionWithExistDep newDistrib = new DistributionWithExistDep(existDistrib, contentDistrib);
		
		return newDistrib;
	}

	
	
	/**
	 * Create a belief content defined as a distribution with "exist" dependency (cf. slice specs)
	 * 
	 * @param probExist 
	 * 			the probability of existence of the belief
	 * @param contentDistrib 
	 * 			the distribution on the rest
	 * 
	 * @return a probability distribution with exist dependency
	 * @throws BinderException 
	 * 			exception thrown is probExist=0 or contentDistrib is a null pointer
	 */
	public DistributionWithExistDep createBeliefContentWithExistDep(
			float probExist, ProbDistribution contentDistrib) throws BinderException {

		if (contentDistrib == null) {
			throw new BinderException("error, distribution is a null pointer");
		}
		
		DiscreteDistribution existDistrib = createExistDistribution(probExist);
		
		return createBeliefContentWithExistDep (existDistrib, contentDistrib);
	}

	
	/**
	 * Create a distribution over existence property, based on the probability that it exists
	 * 
	 * @param probExist 
	 * 			probability >= 0 and <= 1
	 * @return the discrete distrbution
	 * @throws BinderException
	 */
	private DiscreteDistribution createExistDistribution (float probExist) throws BinderException {
		
		if (probExist < 0.0 || probExist > 1) {
			throw new BinderException("error, probExist is < 0 or > 1");
		}
		
		FormulaProbPair[] existPairs = new FormulaProbPair[2];
		
		ElementaryFormula existForm = FormulaBuilder.createNewElementaryFormulaForExist();
		existPairs[0] = createNewFormulaProbPair(existForm, probExist);
		
		NegatedFormula notExistForm = FormulaBuilder.createNewNegatedFormula(existForm);
		existPairs[1] = createNewFormulaProbPair(notExistForm, 1- probExist);
		
		return createNewDiscreteDistribution(existPairs);
	}
	
	
	
	/**
	 * Create a new (empty) set of conditionally independent distributions
	 * 
	 * @return a new set of conditionally independent distributions
	 */
	public ProbDistribution createNewDistribution() {
		return new ProbDistribution();
	}


	
	
	/**
	 * Add a new distribution to a set of conditionally independent distributions
	 * 
	 * @param distribs 
	 * 			the existing set of conditionally independent distributions
	 * @param newDistrib 
	 * 			the next distribution to add
	 * @throws BinderException 
	 * 			exception thrown if distribs or newDistrib is a null pointer
	 * @post distribs now contains newDistrib
	 */
	public void addCondIndependentDistrib(CondIndependentDistribs distribs,
			ProbDistribution newDistrib) throws BinderException {
		
		if (distribs == null || newDistrib == null) {
			throw new BinderException("error, distribution is a null pointer");
		}
		else if (distribs.distribs == null) {
			throw new BinderException ("error, distribution in distribs is a null pointer");
		}
		
		ProbDistribution[] newDistribs = new ProbDistribution[distribs.distribs.length +1];
		for (int i = 0; i < distribs.distribs.length ; i++) {
			newDistribs[i] = distribs.distribs[i];
		}
		newDistribs[distribs.distribs.length] = newDistrib;
		distribs.distribs = newDistribs;
	}

	
	/**
	 * Create a new discrete probability distribution out of a sequence of <formula,prob> pairs
	 * 
	 * @param pairs
	 * 			array of <form,prob> pairs
	 * @return a new, well-formed discrete probability distribution
	 * @throws BinderException 
	 * 			if the <form,prob> pairs are not well-formed
	 */
	public static DiscreteDistribution createNewDiscreteDistribution (FormulaProbPair[] pairs) throws BinderException {
		
		if (pairs == null) {
			throw new BinderException ("error, pairs is null");
		}
		
		else if (pairs.length == 0) {
			throw new BinderException ("error, no <form,prob> pair is provided");
		}
		else {
			float total = 0.0f;
			for (int i = 0 ; i < pairs.length ; i++) {
				if (pairs[i] == null) {
					throw new BinderException("error, pair["+i+"] is null");
				}
				else if (pairs[i].form == null) {
					throw new BinderException("error, form of pair["+i+"] is null");
				}
				total+= pairs[i].prob;
			}
			
			if (total > 1.01) {
				throw new BinderException("error, probabilities of discrete distribution sum up to: " + total);
			}
			else if (total < 0.99) {
				log_high("warning, probabilities sum up to: " + total);
			}
		}
		
		return new DiscreteDistribution(pairs);
	}
	
	
	public static Vector<FormulaProbPair> createModalFormulaPairs (Feature feat, FormulaProbPair[] elpairs) throws BinderException {
		if (feat == null) {
			throw new BinderException("error, feat is null");
		}
		if (elpairs == null) {
			throw new BinderException("error, elpairs is null");
		}
		if (elpairs.length == 0) {
			throw new BinderException("error, elpairs.lengh == 0");
		}
		
		Vector<FormulaProbPair> modalPairs = new Vector<FormulaProbPair>();
		
		for (int i = 0; i < elpairs.length; i++) {
			FormulaProbPair pair = elpairs[i];
			
			if (pair == null) {
				throw new BinderException ("error, form is null");
			}
			if (pair.form == null) {
				throw new BinderException("error, pair.form is null");
			}
			if (pair.prob < 0 || pair.prob > 1) {
				throw new BinderException("error, probability of pair < 0 or > 1");
			}
			
			ModalFormula newMForm = FormulaBuilder.createNewModalFormula(feat, pair.form);
			FormulaProbPair newPair = createNewFormulaProbPair(newMForm, pair.prob);
			modalPairs.add(newPair);
		}
		
		return modalPairs;
	}
	
	public static DiscreteDistribution createNewDiscreteDistributionOverFeature 
			(Feature feat, FormulaProbPair[] elpairs, boolean addUnknownValue) throws BinderException {
		
		
		Vector<FormulaProbPair> modalPairs = createModalFormulaPairs(feat, elpairs);
	
		float total = 0.0f;

		for (Enumeration<FormulaProbPair> e = modalPairs.elements(); e.hasMoreElements() ;) {
			FormulaProbPair pair = e.nextElement();
			total += pair.prob;
		}
		
		if (total > 1.01) {
			throw new BinderException("error, probabilities of discrete distribution sum up to: " + total);
		}
		
		else if (total < 0.99) {
			if (addUnknownValue) {
				log_low("sum of probs is: " + total +", adding unknown value");
				FormulaProbPair uPair = createNewFormulaProbPairForUnknownValue (feat, 1- total);
				modalPairs.add(uPair);
			}
			else {
				log_high("warning, probabilities sum up to: " + total);
			}
		}
		
		return createNewDiscreteDistribution(convertVectorToArray(modalPairs));
	}
	
	
	public static FormulaProbPair createNewFormulaProbPairForUnknownValue(Feature feat, float prob) throws BinderException {
		ModalFormula uForm = FormulaBuilder.createModalFormulaWithUnknownValue(feat);
		return createNewFormulaProbPair(uForm, prob);
	}
	
	
	private static FormulaProbPair[] convertVectorToArray (Vector<FormulaProbPair> pairsV) {
		FormulaProbPair[] pairsA = new FormulaProbPair[pairsV.size()]; 
		return pairsV.toArray(pairsA);
	}
	
	public static FormulaProbPair createNewFormulaProbPair(Formula form, float prob) throws BinderException {
		if (form == null) {
			throw new BinderException("error, form is null");
		}
		if (prob < 0 || prob > 1) {
			throw new BinderException("error, prob < 0 or > 1");
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
	 * @throws BinderException 
	 * 			if 
	 */
	public DiscreteDistribution createNewDiscreteDistributionWithUniquePair (Formula form, float probForm) throws BinderException {
		
		FormulaProbPair[] pairs = new FormulaProbPair[1];
		pairs[0] = new FormulaProbPair(form, probForm);
		
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
	 * @throws BinderException 
	 */
	public NormalDistribution createNewNormalDistribution (Feature feat, double mean, double variance) throws BinderException {
		
		if (feat == null) {
			throw new BinderException("error, feat == null");
		}
		
		return new NormalDistribution(feat, mean, variance);
	}
	
	
	
	public static void log_low(String s) {
		if (LOGGING_LOWPRIORITY) {
			System.out.println("[BeliefContentBuilder] " + s);
		}
	}
	
	public static void log_high(String s) {
		if (LOGGING_HIGHPRIORITY) {
			System.out.println("[BeliefContentBuilder] " + s);
		}
	}

}
