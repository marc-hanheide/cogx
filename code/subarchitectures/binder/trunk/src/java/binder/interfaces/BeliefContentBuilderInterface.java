
// =================================================================                                                        
// Copyright (C) 2010-2012 Pierre Lison (plison@dfki.de)                                                                
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
 

package binder.interfaces;

import binder.arch.BinderException;
import binder.autogen.distribs.CondIndependentDistribs;
import binder.autogen.distribs.DiscreteDistribution;
import binder.autogen.distribs.DistributionWithExistDep;
import binder.autogen.distribs.FormulaProbPair;
import binder.autogen.distribs.NormalDistribution;
import binder.autogen.distribs.ProbDistribution;
import binder.autogen.featurecontent.Feature;
import binder.autogen.logicalcontent.Formula;

public interface BeliefContentBuilderInterface {

	
	/**
	 * Create a belief content defined as an empty probability distribution
	 * 
	 * @return an empty probability distribution
	 */
	public ProbDistribution createNewDistribution ();
	
	
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
	public DistributionWithExistDep createBeliefContentWithExistDep (ProbDistribution existDistrib, ProbDistribution contentDistrib) throws BinderException;

	
	
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
	public DistributionWithExistDep createBeliefContentWithExistDep (float probExist, ProbDistribution contentDistrib) throws BinderException;

	
	/**
	 * Create a new (empty) set of conditionally independent distributions
	 * 
	 * @return a new set of conditionally independent distributions
	 */
	public CondIndependentDistribs createNewCondIndependentDistribs ();
	
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
	public void addCondIndependentDistrib (CondIndependentDistribs distribs, ProbDistribution newDistrib) throws BinderException;

	
	/**
	 * Create a new discrete probability distribution out of a sequence of <formula,prob> pairs
	 * 
	 * @param pairs
	 * 			array of <form,prob> pairs
	 * @return a new, well-formed discrete probability distribution
	 */
	public DiscreteDistribution createNewDiscreteDistribution (FormulaProbPair[] pairs);
	
	
	/**
	 * Create new discrete probability distribution with a unique <form, prob> pair
	 * 
	 * @param form
	 * 			the formula
	 * @param probForm
	 * 			the probability of the formula
	 * @return a new discrete probability distribution with a unique pair
	 */
	public DiscreteDistribution createNewDiscreteDistributionWithUniquePair (Formula form, float probForm);

	
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
	 */
	public NormalDistribution createNewNormalDistribution (Feature feat, double mean, double variance);
	
	
}