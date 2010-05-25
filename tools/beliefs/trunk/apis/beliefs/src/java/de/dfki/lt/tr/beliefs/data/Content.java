// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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

// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.beliefs.data;

//=================================================================
//IMPORTS

//Java
import java.util.HashMap;
import java.util.List;
import java.util.LinkedList;

//Belief API slice
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;


//Belief API util 
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;
import de.dfki.lt.tr.beliefs.util.BeliefNotInitializedException;

/**
 * The <tt>Content</tt> class provides all the principal methods for operating on the content of a situated, multi-agent belief. 
 * The basic underlying structure is a collection of one or more probabilistic distributions. This collection can be provided
 * directly to set up the content, or it can be constructed. 
 * <p>
 * The class provides methods for constructing several types of distributions, including basic distributions from 
 * (formula, probability) pairs and dictionaries of conditionally independent distributions. These distributions can
 * be constructed as externally used objects. In addition, the class provides direct methods for initializing a
 * content object with a dictionary of conditionally independent distributions, and accessing this dictionary directly 
 * (rather than through externally constructed objects).
 * <p>
 * The following creates a basic probability distribution: 
 * <pre>
			LinkedList fpList = new LinkedList<FormulaProbPair>();
 * 			Formula fp1 = new Formula();
			fp1.init();
			fp1.setId(1);
			fp1.asProposition("prop1");   // set the proposition 
			fp1.setProbability(0.3f);	  // set the probability
			fpList.add(fp1.getAsPair());  // get the (formula, probability) as pair, and add
			Formula fp2 = new Formula();
			fp2.init();
			fp2.setId(2);
			fp2.asProposition("prop2");
			fp2.setProbability(0.6f);
			fpList.add(fp2.getAsPair());
			FormulaValues formulaValues = new FormulaValues(fpList);
			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
 * </pre><p>
 * 
 * We can then create a dictionary of conditionally independent distributions, and add the above one: 
 * 
 * <pre>
 * 			// create a dictionary of conditionally independent distributions
			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
			// add the new distribution directly to the dictionairy of conditionally independent dists
			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
 * </pre><p>
 * 
 * The dictionary can be set as the distribution in the content object using the <tt>setDictionary</tt> method. 
 * This then allows us to add more distributions, directly through the content object: 
 * 
 * <pre>
 * 			// now create another base distribution, 
			BasicProbDistribution baseDist2 = Content.createBasicDistribution("formula2", formulaValues);
			// add it to the content directly
			content.addDistributionToDictionary(baseDist2);
 * </pre> <p>
 * 
 * The class also provides a method <tt>getDistributionFromDictionary</tt> for immediately retrieving a distribution, 
 * whereas <tt>removeDistributionFromDictionary</tt> removes a distribution by key. 
 * <p>
 * In principle these methods make it possible to create content around a dictionary without having to construct that 
 * dictionary as external object at all. By initializing the content structure, and then calling <tt>asDictionary</tt> 
 * we can directly apply <tt>add/get/removeDistributionFromDictionary</tt>. 
 *
 * <pre>
 * 			Content content = new Content();
 * 			content.init();
 * 			content.asDictionary();
 * 			// construct baseDist 
 *			// ... 
 * 			content.addConditionallyIndependentDistribution(ciDists,baseDist);
 * 			// construct baseDist2
 * 			// ... 
 * 			content.addConditionallyIndependentDistribution(ciDists,baseDist2);
 * </pre><p>
 * 
 * 
 * @author	Geert-Jan M. Kruijff (gj@dfki.de)
 * @author	Pierre Lison (pierre.lison@dfki.de)
 * @version 100521
 * @started 100523
 */

public class Content {

	private ProbDistribution _content = null;
	
	/**
	 * Initializes the internal datastructures
	 */
	
	public void init() 
	{
		_content = new ProbDistribution();
	} // end init
	
	/**
	 * Checks for initialization of the content data structure
	 * @param  msg The message for the exception 
	 * @throws BeliefNotInitializedException If the content has not yet been initialized
	 */
	
	private void checkInitialized (String msg)
	throws BeliefNotInitializedException 
	{
		if (_content == null) 
		{
			throw new BeliefNotInitializedException(msg);
		}
	} // end checkInitialized
	
	/**
	 * Returns whether the content is empty or null
	 * @return boolean True if the content is empty or null
	 */
	
	public boolean isEmpty () 
	{ 
		return (_content == null);
	} // end isEmpty
	
	/** 
	 * Sets the content distribution as a dictionary of conditionally independent distributions
	 */
	public void asDictionary () 
	{
		CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
		_content = ciDists; 
	} // end asDictionary
	
	
	/**
	 * Sets the content distribution to the provided object
	 * @param dist	A content representation based in a probability distribution 
	 * @throws BeliefNotInitializedException If the content structure has not been initialized
	 * @throws BeliefMissingValueException If the provided distribution is null
	 */
	
	public void setDistribution (ProbDistribution dist)
	throws BeliefNotInitializedException, BeliefMissingValueException  
	{
		checkInitialized("Cannot set [distribution] for non-initialized content");
		if (dist == null) 
		{
			throw new BeliefMissingValueException("Cannot set [distribution]: Provided parameter is null");
		}
		// set the content to the provided distribution
		_content = dist;
	} // end setDistribution
	

	
	
	/**
	 * Returns whether the content already has a distribution 
	 * @return boolean True if the content already has a distribution
	 * 
	 */
	public boolean hasDistribution () 
	{ 
		// check whether we have an empty content, 
		if (_content == null) 
		{ 
			return false; 
		}
		// else we return true
		return true;
	} // end hasDistribution 
	
	/**
	 * Returns the content distribution 
	 * @return ProbDistribution The content distribution 
	 * @throws BeliefNotInitializedException If the content structure has not been initialized
	 */
	
	public ProbDistribution getDistribution() 
	throws BeliefNotInitializedException, BeliefInvalidOperationException  
	{
		checkInitialized("Cannot get [distribution] from non-initialized content");
		return _content;
	} // end getDistribution 
	

	/**
	 * Creates a new basic distribution given a key, and a list with distribution values. The list can be
	 * a list of pairs of (formula content, probability). If the total sum of the probabilities in the 
	 * list is less than 1.0, another pair (-100:unknown, 1-total) is added. (The -100 is used as the 
	 * identifier of the formula in the pair.)
	 * @return BasicPropDistribution A basic probability distribution
	 * @throws BeliefMissingValueException If either the key or the list of formulas is empty or null
	 * @throws BeliefInvalidOperationException If the kind of distribution values list is not known by the method. 
	 */
	
	public static BasicProbDistribution createBasicDistribution (String key, DistributionValues distValues)
	throws BeliefMissingValueException, BeliefInvalidOperationException 
	{
		// abort if the key is null or empty
		if (key == null || key.equals("")) 
		{
			throw new BeliefMissingValueException("Cannot create basic distribution: Provided key is null/empty");
		}
		// abort if the distribution values list is null or empty
		if (distValues == null)
		{
			throw new BeliefMissingValueException("Cannot create basic distribution: Provided distribution values is null");
		}
		// check what kind of distribution we have
		if (distValues instanceof FormulaValues) 
		{
			return Content.createBasicDistFromProbabilityFormulas (key, (FormulaValues)distValues);
		} 
		else
		{
			throw new BeliefInvalidOperationException("Cannot create basic distribution: "
					+"Method cannot handle type of distribution values ["+distValues.getClass().getName()+"]");
		}
	} // end createBasicDistribution
	
	/**
	 * Specialized method to create a basic distribution from a list of (formula, probability) pairs
	 * @param key			The key for the distribution
	 * @param distValues	A list of formula,probability pairs to create the distribution from 
	 * @return BasicProbDistribution The distribution to be returned
	 * @throws BeliefInvalidOperationException If the sum of the probabilities in the distValues list sum up to more than 1.0f
	 */
	
	private static BasicProbDistribution createBasicDistFromProbabilityFormulas (String key, FormulaValues distValues)
	throws BeliefInvalidOperationException
	{
		// initialize the return result
		BasicProbDistribution _result = null;
		// initialize the list of values
		List<FormulaProbPair> _values = distValues.values; 
		// initialize the total sum of the probabilities to 0
		float total = 0.0f;
		// cycle over the list of formula value pairs in the provided distribution 
		for (FormulaProbPair value : _values) {
			total += value.prob;
		}
		// check that the total values is not more than 1.0
		if (total > 1.01) {
			throw new BeliefInvalidOperationException("Error in creating a feature distribution: "+
					"probabilities of formula distribution sum up to more than 1 [" + total+"]");
		}
		else if (total < 0.99) 
		{
				// create a formula with proposition "unknown" and add it with a value 1-total
				Formula _formula = new Formula();
				_formula.init();
				_formula.setId(-100);
				_formula.asProposition("unknown");
				_formula.setProbability(1-total);
				FormulaProbPair uval = _formula.getAsPair();
				_values.add(uval);
		}
		// 
		FormulaValues formValues = new FormulaValues(_values);
		return new BasicProbDistribution (key, formValues);
	} // end createBasicDistFromProbabilityFormulas
	
	/**
	 * Retrieves a formula from the stored distribution by identifier
	 * @param id 	The identifier of the formula to be retrieved
	 * @return Formula The formula as retrieved from the distribution 
	 * @throws BeliefInvalidOperationException If the content has no distribution, or does not have Formula distribution
	 * @throws BeliefMissingValueException If the distribution does not contain a formula with given identifier
	 */
	
	public Formula getById (int id)
	throws BeliefInvalidOperationException, BeliefMissingValueException 
	{
		if (this.isEmpty()) 
		{
			throw new BeliefInvalidOperationException("Cannot retrieve formula from non-initialized content");
		}
		if (_content instanceof BasicProbDistribution && 
				((BasicProbDistribution)_content).values instanceof FormulaValues) 
		{ 
			FormulaProbPair _retrieved = null;
			FormulaValues _fvalues = (FormulaValues) ((BasicProbDistribution)_content).values;
			if (_fvalues.values.size() == 0)
			{
				throw new BeliefMissingValueException("Cannot retrieve a formula from an empty distribution");
			}
			for (FormulaProbPair fpPair : _fvalues.values) 
			{
				if (fpPair.val.id == id) 
				{
					_retrieved = fpPair;
				} // end if .. check for id
			} // end for
			if (_retrieved != null)
			{
				return new Formula(_retrieved.val, _retrieved.prob);
			} 
			else 
			{
				throw new BeliefMissingValueException("Unknown formula with id ["+id+"] in distribution");
			}
		} 
		else 
		{
			throw new BeliefInvalidOperationException("Cannot retrieve formula from a distribution not based on formulas");
		}
	} // end getById
	
	/**
	 * Create a new set of conditionally independent distributions
	 * @return a new CondIndependentDistribs object
	 */
	public static CondIndependentDistribs createConditionallyIndependentDistributions () {
		return new CondIndependentDistribs(new HashMap<String, ProbDistribution>());
	} // end 
	
	
	/**
	 * Insert a new distribution to a set of conditionally independent distributions
	 * NB: the key/identifier to the distribution must be set in the provided newDistrib. This key identifies the 
	 * feature over whose values the distribution is defined. The key is used to store the distribution in a hash map. 
	 * 
	 * @param distribs 
	 * 			the existing set of conditionally independent distributions
	 * @param newDistrib 
	 * 			the next distribution to add
	 * @throws BeliefMissingValueException 
	 * 			exception thrown if distribs or newDistrib is a null pointer
	 * @post distribs now contains newDistrib
	 */
	public static void addConditionallyIndependentDistribution(CondIndependentDistribs distribs, 
			BasicProbDistribution newDistrib) 
	throws BeliefMissingValueException 
	{
		if (distribs == null) {
			throw new BeliefMissingValueException("Error in adding a new conditionally independent distribution: "+
					"Conditional distributions map is null");
		}
		else if (newDistrib == null) { 
			throw new BeliefMissingValueException("Error in adding a new conditionally independent distribution: "+
					"Provided probability distribution is null");	
		} 
		else if (distribs.distribs == null) {
			throw new BeliefMissingValueException("Error in adding a new conditionally independent distribution: "+
					"Conditional distributions map has not been properly initialized");
		}
		
		else if (newDistrib.key == null || newDistrib.key.equals("")) {
			throw new BeliefMissingValueException("Error in adding a new conditionally independent distribution: "+
					"Null or empty key specified in the provided probability distribution");
		}		
		distribs.distribs.put(newDistrib.key, newDistrib);
	} // end addConditionallyIndependentDistribution
	
	
	/**
	 * Get a probability distribution, specified by key, from the set of conditionally independent distributions
	 * 
	 * @param 	key 				The key for the distribution to be retrieved
	 * @param 	cDists				The set of conditionally independent distributions
	 * @returns	ProbDistribution 	The distribution to be retrieved
	 * @throws	BeliefMissingValueException 	Thrown if the set does not contain a distribution with the specified key
	 */
	
	public static ProbDistribution getConditionallyIndependentDistribution (CondIndependentDistribs cDists, String key)
	throws BeliefMissingValueException
	{ 
		if (cDists.distribs.containsKey(key)) {
			ProbDistribution dist = (ProbDistribution) cDists.distribs.get(key);
			return dist;
		} else {
			throw new BeliefMissingValueException("Error in retrieving conditionally independent distribution: "
					+ "Key ["+key+"] unknown in set of distributions");
		} // end if..else
	} // end method
	
	/**
	 * Remove a probability distribution, specified by key, from the set of conditionally independent distributions
	 * 
	 * @param 	key 				The key for the distribution to be retrieved
	 * @param 	cDists				The set of conditionally independent distributions
	 * @returns	ProbDistribution 	The distribution to be retrieved
	 * @throws	BeliefMissingValueException 	Thrown if the set does not contain a distribution with the specified key
     * @post	The set of conditionally independent distributions cDists no longer contains the distribution for key
	 */	
	
	public static void removeConditionallyIndependentDistribution (CondIndependentDistribs cDists, String key) 
	throws BeliefMissingValueException
	{
		if (cDists.distribs.containsKey(key)) {
			cDists.distribs.remove(key);
		} else {
			throw new BeliefMissingValueException("Error in retrieving conditionally independent distribution: "
					+ "Key ["+key+"] unknown in set of distributions");
		} // end if..else	
	} // end method
	
	/**
	 * Adds the provided distribution to a stored dictionary of conditionally independent distributions. 
	 * It is not checked whether there is already a distribution with this key in the dictionary. This allows
	 * for an "overloading" of this operation: clean add, or update (overwrite) an existing distribution. 
	 * @param dist	The distribution to be added
	 * @throws BeliefInvalidOperationException If the content does not have a c.i. distributions dictionary
	 * @throws BeliefMissingValueException If the dist is null, or its key is null or empty
	 */
	public void addDistributionToDictionary (BasicProbDistribution dist)
	throws BeliefInvalidOperationException, BeliefMissingValueException 
	{
		// abort if we do not have a dictionary
		if (this.isEmpty() || !(_content instanceof CondIndependentDistribs))
		{
			throw new BeliefInvalidOperationException ("Cannot add a distribution to a dictionary: No dictionary present in content");
		}
		// abort if we do not have a distribution 
		if (dist == null)
		{
			throw new BeliefMissingValueException("Canot add a distribution to a dictionary: Provided distribution is null");
		}
		// abort if the key of the distribution is null or empty
		if (dist.key == null || dist.key.equals(""))
		{
			throw new BeliefMissingValueException("Cannot add a distribution to a dictionary: Provided distribution has null/empty key");
		}
		Content.addConditionallyIndependentDistribution((CondIndependentDistribs)_content, dist);
	} // end addDistributionToDictionary
	
	/**
	 * Gets the distribution with the given key directly from the dictionary stored in the content
	 * @param key 	The key of the distribution to be retrieved
	 * @return BaseProbDistribution The distribution as retrieved from the dictionary
	 * @throws BeliefMissingValueException If the dictionary does not contain a distribution with the given key, 
	 * 										or if the key is empty or null
	 * @throws BeliefInvalidOperationException If the content has no dictionary
	 */
	public BasicProbDistribution getDistributionFromDictionary (String key)
	throws BeliefMissingValueException, BeliefInvalidOperationException 
	{
		// abort if we do not have a dictionary
		if (this.isEmpty() || !(_content instanceof CondIndependentDistribs))
		{
			throw new BeliefInvalidOperationException ("Cannot get a distribution from dictionary: No dictionary present in content");
		}	
		if (key == null || key.equals(""))
		{
			throw new BeliefMissingValueException ("Cannot get a distribution from dictionary: Provided key is empty/null");
		}
		// we have a key and a dictionary
		return (BasicProbDistribution) Content.getConditionallyIndependentDistribution((CondIndependentDistribs)_content, key);
	} // end getDistributionFromDictionary
	
	/**
	 * Remove the distribution with the given key directly from the dictionary stored in the content
	 * @param key 	The key of the distribution to be retrieved
	 * @throws BeliefMissingValueException If the dictionary does not contain a distribution with the given key, 
	 * 										or if the key is empty or null
	 * @throws BeliefInvalidOperationException If the content has no dictionary
	 */
	public void removeDistributionFromDictionary (String key)
	throws BeliefMissingValueException, BeliefInvalidOperationException 
	{
		// abort if we do not have a dictionary
		if (this.isEmpty() || !(_content instanceof CondIndependentDistribs))
		{
			throw new BeliefInvalidOperationException ("Cannot remove a distribution from dictionary: No dictionary present in content");
		}	
		if (key == null || key.equals(""))
		{
			throw new BeliefMissingValueException ("Cannot remove a distribution from dictionary: Provided key is empty/null");
		}
		// we have a key and a dictionary
		Content.removeConditionallyIndependentDistribution((CondIndependentDistribs)_content, key);
	} // end removeDistributionFromDictionary
	
	
	
	
} // end class
