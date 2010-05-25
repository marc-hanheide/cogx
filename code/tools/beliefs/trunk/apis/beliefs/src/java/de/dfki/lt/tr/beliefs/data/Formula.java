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



// Belief API slice
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair; 
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula; 
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula; 

// Belief API util
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;
import de.dfki.lt.tr.beliefs.util.BeliefNotInitializedException;

/**
 * The <tt>Formula</tt> class implements the basic structure for building up content. 
 * 
 * 
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @started 100521
 * @version 100521
 */

public class Formula {

	private dFormula _formula = null;
	private float _probability;
	private String _instantiated; 
	
	/** Empty constructor. No variables are initialized. */ 
	public Formula() { 
		
	} // end constructor
	
	/**
	 * Object is created from underlying slice-based datastructure, and a given probability.
	 */
	public Formula (dFormula formula, float prob) 
	{
		_formula = formula;
		_probability = prob;
		_instantiated = formula.getClass().getName();
	} // end constructor
	
	/** 
	 * Initializes the internal datastructures
	 */
	public void init () 
	{
		_formula = new dFormula();
		_formula.id = -1;
		_probability = -1;
		_instantiated = ""; 
	} // end init

	/**
	 * Checks whether the formula is initialized; if not, throw an exception with the provided message
	 * @param msg	The message for the exception
	 * @throws BeliefNotInitializedException If the formula has not been initialized
	 */
	private void checkInitialized (String msg) 
	throws BeliefNotInitializedException 
	{
		if (_formula == null)
		{
			throw new BeliefNotInitializedException(msg);
		}
	} // end checkInitialized
	
	/**
	 * Checks whether the formula is instantiated; if not, throw an exception with the provided message
	 * @param msg	The message for the exception
	 * @throws BeliefInvalidOperationException If the formula has not been instantiated
	 */
	private void checkInstantiated (String msg) 
	throws BeliefInvalidOperationException 
	{
		if (_instantiated.equals(""))
		{
			throw new BeliefInvalidOperationException(msg);
		}
	} // end checkInstantiated
	
	
	
	
	/**
	 * Returns the identifier of the formula. By default this is set to -1, upon initialization.
	 * @return int The identifier of the formula
	 * @throws BeliefNotInitializedException If the formula has not been initialized
	 */

	public int getId () 
	throws BeliefNotInitializedException 
	{
		checkInitialized("Cannot get [id] for non-initialized formula");
		// for an initialized formula, return the identifier
		return _formula.id;
	} // end getId
	
	/** 
	 * Sets the identifier of the formula. 
	 * @param id 	The identifier (represented as an integer) for the formula
	 * @throws 	BeliefNotInitializedException If the formula has not been initialized
	 */
	
	public void setId (int id) 
	throws BeliefNotInitializedException 
	{
		checkInitialized("Cannot set [id] for non-initialized formula");
		// no need to test for null, as int's cannot be null objects (checked at compile time)
		// set the identifier
		_formula.id = id;
	} // end setId
	
	/**
	 * Instantiates the formula as a proposition, using the provided argument as proposition value
	 * @param 	prop The proposition to be used
	 * @throws  BeliefNotInitializedException If the formula has not been initialized
	 * @throws  BeliefMissingValueException   If the provided argument is empty or null
	 * @throws  BeliefInvalidOperationException 
	 */
	
	public void asProposition (String prop)
	throws BeliefNotInitializedException, BeliefMissingValueException, BeliefInvalidOperationException
	{
		// abort if the formula has not yet been initialized
		checkInitialized("Cannot instantiate non-initialized formula as proposition");
		// abort if the formula has already been instantiated
		if (!(_formula instanceof dFormula)) 
		{
			throw new BeliefInvalidOperationException ("Cannot instantiate formula as proposition: Formula already instantiated");
		}
		// abort if the proposition is null or empty
		if (prop == null || prop.equals("")) 
		{
			throw new BeliefMissingValueException ("Cannot instantiate formula as proposition: Provided argument is null/empty");
		}
		// now instantiate the formula as a proposition
		// store the identifier we might have set before, else it gets lost
		int tempId = _formula.id;
		_formula = new ElementaryFormula ();
		_formula.id = tempId;
		((ElementaryFormula)_formula).prop = prop;
		_instantiated = ((ElementaryFormula)_formula).getClass().getName();
	} // end asProposition

	/**
	 * Returns whether the formula is a proposition (or not)
	 * @return boolean True if the formula is a proposition
	 * @throws BeliefNotInitializedException 	If the formula has not been initialized
	 */
	
	public boolean isProposition () 
	throws BeliefNotInitializedException 
	{
		checkInitialized("Cannot check non-initialized formula for type");
		return (_formula instanceof ElementaryFormula);		
	} // end isProposition
	
	/**
	 * Returns the proposition for a formula, provided the formula is propositional 
	 * @return String The proposition of the formula
	 * @throws BeliefNotInitializedException If the formula is not initialized
	 * @throws BeliefInvalidQueryException If the formula is not of type proposition
	 */
	
	public String getProposition ()
	throws BeliefNotInitializedException, BeliefInvalidQueryException 
	{
		checkInitialized("Cannot return proposition for a non-initialized formula");
		if (!(_formula instanceof ElementaryFormula)) 
		{
			throw new BeliefInvalidQueryException("Cannot query [proposition] for formula: Formula not of type proposition"); 
		}
		else 
		{
			return ((ElementaryFormula)_formula).prop;
		}
	} // end getProposition
	
	/**
	 * Sets the probability for an instantiated formula
	 * @param prob	The probability
	 * @throws BeliefNotInitializedException If the formula is not initialized
	 * @throws BeliefInvalidOperationException If the formula has not been instantiated
	 */
	
	public void setProbability (float prob)
	throws BeliefNotInitializedException, BeliefInvalidOperationException
	{
		checkInitialized("Cannot set the probability of a non-initialized formula");
		checkInstantiated("Cannot set the probability of a non-instantiated formula");
		_probability = prob;
	} // end setProbability
	
	/**
	 * Returns the probability for an instantiated formula
	 * @return float The probability of the formula
	 * @throws BeliefInvalidOperationException If the formula is not instantiated
	 * @throws BeliefNotInitializedException If the formula is not initialized
	 */
	
	public float getProbability ()
	throws BeliefNotInitializedException, BeliefInvalidOperationException
	{
		checkInitialized("Cannot get [probability] of a non-initialized formula");
		checkInstantiated("Cannot get [probability] of a non-instantiated formula");
		return _probability;
	} // end setProbability
	
	
	/**
	 * Returns the formula and probability as a pair
	 * @return FormulaProbPair	The slice-based datastructure of a formula and its associated probability
	 * @throws BeliefNotInitializedException If the formula is not instantiated
	 * @throws BeliefInvalidOperationException If the probability or the formula is not initialized
	 */
	
	public FormulaProbPair getAsPair () 
	throws BeliefNotInitializedException, BeliefInvalidOperationException
	{ 
		checkInitialized("Cannot create formula,probability pair for non-initialized formula");
		checkInstantiated("Cannot create formula,probability pair for non-instantiated formula");
		if (_probability == -1)
		{
			throw new BeliefInvalidOperationException("Cannot create formula,probability pair for non-initialized probability");
		}
		return new FormulaProbPair(_formula,_probability);
	} // end getAsPair
	
	
	
	
	
} // end class
