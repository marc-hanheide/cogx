//=================================================================
//Copyright (C) 2010 Pierre Lison (plison@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.util;

//=================================================================
//IMPORTS

// Java
import java.util.Arrays;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

/**
 * The <tt>Rule</tt> class provides the basic data structure for rules in a CFG, of the form 
 * LHS : RHS. A RHS is a sequence of String symbols, and can be associated with a weights map. 
 * Weights provide a weighted link from a previous rule (i.e. a LHS, used as key) to the current RHS. 
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 100608
 */

public class Rule {

	public String LHS;
	public Vector<String> RHS;
	public String semantics;
	public Hashtable<String,Float> weights;

	/**
	 * Constructs an empty rule
	 */
	
	public Rule() {
		RHS = new Vector<String>();
		weights = new Hashtable<String,Float>();
	}
	
	/**
	 * Constructs a rule from the given LHS and RHS
	 * @param LHS	The left-hand side of the rule
	 * @param RHS	The right-hand side, as a sequence of string symbols
	 */
	public Rule(String LHS, Vector<String> RHS) {
		this.LHS = LHS;
		this.RHS = RHS;
		weights = new Hashtable<String,Float>();
	}

	/**
	 * Constructs a rule from the given LHS and RHS. The RHS is parsed into a sequence of separate
	 * string symbols, using space as separator. 
	 * 
	 * @param LHS	The left-hand side of the rule
	 * @param RHS	The right-hand side, as a string.
	 */
	
	public Rule(String LHS, String RHS) {
		this.LHS = LHS;
		this.RHS = new Vector<String>(Arrays.asList(RHS.split(" ")));
	}
	
	/**
	 * Constructs a rule from the given RHS. The LHS is assigned the empty string "". 
	 * @param RHS	The right-hand side, as a sequence of string symbols
	 */
	
	public Rule(Vector<String> RHS) {
		this.LHS = "";
		this.RHS = RHS;
		weights = new Hashtable<String,Float>();
	}
	
	/**
	 * Sets the semantics associated with invoking the rule
	 * @param semantics
	 */
	
	public void setSemantics(String semantics) {
		this.semantics = semantics;
	}
	
	/**
	 * Sets the LHS symbol of the rule
	 * @param LHS	The LHS for the rule
	 */
	
	public void setLHS (String LHS) {
		this.LHS = LHS;
	}
	
	/**
	 * Returns the semantics as a String
	 * @return String	The semantics stored with the rule
	 */
	
	public String getSemantics() {
		return semantics;
	}
	
	/**
	 * Adds the given string as symbol to the RHS
	 * @param str
	 * @throws DialogueMissingValueException Thrown if the parameter is null or empty
	 */
	
	public void addToRHS(String str) 
	throws DialogueMissingValueException 
	{
		if (str == null || str.equals(""))
		{
			throw new DialogueMissingValueException("Cannot add to RHS: Provided string null or empty");
		}
		RHS.add(str);
	} // end addToRHS
	
	/**
	 * Adds a given vector of strings to the RHS of the rule
	 * @param strs	The vector of strings to be added
	 * @throws DialogueMissingValueException	Thrown if the vector is null or empty
	 */
	
	public void addToRHS(Vector<String> strs) 
	throws DialogueMissingValueException 
	{
		if (strs == null || strs.size() == 0)
		{
			throw new DialogueMissingValueException("Cannot add to RHS: Provided vector null or empty");
		}
		RHS.addAll(strs);
	} // end addToRHS

	/**
	 * Adds the RHS of the given rule to the local rule
	 * @param 	rule	The rule whose RHS is to be added 
	 * @throws	DialogueMissingValueException Thrown if the provided rule is null
	 */
	
	public void addToRHS(Rule rule) 
	throws DialogueMissingValueException 
	{
		if (rule == null)
		{
			throw new DialogueMissingValueException("Cannot add to RHS: Provided RHS is null");
		}
		addToRHS(rule.getRHS());
	} // end addToRHS
	
	/**
	 * Returns the right-hand side of the rule, as a list (vector) of string symbols
	 * @return Vector<String> The right-hand side of the rule
	 */
	
	public Vector<String> getRHS() {return RHS ; }

	/**
	 * Returns the left-hand side of the rule as a String symbol
	 * @return	String	The left-hand side of the rule
	 */
	
	public String getLHS() { return LHS; }

	/**
	 * Returns a String representation of the rule, of the format "LHS ==&gt; RHS1 ... RHSn"
	 * @return String 	The string representation 
	 */
	
	public String toString() {
		String result = LHS + " ==> ";
		for (Enumeration<String> e = RHS.elements() ; e.hasMoreElements() ; ) {
			result += e.nextElement() + " ";
		}
		return result;
	} // end toString

	/**
	 * Returns the symbol at the given position within the RHS
	 * @param  pos	The position, which is to be 0 &lt;= .. &lt; RHS.size()
	 * @return	String	The symbol at the given position 
	 * @throws DialogueInvalidOperationException	Thrown if the position is out of range
	 */
	
	public String getRHSConstituent(int pos) 
	throws DialogueInvalidOperationException 
	{
		if (pos < RHS.size() && pos >= 0) {
			return RHS.elementAt(pos);
		}
		else 
		{
			throw new DialogueInvalidOperationException("Cannot retrieve RHS constituent: index out of range");
		}
	} // end getRHSConstituent

	/**
	 * Sets a weight associated with the rule, and a given previous rule (LHS)
	 * @param prevRule	The symbol of the previous rule (LHS)
	 * @param weight	The weight to be associated
	 * @throws DialogueMissingValueException Thrown if the prevRule symbol is null or empty
	 */
	
	public void setWeight(String prevRule, float weight) 
	throws DialogueMissingValueException 
	{
		if (prevRule == null || prevRule.equals(""))
		{
			throw new DialogueMissingValueException("Cannot set weight: Provided previous rule symbol null or empty");
		}
		weights.put(prevRule, new Float(weight));
	} // end setWeight

} // end class
