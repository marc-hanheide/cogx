
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@dfki.de)                                                                
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


package de.dfki.lt.tr.dialmanagement.data;

import java.util.Collection;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

/**
 * Wrapper around the formula of an epistemic object, providing additional 
 * functionalities for dialogue management
 *  
 * @author Pierre Lison (plison@dfki.de)
 * @version 7/10/2010
 *
 */
public class FormulaWrapper {
	
	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the content, as a formula
	protected dFormula content;
	
	
	/**
	 * Construct a formula wrapper given the string representation
	 * of the formula
	 * 
	 * @param utterance the string for the formula
	 */
	public FormulaWrapper (String utterance) {		
		try {
			content = FormulaUtils.constructFormula(utterance);
		} catch (DialogueException e1) {
			e1.printStackTrace();
		}
	}
	
	/**
	 * Construct a formula wrapper given the formula
	 * 
	 * @param formula the formula
	 */
	public FormulaWrapper (dFormula formula) {
		content = formula;
	}
	

	/**
	 * Returns true if the formula contains underspecified subformulae,
	 * false otherwise
	 * 
	 * @return true if partly or fully underspecified, false otherwise
	 */
	public boolean isUnderspecified () {
		return (getUnderspecifiedArguments().size() > 0);
	}
	
	/**
	 * Returns true if the formula is unknown, else false
	 * 
	 * @return true if unknown, false otherwise
	 */
	public boolean isUnknown() {
		return (content instanceof UnknownFormula);
	}
	
	
	/**
	 * Returns true if the formula subsumes the formula given as argument,
	 * else returns false
	 * 
	 */
	@Override
	public boolean equals(Object obj) {
		if (obj instanceof FormulaWrapper) {
			return FormulaUtils.subsumes(content, ((FormulaWrapper)obj).content);
		}
		
		return false;
	}

	/**
	 * Returns the content of the wrapper
	 * 
	 * @return the formula
	 */
	public dFormula getContent() {
		return content;
	}

	/**
	 * Returns the underspecified subformulae contained in the wrapper
	 * 
	 * @return a collection of underspecified subformulae
	 */
	public Collection<UnderspecifiedFormula> getUnderspecifiedArguments () {
		return getUnderspecifiedArguments(content);
	}

	
	/**
	 * Returns the underspecified subformulae contained in the formula
	 * 
	 * @param formula the formula to visit
	 * @return a collection of underspecified subformulae
	 */
	private Collection<UnderspecifiedFormula> getUnderspecifiedArguments (dFormula formula) {
		Vector<UnderspecifiedFormula> uforms = new Vector<UnderspecifiedFormula>();
		
		if (formula instanceof ComplexFormula) {
			for (dFormula subform : ((ComplexFormula)formula).forms) {
				uforms.addAll(getUnderspecifiedArguments(subform));
			}
		}
		else if (formula instanceof ModalFormula) {
			uforms.addAll(getUnderspecifiedArguments(((ModalFormula)formula).form));
		}
		else if (formula instanceof UnderspecifiedFormula) {
			uforms.add((UnderspecifiedFormula)formula);
		}
		return uforms;
	}
	
	
	/**
	 * Returns a string version of the formula
	 */
	@Override
	public String toString () {
		return FormulaUtils.getString(content);
	}
	


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[formwrapper] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[formwrapper] " + s);
		}
	}
}
