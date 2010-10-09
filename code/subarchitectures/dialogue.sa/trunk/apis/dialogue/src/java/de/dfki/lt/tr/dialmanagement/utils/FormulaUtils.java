
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

package de.dfki.lt.tr.dialmanagement.utils;

import java.io.ByteArrayInputStream;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.GenericPointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.formulaParser.FormulaParser;
import de.dfki.lt.tr.dialmanagement.utils.formulaParser.ParseException;

/**
 * Utility functions for manipulating propositional modal formulae
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 09/10/2010
 *
 */
public class FormulaUtils {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the formula parser
	private static FormulaParser parser;

	
	/**
	 * Returns a text representation of the formula
	 * 
	 * @param formula the formula
	 * @return the resulting string
	 */
	public static String getString(dFormula formula) {
		if (formula instanceof ElementaryFormula) {
			return ((ElementaryFormula)formula).prop;
		}
		else if (formula instanceof UnderspecifiedFormula) {
			return "*";
		}
		else if (formula instanceof UnknownFormula) {
			return "?";
		}
		else if (formula instanceof ModalFormula) {
			return "<" + ((ModalFormula)formula).op + ">(" + getString(((ModalFormula)formula).form) + ")";
		}
	
		else if (formula instanceof ComplexFormula) {
			ComplexFormula cform = (ComplexFormula) formula;
			if (cform.forms.size() == 0) {
				return "";
			}
			else if (cform.forms.size() == 1) {
				return getString (cform.forms.get(0));
			}
			else {
				String str = "";
		//		String str = "(";
				for (dFormula subform : cform.forms) {
					str += getString(subform) ;
					if (!subform.equals(cform.forms.get(cform.forms.size() - 1))) {
						if (cform.op.equals(BinaryOp.conj))
						str += " ^ ";
						else if (cform.op.equals(BinaryOp.disj)) {
							str += " v ";
						}
					}
				}
		//		str += ")";
				return str;
			}
		}
		else if (formula != null) {
			return formula.toString();
		}
		return "";
	}


	/**
	 * Construct a logical formula from a string representation
	 * 
	 * @param s the string
	 * @return the formula in propositional modal logic
	 * @throws DialogueException if the formula is not well-formed
	 */
	public static dFormula constructFormula (String s) throws DialogueException {

		if (s.length() > 0) {
		try {
			String formattedString = s.trim().replace("\"", " \" ").replace("<", " < ")
				.replace(">", " > ").replace("!(", "! (").replace("(", " ( ").replace(")", " ) ");
			return getFormulaFromString(formattedString);
		} 
		catch (ParseException e) 
		{
			e.printStackTrace();
			throw new DialogueException("ERROR: couldn't parse formula: " + s);
		}
		}
		return new ElementaryFormula(0,"");
	}


	
	  /**
	   * Get the formula from a string representation (assuming the preformatting is 
	   * already done)
	   * 
	   * @param s the preformatted string
	   * @return the resulting formula
	   * @throws ParseException if the formula is not well-formed
	   */
	  private static dFormula getFormulaFromString (String s) throws ParseException {
		StringBuffer StringBuffer1 = new StringBuffer(s);
	    ByteArrayInputStream Bis1 = new ByteArrayInputStream(StringBuffer1.toString().getBytes());
	    if (parser == null) {
	    	parser = new FormulaParser(Bis1);
	    }
	    else {
	    	parser.ReInit(Bis1);
	    }
		return parser.Input();

	  }
		
	 
	/**
	 * Returns true if the content of form1 subsumes the content form2, 
	 * and false otherwise
	 *  
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if form1 subsumes form2, false otherwise
	 */
	public static boolean subsumes (dFormula form1, dFormula form2)  {
		
		// elementary formulae
		if (form1 instanceof ElementaryFormula && form2 instanceof ElementaryFormula) {
			if (((ElementaryFormula)form1).prop.toLowerCase().equals(((ElementaryFormula)form2).prop.toLowerCase())) {
				return true;
			}
		}
		
		// complex formulae
		else if (form1 instanceof ComplexFormula && form2 instanceof ComplexFormula) {
			return compare ((ComplexFormula)form1, (ComplexFormula)form2);
		}
		
		// modal formulae
		else if (form1 instanceof ModalFormula && form2 instanceof ModalFormula) {
			return compare ((ModalFormula)form1, (ModalFormula)form2);
		}
		
		// integer formulae
		else if (form1 instanceof IntegerFormula && form2 instanceof IntegerFormula) {
			return ((IntegerFormula)form1).val == ((IntegerFormula)form2).val;
		}
		
		// float formulae
		else if (form1 instanceof FloatFormula && form2 instanceof FloatFormula) {
			return ((FloatFormula)form1).val == ((FloatFormula)form2).val;
		}
		
		// generic pointer formulae
		else if (form1 instanceof GenericPointerFormula && form2 instanceof GenericPointerFormula) {
			return ((GenericPointerFormula)form1).pointer.equals(((GenericPointerFormula)form2).pointer);
		}
		
		// CAST pointer formulae
		else if (form1 instanceof PointerFormula && form2 instanceof PointerFormula) {
			return (((PointerFormula)form1).pointer.subarchitecture.equals(((PointerFormula)form2).pointer.subarchitecture) && 
					((PointerFormula)form1).pointer.id.equals(((PointerFormula)form2).pointer.id) && 
					((PointerFormula)form1).type.equals(((PointerFormula)form2).type));
		}
		
		// boolean formulae
		else if (form1 instanceof BooleanFormula && form2 instanceof BooleanFormula) {
			return ((BooleanFormula)form1).val == ((BooleanFormula)form2).val;
		}
		
		// underspecified formulae (notice the || operator!)
		else if (form1 instanceof UnderspecifiedFormula) {
			return true;
		}
		
		// unknown formulae
		else if (form1 instanceof UnknownFormula && form2 instanceof UnknownFormula) {
			return true;
		}
		
		// else, look at the raw string
		return FormulaUtils.getString(form1).equals(FormulaUtils.getString(form2));
	}
	
	
	
	/**
	 * returns true if the content of form1 is equal to the content of form2,
	 * false otherwise (with the two being modal formulae)
	 * 
	 * @param form1 the first modal formula
	 * @param form2 the second modal formula
	 * @return true if contents are equal, false otherwise
	 */
	private static boolean compare (ModalFormula form1, ModalFormula form2) {
		
		if (!form1.op.equals(form2.op)) {
			return false;
		}
		
		return (subsumes(form1.form, form2.form));
	}
	
	/**
	 * returns true if the content of form1 is equal to the content of form2,
	 * false otherwise (with the two being complex formulae)
	 * 
	 * @param form1 the first complex formula
	 * @param form2 the second complex formula
	 * @return true if contents are equal, false otherwise
	 */
	private static boolean compare (ComplexFormula form1, ComplexFormula form2) {
		
		if ((form1.forms.size() != form2.forms.size()) || !(form1.op.equals(form2.op))) {
			return false;
		}
		
		for (dFormula subform1 : form1.forms) {	
			Vector<dFormula> alreadyMatched = new Vector<dFormula>();
			boolean foundMatch = false;
			for (dFormula subform2 : form2.forms) {
				if (!alreadyMatched.contains(subform2) && subsumes(subform1, subform2)) {
					foundMatch = true;
					alreadyMatched.add(subform2);
				}
			}
			if (!foundMatch) {
				return false;
			}
		}
		return true;
	}
	

	


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[formulautils] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[formulautils] " + s);
		}
	}
	 
}
