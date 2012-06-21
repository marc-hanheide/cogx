
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
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
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.StringTokenizer;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.GenericPointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.formulaParser.FormulaParser;
import de.dfki.lt.tr.dialmanagement.utils.formulaParser.ParseException;
import de.dfki.lt.tr.dialmanagement.utils.formulaParser.TokenMgrError;

/**
 * Utility functions for manipulating logical formulae
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class FormulaUtils {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the formula parser
	private static FormulaParser parser;

	
	// ==============================================================
	// FORMULA CONVERSION METHODS
	// ==============================================================


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
				return "%"+((UnderspecifiedFormula)formula).arglabel;
			
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
		else if (formula instanceof IntegerFormula) {
			return "" + ((IntegerFormula)formula).val;
		}
		else if (formula instanceof FloatFormula) {
			return "" + ((FloatFormula)formula).val + "f";
		}
		else if (formula instanceof BooleanFormula) {
			return "" + ((BooleanFormula)formula).val;
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
				return new ElementaryFormula(0, s);
			}
			catch (TokenMgrError e) 
			{
				return new ElementaryFormula(0, s);
			}
		}
		return new UnknownFormula(0);
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



	// ==============================================================
	// FORMULA COPY METHODS
	// ==============================================================



	/**
	 * Create a new formula with identical content
	 * 
	 * @param form the formula to copy
	 * @return the new copy
	 * @throws DialogueException
	 */
	public static dFormula copy (dFormula form) throws DialogueException {

		if (form instanceof ElementaryFormula) {
			return new ElementaryFormula(0, ((ElementaryFormula)form).prop);
		}
		else if (form instanceof ComplexFormula) {
			LinkedList<dFormula> subFormulae = new LinkedList<dFormula>();
			for (dFormula initSubFormula : ((ComplexFormula)form).forms) {
				subFormulae.add(copy(initSubFormula));
			}
			return new ComplexFormula(0, subFormulae, ((ComplexFormula)form).op);
		}
		else if (form instanceof ModalFormula) {
			return new ModalFormula(0, ((ModalFormula)form).op, copy(((ModalFormula)form).form));
		}
		else if (form instanceof IntegerFormula) {
			return new IntegerFormula(0, ((IntegerFormula)form).val);
		}
		else if (form instanceof FloatFormula) {
			return new FloatFormula(0, ((FloatFormula)form).val);
		}
		else if (form instanceof BooleanFormula) {
			return new BooleanFormula(0, ((BooleanFormula)form).val);
		}
		else if (form instanceof GenericPointerFormula) {
			return new GenericPointerFormula(0, ((GenericPointerFormula)form).pointer);
		}
		else if (form instanceof UnderspecifiedFormula) {
			return new UnderspecifiedFormula(0, ((UnderspecifiedFormula)form).arglabel);
		}
		else if (form instanceof UnknownFormula) {
			return new UnknownFormula(0);
		}

		return constructFormula(getString(form));
	}



	// ==============================================================
	// METHODS FOR MANIPULATION OF MODAL CONTENT
	// ==============================================================

	
    /**
     * In a formula, search for a subformula with a modal operator of variable "modOp",
     * and sets its pointed formula to be an elementary formula with the given value
     * 
     * @param form the formula to change
     * @param modOp the modal operator to search
     * @param val the value to enter
     */
    public static void setModalOperatorValue(dFormula form, String modOp, String val) {
	
	if (form instanceof ComplexFormula) {
	    for (dFormula subform : ((ComplexFormula)form).forms) {
		setModalOperatorValue(subform, modOp, val);
	    }
	} 
	else if (form instanceof ModalFormula && ((ModalFormula)form).op.equals(modOp)) {
	    ((ModalFormula)form).form = new ElementaryFormula(0,val);
	}
	else if (form instanceof ModalFormula) {
	    setModalOperatorValue(((ModalFormula)form).form, modOp, val);
	}
    }

    

	
	/**
	 * Returns the formula accessible from a particular modal operator applied
	 * to a given formula
	 * 
	 * @param form the full formula
	 * @param modOp the modal operator
	 * @return the formula if it is accessible, null otherwise
	 */
	public static dFormula getModalOperatorValue(dFormula form, String modOp) {
		
		if (form instanceof ComplexFormula) {
			for (dFormula subform : ((ComplexFormula)form).forms) {
				dFormula val = getModalOperatorValue(subform, modOp);
				if (val != null) {
					return val;
				}
			}
		} 
		else if (form instanceof ModalFormula && ((ModalFormula)form).op.equals(modOp)) {
			return ((ModalFormula)form).form;
		}
		else if (form instanceof ModalFormula) {
			return getModalOperatorValue(((ModalFormula)form).form, modOp);
		}
		
		return null;
	}
	

	// ==============================================================
	// FORMULA COMPARISON METHODS
	// ==============================================================


	
	/**
	 * Returns true if the content of form1 subsumes the content form2, 
	 * and false otherwise
	 *   
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if form1 subsumes form2, false otherwise
	 */
	public static boolean subsumes (dFormula form1, dFormula form2)  {
 
		
		form1 = flattenFormula(form1);
		form2 = flattenFormula(form2);
		
		debug("form1: " + getString(form1) + ", type: " + form1.getClass().getSimpleName());
		debug("form2: " + getString(form2) + ", type: " + form2.getClass().getSimpleName());
		
		
		// elementary formulae
		if (form1 instanceof ElementaryFormula && form2 instanceof ElementaryFormula) {
			return compare (((ElementaryFormula)form1).prop, ((ElementaryFormula)form2).prop);
		}

		// complex formulae
		else if (form1 instanceof ComplexFormula && form2 instanceof ComplexFormula) {
			return compare ((ComplexFormula)form1, (ComplexFormula)form2);
		}

		// modal formulae
		else if (form1 instanceof ModalFormula && form2 instanceof ModalFormula) {
			return compare ((ModalFormula)form1, (ModalFormula)form2);
		}
		
		else if (form1 instanceof ModalFormula && form2 instanceof ComplexFormula) {
			List<dFormula> formList = new LinkedList<dFormula>();
			formList.add(form1);
			return compare(new ComplexFormula(0, formList, BinaryOp.conj), (ComplexFormula)form2);
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

		// boolean formulae
		else if (form1 instanceof BooleanFormula && form2 instanceof BooleanFormula) {
			return ((BooleanFormula)form1).val == ((BooleanFormula)form2).val;
		}

		// underspecified formulae (notice the || operator!)
		else if (form1 instanceof UnderspecifiedFormula) {
			return true;
		}

		// unknown formulae
		else if (form1 instanceof UnknownFormula) {
			return true;
		}

		// else, look at the raw string
		return FormulaUtils.getString(form1).equals(FormulaUtils.getString(form2));
	}



	/**
	 * Compares the two strings and returns true if the first one "subsumes"
	 * the second one -- in this case, it means that the two strings are
	 * essentially equal modulo some underspecified arguments
	 * 
	 * @param first the first string
	 * @param second the second string
	 * @return true if the first string subsumes the second, false otherwise
	 */
	private static boolean compare (String first, String second) {
		if (second.toLowerCase().contains(first.toLowerCase())) {
			return true;
		}
		else if (first.contains("%")) {
			
			String[] initSplit = first.split("%");
					
			String varName = "";
			for (int i = 1 ; i < initSplit.length ; i++) {
				StringTokenizer t = new StringTokenizer(initSplit[i]);
				varName = t.nextToken().trim();
			}
			
			String[] fullSplit = first.split("%"+varName);
			
			for (String substr : fullSplit) {
				if (!second.contains(substr)) {
					return false;
				}
			}
			return true;
		}
		else if (first.contains("*")) {
			
			String[] fullSplit = first.split("\\*");
			
			for (String substr : fullSplit) {
				if (!second.contains(substr)) {
					return false;
				}
			}
			return true;
		}
		return false;
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

		if ((form1.forms.size() > form2.forms.size()) || !(form1.op.equals(form2.op))) {
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




	// ==============================================================
	// UTILITY METHODS
	// ==============================================================

	

	/**
	 * Flatten the formula (i.e. transform a complex formula of size one into the
	 * included formula)
	 * 
	 * @param form the formula to flatten
	 * @return the flattened formula
	 */
	public static dFormula flattenFormula (dFormula form) {
		if (form instanceof ComplexFormula && ((ComplexFormula)form).forms.size() == 1) {
			return ((ComplexFormula)form).forms.get(0);
		}
		else {
			return form;
		}
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
