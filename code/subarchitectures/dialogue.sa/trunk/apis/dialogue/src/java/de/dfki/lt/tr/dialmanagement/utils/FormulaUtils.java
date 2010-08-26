package de.dfki.lt.tr.dialmanagement.utils;

import java.io.ByteArrayInputStream;
import java.util.LinkedList;
import java.util.List;
import java.util.StringTokenizer;

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

/**
 * Utility functions for manipulating propositional modal formulae
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public class FormulaUtils {

	
	public boolean LOGGING = true;
	
	
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
			return "<" + ((ModalFormula)formula).op + ">" + getString(((ModalFormula)formula).form);
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
				String str = "(";
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
				str += ")";
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
	 * @throws DialogueException
	 */
	public static dFormula constructFormula (String s) throws DialogueException {

		if (s.trim().equals("*")) {
			return new UnderspecifiedFormula(0);
		}
		else if (s.trim().equals("?")) {
			return new UnknownFormula(0);
		} 
		if (s.length() > 0) {
		try {
			String formattedString = s.trim().replace("\"", " \" ").replace("<", " < ")
				.replace(">", " > ").replace("!(", "! (").replace("(", " ( ").replace(")", " ) ");
			log("STRING TO PROCESS: " + formattedString);
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
	   * @param s
	   * @return
	   * @throws ParseException
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
	 * Returns true if the content of form1 is equal to the content form2, 
	 * and false otherwise
	 *  
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if contents are equal, false otherwise
	 */
	public static boolean isEqualTo (dFormula form1, dFormula form2)  {
		
		if (form1 instanceof ElementaryFormula && form2 instanceof ElementaryFormula) {
			if (((ElementaryFormula)form1).prop.toLowerCase().equals(((ElementaryFormula)form2).prop.toLowerCase())) {
				return true;
			}
		}
		else if (form1 instanceof ComplexFormula && form2 instanceof ComplexFormula) {
			return compare ((ComplexFormula)form1, (ComplexFormula)form2);
		}
		else if (form1 instanceof ModalFormula && form2 instanceof ModalFormula) {
			return compare ((ModalFormula)form1, (ModalFormula)form2);
		}
		
		else if (form1 instanceof IntegerFormula && form2 instanceof IntegerFormula) {
			return ((IntegerFormula)form1).val == ((IntegerFormula)form2).val;
		}
		
		else if (form1 instanceof FloatFormula && form2 instanceof FloatFormula) {
			return ((FloatFormula)form1).val == ((FloatFormula)form2).val;
		}
		
		else if (form1 instanceof GenericPointerFormula && form2 instanceof GenericPointerFormula) {
			return ((GenericPointerFormula)form1).pointer.equals(((GenericPointerFormula)form2).pointer);
		}
		
		else if (form1 instanceof BooleanFormula && form2 instanceof BooleanFormula) {
			return ((BooleanFormula)form1).val == ((BooleanFormula)form2).val;
		}
		
		else if (form1 instanceof UnderspecifiedFormula || form2 instanceof UnderspecifiedFormula) {
			return true;
		}
		else if (form1 instanceof UnknownFormula && form2 instanceof UnknownFormula) {
			return true;
		}
		
		
		return FormulaUtils.getString(form1).equals(FormulaUtils.getString(form2));
	}
	
	
	/**
	 * returns true if the content of form1 is equal to the content of form2,
	 * false otherwise
	 * 
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if contents are equal, false otherwise
	 */
	private static boolean compare (ModalFormula form1, ModalFormula form2) {
		
		if (!form1.op.equals(form2.op)) {
			return false;
		}
		
		return (isEqualTo(form1.form, form2.form));
	}
	
	/**
	 * returns true if the content of form1 is equal to the content of form2,
	 * false otherwise
	 * 
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if contents are equal, false otherwise
	 */
	private static boolean compare (ComplexFormula form1, ComplexFormula form2) {
		
		if ((form1.forms.size() != form2.forms.size()) || !(form1.op.equals(form2.op))) {
			return false;
		}
		
		for (dFormula subform1 : form1.forms) {		
			boolean foundMatch = false;
			for (dFormula subform2 : form2.forms) {
				if (isEqualTo(subform1, subform2)) {
					foundMatch = true;
				}
			}
			if (!foundMatch) {
				return false;
			}
		}
		return true;
	}
	
	
	private static void log (String s) {
		System.out.println("[FormulaUtils] " + s);
	}
	 
}
