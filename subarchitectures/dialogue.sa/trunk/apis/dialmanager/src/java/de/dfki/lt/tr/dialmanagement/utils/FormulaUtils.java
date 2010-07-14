package de.dfki.lt.tr.dialmanagement.utils;

import java.io.ByteArrayInputStream;
import java.util.LinkedList;
import java.util.List;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.formulaParser.FormulaParser;
import de.dfki.lt.tr.dialmanagement.utils.formulaParser.ParseException;

/**
 * Utility functions for manipulating propositional modal formulae
 * 
 * TODO: extend this to generic formulae (not only elementary ones)
 *
 * @author Pierre Lison (plison@dfki.de)
 * @version 03/07/2010
 *
 */
public class FormulaUtils {

	
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
			System.out.println("STRING TO PROCESS: " + formattedString);
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
		else if (form1 instanceof UnderspecifiedFormula || form2 instanceof UnderspecifiedFormula) {
			return true;
		}
		else if (form1 instanceof UnknownFormula && form2 instanceof UnknownFormula) {
			return true;
		}
		return false;
	}
	
	 
}
