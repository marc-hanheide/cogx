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

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.StringTokenizer;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;


/**
 * Utilities for manipulating objects related to dialogue policy
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 09/10/2010
 *
 */
public class PolicyUtils {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	
	

	// ==============================================================
	// EDGE SORTING METHODS (PARTIAL ORDERING)
	// ==============================================================

	
	
	/**
	 * Sort the edge according to their specificity, the head of the list being the most
	 * specific while the tail is the most underspecified
	 *    
	 * @param unsortedEdges the unsorted collection of edges
	 */
	public static void sortEdges (List<PolicyEdge> edges) {
		
		debug("sortedEdges: " + edges);
		
		// fixed point to determine when to stop sorting
		boolean fixedPointReached = false;
		
		while (!fixedPointReached) {
			fixedPointReached = true;
			for (int i = 0 ; fixedPointReached && i < edges.size(); i++) {
				PolicyEdge firstEdge = edges.get(i);
				if (i < edges.size() -1) {
					PolicyEdge nextEdge = edges.get(i+1);
					
					// we first use subsumption to sort the edges according
					// to their specificity
					boolean subsumption1 = 
						FormulaUtils.subsumes(firstEdge.getConditionsAsSingleFormula(), 
								nextEdge.getConditionsAsSingleFormula());
					boolean subsumption2 = 
						FormulaUtils.subsumes(nextEdge.getConditionsAsSingleFormula(),
								firstEdge.getConditionsAsSingleFormula());
					
					// if one edge is more specific than the other one, it is moved up
					if (subsumption1 && !subsumption2) {
						edges.remove(i);
						edges.add(i+1, firstEdge);
						fixedPointReached = false;
					}
					
					// if the subsumption above was not enough to sort the edges, 
					// we resort to simple heuristics
					else if (!subsumption1 && !subsumption2) {
						
						// counting the number of underspecified arguments
						if (firstEdge.getAllUnderspecifiedArguments().size() > 
						nextEdge.getAllUnderspecifiedArguments().size()) {
							edges.remove(i);
							edges.add(i+1, firstEdge);
							fixedPointReached = false;
						}
						
						// or simply comparing the lengths of the conditions
						else if (FormulaUtils.getString(firstEdge.getConditionsAsSingleFormula()).length() 
								>= FormulaUtils.getString(nextEdge.getConditionsAsSingleFormula()).length()) {
							edges.remove(i);
							edges.add(i+1, firstEdge);
							fixedPointReached = false;
						}
					}
				}
			}
		}
		
	} 
		 	
	
	
	

	// ==============================================================
	// METHODS FOR MANIPULATION OF UNDERSPECIFIED CONTENT
	// ==============================================================


	
	/**
	 * Returns the underspecified subformulae contained in the formula
	 * 
	 * @param formula the formula to visit
	 * @return a collection of underspecified subformulae
	 */
	public static Collection<String> getUnderspecifiedArguments (dFormula formula) {
		Vector<String> uforms = new Vector<String>();
		
		if (formula instanceof ComplexFormula) {
			for (dFormula subform : ((ComplexFormula)formula).forms) {
				uforms.addAll(getUnderspecifiedArguments(subform));
			}
		}
		else if (formula instanceof ModalFormula) {
			uforms.addAll(getUnderspecifiedArguments(((ModalFormula)formula).form));
		}
		else if (formula instanceof UnderspecifiedFormula) {
			uforms.add(""+((UnderspecifiedFormula)formula).arglabel);
		}
		else if (formula instanceof ElementaryFormula) {
			uforms.addAll(getUnderspecifiedFormulae(((ElementaryFormula)formula).prop));
		}
		return uforms;
	}
	
	 
	
	private static Collection<String> getUnderspecifiedFormulae (String text) {
		
		Vector<String> uforms = new Vector<String>();
		
		String[] splits = text.split("%");
		
		for (int i = 1 ; splits.length > 1 && i < splits.length ; i++) {
			StringTokenizer t = new StringTokenizer(splits[i]);
			String varName = t.nextToken().replace("!", "").replace(",", "").replace(".", "");
			uforms.add(varName);
		}
		
		return uforms;
	}
	
	
	
	
	/**
	 * Given two formulae (one of which is fully specified and the other contains underspecified
	 * arguments), extract the value of each argument
	 * @param form1 the underspecified formula
	 * @param form2 the fully specified formula
	 * @return  a mapping from each argument i in form1 to its value in form2
	 * @throws DialogueException 
	 */
	public static HashMap<String,dFormula> extractFilledArguments (dFormula form1, dFormula form2)  {
						
		HashMap<String,dFormula> filledArguments = new HashMap<String,dFormula>();

		form1 = FormulaUtils.flattenFormula(form1);
		form2 = FormulaUtils.flattenFormula(form2);	
		
		if (form1 instanceof ComplexFormula && form2 instanceof ComplexFormula) {
			filledArguments.putAll(extractFilledArgumentsInComplexFormula((ComplexFormula)form1, (ComplexFormula)form2));
		}
		
		else if (form1 instanceof ModalFormula && form2 instanceof ModalFormula) {
			if (((ModalFormula)form1).op.equals(((ModalFormula)form2).op)) {
				filledArguments.putAll(extractFilledArguments(((ModalFormula)form1).form, ((ModalFormula)form2).form));
			}
		}
		
		else if (form1 instanceof ModalFormula && form2 instanceof ComplexFormula) {
			List<dFormula> formList = new LinkedList<dFormula>();
			formList.add(form1);
			filledArguments.putAll(extractFilledArgumentsInComplexFormula(
					new ComplexFormula(0, formList, BinaryOp.conj), (ComplexFormula)form2));
		}
		
		else if (form1 instanceof UnderspecifiedFormula) {
			debug("detected the following argument: <" + ((UnderspecifiedFormula)form1).arglabel + ">" + FormulaUtils.getString(form2));
			filledArguments.put(((UnderspecifiedFormula)form1).arglabel, form2);
		}
		
		return filledArguments;
	}
	



	/**
	 * Given two complex formulae (one of which is fully specified and the other contains underspecified
	 * arguments), extract the value of each argument
	 * @param form1 the underspecified complex formula
	 * @param form2 the fully specified complex formula
	 * @return  a mapping from each argument i in form1 to its value in form2
	 * @throws DialogueException 
	 */	
	public static HashMap<String,dFormula> extractFilledArgumentsInComplexFormula 
		(ComplexFormula form1, ComplexFormula form2) {
			
		HashMap<String,dFormula> filledArguments = new HashMap<String,dFormula>();
		
		if ((form1.forms.size() > form2.forms.size()) || !(form1.op.equals(form2.op))) {
			return filledArguments;
		}
		
		for (dFormula subform1 : form1.forms) {	
			Vector<dFormula> alreadyMatched = new Vector<dFormula>();
			boolean foundMatch = false;
			for (dFormula subform2 : form2.forms) {
				if (!alreadyMatched.contains(subform2) && FormulaUtils.subsumes(subform1, subform2)) {
					foundMatch = true;
					alreadyMatched.add(subform2);
					filledArguments.putAll(extractFilledArguments(subform1,subform2));
				}
			}
			if (!foundMatch) {
				return new HashMap<String,dFormula>();
			}
		}
		return filledArguments;
	}
	 



	// ==============================================================
	// UTILITY METHODS
	// ==============================================================


	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[policyutils] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[policyutils] " + s);
		}
	}
}
