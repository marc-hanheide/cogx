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

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.FormulaWrapper;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;

public class PolicyUtils {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	
	public static List<PolicyEdge> sortEdges (Collection<PolicyEdge> unsortedEdges) {
		
		LinkedList<PolicyEdge> sortedEdges = new LinkedList<PolicyEdge>();
		
		for (PolicyEdge edge: unsortedEdges) {
			if (edge.getObservation().isUnderspecified() || edge.getObservation().isUnknown() ) {
				sortedEdges.addLast(edge);
			}
			else {
				sortedEdges.addFirst(edge);
			}
		}
		
		return sortedEdges;
	}
		
	
	public static HashMap<Integer,dFormula> extractFilledArguments (Observation obs, PolicyEdge edge) {
		
		for (FormulaWrapper alternative : obs.getAlternatives()) {		
			if (alternative.equals(edge.getObservation())) {
				return extractFilledArguments (edge.getObservation().getContent(), alternative.getContent()) ;
			}
		}
		return new HashMap<Integer,dFormula>();
	}
	
	
	
	
	
	public static HashMap<Integer,dFormula> extractFilledArguments (dFormula form1, dFormula form2)  {
		
		HashMap<Integer,dFormula> filledArguments = new HashMap<Integer,dFormula>();

		if (form1 instanceof ComplexFormula && form2 instanceof ComplexFormula) {
			filledArguments.putAll(extractFilledArgumentsInComplexFormula((ComplexFormula)form1, (ComplexFormula)form2));
		}
		
		if (form1 instanceof ModalFormula && form2 instanceof ModalFormula) {
			if (((ModalFormula)form1).op.equals(((ModalFormula)form2).op)) {
				filledArguments.putAll(extractFilledArguments(((ModalFormula)form1).form, ((ModalFormula)form2).form));
			}
		}
		
		else if (form1 instanceof UnderspecifiedFormula) {
			filledArguments.put(form1.id, form2);
		}
			
		return filledArguments;
	}
	

	
	public static HashMap<Integer,dFormula> extractFilledArgumentsInComplexFormula (ComplexFormula form1, ComplexFormula form2)  {
			
		HashMap<Integer,dFormula> filledArguments = new HashMap<Integer,dFormula>();
		
		if ((form1.forms.size() != form2.forms.size()) || !(form1.op.equals(form2.op))) {
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
				return new HashMap<Integer,dFormula>();
			}
		}
		return filledArguments;
	}
	
}
