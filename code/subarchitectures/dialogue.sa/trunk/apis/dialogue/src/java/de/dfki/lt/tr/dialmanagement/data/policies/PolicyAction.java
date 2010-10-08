
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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


package de.dfki.lt.tr.dialmanagement.data.policies;

import java.util.HashMap;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.FormulaWrapper;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;


/**
 * The action 
 * @author plison
 *
 */
public class PolicyAction extends FormulaWrapper {

	private boolean isVoid = false;
	
	private String id;
	
	public PolicyAction(String id) {
		super ("");
		this.id = id;
		isVoid = true;
	}
	
	public PolicyAction(String id, String str) {
		super(str);
		this.id = id;
	}
	
	public PolicyAction(String id, dFormula formula) {
		super(formula);
		this.id = id;
	}
	

	public boolean isVoid() {
		return isVoid;
	}
	
	public String getId() {
		return id;
	}
	
	public void fillActionArguments (HashMap<Integer,dFormula> arguments) {
		content = fillActionArguments (arguments, content);
	}

	private dFormula fillActionArguments (HashMap<Integer,dFormula> arguments, dFormula formula) {
		
		if (formula instanceof ComplexFormula) {
			for (dFormula subform : ((ComplexFormula)formula).forms) {
				subform = fillActionArguments(arguments,subform);
			}
		}
		else if (formula instanceof ModalFormula) {
			((ModalFormula)formula).form = fillActionArguments(arguments,((ModalFormula)formula).form);
		}
		
		else if (formula instanceof UnderspecifiedFormula) {
			if (arguments.containsKey(((UnderspecifiedFormula)formula).id)) {
				formula = arguments.get(((UnderspecifiedFormula)formula).id);
			}
		}
		return formula;
	}
	
	@Override
	public String toString() {
		if (isVoid) {
			return "VoidAction";
		}
		else {
			return "CI[" + FormulaUtils.getString(content) + "]";
		}
	}
	
	public static PolicyAction createVoidAction() {
		return new PolicyAction("void");
	}
}
