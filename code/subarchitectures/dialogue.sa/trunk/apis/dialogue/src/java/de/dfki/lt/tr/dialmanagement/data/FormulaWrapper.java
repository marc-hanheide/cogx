
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


package de.dfki.lt.tr.dialmanagement.data;

import java.util.Collection;
import java.util.HashMap;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

public class FormulaWrapper {
	
	protected dFormula content;
	
	public FormulaWrapper (String utterance) {		
		try {
			content = FormulaUtils.constructFormula(utterance);
		} catch (DialogueException e1) {
			e1.printStackTrace();
		}
	}
	

	public FormulaWrapper (dFormula formula) {
		content = formula;
	}
	

	
	public boolean isUnderspecified () {
		return (getUnderspecifiedArguments().size() > 0);
	}
	
	public boolean isUnknown() {
		return (content instanceof UnknownFormula);
	}
	
	
	public boolean equals(Object obj) {
		if (obj instanceof FormulaWrapper) {
			return FormulaUtils.subsumes(content, ((FormulaWrapper)obj).content);
		}
		
		return false;
	}

	public dFormula getContent() {
		return content;
	}

	public Collection<UnderspecifiedFormula> getUnderspecifiedArguments () {
		return getUnderspecifiedArguments(content);
	}

	
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
	
	
	@Override
	public String toString () {
		return FormulaUtils.getString(content);
	}
	
}
