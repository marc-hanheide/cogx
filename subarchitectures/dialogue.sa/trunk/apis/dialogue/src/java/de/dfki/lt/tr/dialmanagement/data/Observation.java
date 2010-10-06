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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

import java.util.Collection;
import java.util.HashMap;

public class Observation {

	private HashMap<FormulaWrapper,Float> alternatives ;
	
	private int observationType ;
	
	public static final int INTENTION = 0;
	public static final int EVENT = 1;
	
	public Observation(int observationType) throws DialogueException {
		alternatives = new HashMap<FormulaWrapper, Float>();

		if (observationType == INTENTION || observationType == EVENT) {
			this.observationType = observationType;
		}
		else {
			throw new DialogueException("ERROR: observation type not allowed");
		}
	}
	
	public void addAlternative (FormulaWrapper content, float prob) {
		alternatives.put(content, prob);
	}
	
	
	public void addAlternative (dFormula formula, float prob) {
		alternatives.put(new FormulaWrapper (formula), prob);
	}
	
	
	public void addAlternative (String str, float prob) {
		if (str.contains(" ") && !str.contains("^") && !str.contains("<")) {
			addAlternative(new FormulaWrapper ("\"" +str + "\""), prob);
		}
		else  {
			addAlternative(new FormulaWrapper (str), prob);
		}
	}
	
	
	public Collection<FormulaWrapper> getAlternatives () {
		return alternatives.keySet();
	}
	
	public int getType() {
		return observationType;
	}
	
	
	public float getProbability (FormulaWrapper content) {
		if (alternatives.containsKey(content)) {
			return alternatives.get(content);
		}
		else {
			return 0.0f;
		}
	}
	
	public String toString() {
		String result = "{";
		for (FormulaWrapper key : alternatives.keySet()) {
			result += "(" + key + ", "  + alternatives.get(key) + "), ";
		}
		return result.substring(0, result.length() -2) + "}";
	}
	
}
