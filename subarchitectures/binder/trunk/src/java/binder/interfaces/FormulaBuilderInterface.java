

// =================================================================                                                        
// Copyright (C) 2010-2012 Pierre Lison (plison@dfki.de)                                                                
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
 

package binder.interfaces;

import binder.autogen.featurecontent.Feature;
import binder.autogen.logicalcontent.BinaryOp;
import binder.autogen.logicalcontent.ComplexFormula;
import binder.autogen.logicalcontent.ElementaryFormula;
import binder.autogen.logicalcontent.Formula;
import binder.autogen.logicalcontent.ModalFormula;
import binder.autogen.logicalcontent.NegatedFormula;
import binder.autogen.logicalcontent.PointerFormula;


public interface FormulaBuilderInterface {

	
	/**
	 * Create a new, empty complex formula connected with the operator op
	 * 
	 * @param op
	 * 			the binary operator connecting the subformulae
	 * @return a new, empty complex formula
	 */
	public ComplexFormula createNewComplexformula(BinaryOp op);
	
	
	/**
	 * Add a new subformula to an existing complex formula
	 * @param cform
	 * 			the existing complex formula
	 * @param newForm
	 * 			the new formula to add
	 * @post cform now contains newForm
	 */
	public void addSubformulaToComplexFormula (ComplexFormula cform, Formula newForm);


	/**
	 * Create a new elementary formula with proposition prop
	 * @param prop
	 * 			the logical proposition
	 * @return the new elementary formula
	 */
	public ElementaryFormula createNewElementaryFormula (String prop) ;
	
	
	/**
	 * Create a new pointer formula, i.e. a formula whose proposition is a pointer
	 * to another belief
	 * 
	 * @param pointer 
	 * 			a pointer to another belief identifier, which must currently exist 
	 * 			onto the binder working memory	
	 * @return a new pointer formula
	 */
	public PointerFormula createNewPointerFormula (String pointer);
	
	
	/**
	 * Create a new negated formula
	 * 
	 * @param negForm
	 * 			the formula to negate
	 * @return a new negated formula
	 */
	public NegatedFormula createNewNegatedFormula (Formula negForm);

	
	/**
	 * Create a new modal formula, the modal operator being one possible feature, and 
	 * which points to another formula
	 * 
	 * @param feat
	 * 			the modal operator describing the feature
	 * @param form
	 * 			the formula pointed to
	 * @return a new modal formula
	 */
	public ModalFormula createNewModalFormula (Feature feat, Formula form);
}
