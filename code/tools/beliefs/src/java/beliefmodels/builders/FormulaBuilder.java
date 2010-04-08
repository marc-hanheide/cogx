
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


package beliefmodels.builders;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.logicalcontent.BinaryOp;
import beliefmodels.autogen.logicalcontent.ComplexFormula;
import beliefmodels.autogen.logicalcontent.ElementaryFormula;
import beliefmodels.autogen.logicalcontent.Formula;
import beliefmodels.autogen.logicalcontent.ModalFormula;
import beliefmodels.autogen.logicalcontent.NegatedFormula;
import beliefmodels.autogen.logicalcontent.PointerFormula;

public class FormulaBuilder {
	
	
	private static int increment = 0;
	
	public static final String EXIST_PROP = "Exists";
	
	
	// ============================================
	// GENERAL FORMULA CONSTRUCTION METHODS
	// ============================================


	/**
	 * Create a new elementary formula with proposition prop
	 * @param prop
	 * 			the logical proposition
	 * @return the new elementary formula
	 */
	public static ElementaryFormula createNewElFormula (String prop) {	
		
		return new ElementaryFormula(getNewNominal(), prop);
	}
	
	
	
	/**
	 * Create a new, empty complex formula connected with the operator op
	 * 
	 * @param op
	 * 			the binary operator connecting the subformulae
	 * @return a new, empty complex formula
	 * @throws BinderException 
	 * 			if op is null
	 */
	public static ComplexFormula createNewComplexformula(BinaryOp op) throws BeliefException {
		
		if (op == null) {
			throw new BeliefException("error, op is null");
		}
		
		Formula[] forms = new Formula[0];
		ComplexFormula form = new ComplexFormula (getNewNominal(), forms, op);
		return form;
	}
	
	
	/**
	 * Add a new subformula to an existing complex formula
	 * @param cform
	 * 			the existing complex formula
	 * @param newForm
	 * 			the new formula to add
	 * @throws BinderException 
	 * 			if cform or newForm is null
	 * @post cform now contains newForm
	 */
	public static void addSubformulaToComplexFormula (ComplexFormula cform, Formula newForm) throws BeliefException {
		
		if (cform == null || newForm == null) {
			throw new BeliefException("error, formula is null");
		}
		
		else if (cform.forms == null) {
			throw new BeliefException("error, cform.forms is null");
		}
		
		Formula[] newForms = new Formula[cform.forms.length + 1];
		for (int i = 0 ; i < cform.forms.length ; i++) {
			newForms[i] = cform.forms[i];
		}
		newForms[cform.forms.length] = newForm;
		
		cform.forms = newForms;
	}

	
	/**
	 * Create a new pointer formula, i.e. a formula whose proposition is a pointer
	 * to another belief
	 * 
	 * @param pointer 
	 * 			a pointer to another belief identifier, which must currently exist 
	 * 			onto the binder working memory	
	 * @return a new pointer formula
	 */
	public static PointerFormula createNewPointerFormula (String pointer) {
		
		return new PointerFormula(getNewNominal(), pointer);
	}
	
	
	/**
	 * Create a new negated formula
	 * 
	 * @param negForm
	 * 			the formula to negate
	 * @return a new negated formula
	 * @throws BinderException 
	 * 			if negform is null
	 */
	public static NegatedFormula createNewNegatedFormula (Formula negForm) throws BeliefException {
		
		if (negForm == null) {
			throw new BeliefException("error, formula is null");
		}
		
		return new NegatedFormula(getNewNominal(), negForm);
	}

	
	/**
	 * Create a new modal formula, the modal operator being one possible feature, and 
	 * which points to another formula
	 * 
	 * @param feat
	 * 			the modal operator describing the feature
	 * @param form
	 * 			the formula pointed to
	 * @return a new modal formula
	 * @throws BinderException 
	 * 			if feat or form is null
	 */
	public static ModalFormula createNewModalFormula (String op, Formula form) throws BeliefException {
		
		if (op == null || form == null) {
			throw new BeliefException("error, feat or form is null");
		}
		
		return new ModalFormula(getNewNominal(), op, form);
	}

	
	// ============================================
	// FACILITY METHODS FOR FREQUENT FORMULAE
	// ============================================

	
	
	/**
	 * Create a new elementary formula for entity existence
	 * @return the new elementary formula
	 */
	public static ElementaryFormula createNewExistFormula () {	
		
		return new ElementaryFormula(getNewNominal(), EXIST_PROP);
	}
	

	
	
	// ============================================
	// UTILITIES
	// ============================================

	
	
	/**
	 * Forge a new nominal
	 * 
	 * @return a new nominal
	 */
	private static int getNewNominal() {
		increment++;
		return increment;
	}
}
