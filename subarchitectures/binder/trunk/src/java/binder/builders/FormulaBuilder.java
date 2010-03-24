
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


package binder.builders;

import binder.arch.BinderException;
import binder.autogen.Feature;
import binder.autogen.formulae.BinaryOp;
import binder.autogen.formulae.ComplexFormula;
import binder.autogen.formulae.ElementaryFormula;
import binder.autogen.formulae.Formula;
import binder.autogen.formulae.ModalFormula;
import binder.autogen.formulae.NegatedFormula;
import binder.autogen.formulae.PointerFormula;

public class FormulaBuilder {
	
	
	private static int increment = 0;
	
	public static final String EXIST_PROP = "Exists";

	public static final String UNKNOWN_PROP = "unknown";
	
	
	/**
	 * Create a new, empty complex formula connected with the operator op
	 * 
	 * @param op
	 * 			the binary operator connecting the subformulae
	 * @return a new, empty complex formula
	 * @throws BinderException 
	 * 			if op is null
	 */
	public static ComplexFormula createNewComplexformula(BinaryOp op) throws BinderException {
		
		if (op == null) {
			throw new BinderException("error, op is null");
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
	public static void addSubformulaToComplexFormula (ComplexFormula cform, Formula newForm) throws BinderException {
		
		if (cform == null || newForm == null) {
			throw new BinderException("error, formula is null");
		}
		
		else if (cform.forms == null) {
			throw new BinderException("error, cform.forms is null");
		}
		
		Formula[] newForms = new Formula[cform.forms.length + 1];
		for (int i = 0 ; i < cform.forms.length ; i++) {
			newForms[i] = cform.forms[i];
		}
		newForms[cform.forms.length] = newForm;
		
		cform.forms = newForms;
	}


	/**
	 * Create a new elementary formula with proposition prop
	 * @param prop
	 * 			the logical proposition
	 * @return the new elementary formula
	 */
	public static ElementaryFormula createNewElementaryFormula (String prop) {	
		
		return new ElementaryFormula(getNewNominal(), prop);
	}
	
	
	/**
	 * Create a new elementary formula for entity existence
	 * @return the new elementary formula
	 */
	public static ElementaryFormula createNewElementaryFormulaForExist () {	
		
		return new ElementaryFormula(getNewNominal(), EXIST_PROP);
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
	public static NegatedFormula createNewNegatedFormula (Formula negForm) throws BinderException {
		
		if (negForm == null) {
			throw new BinderException("error, formula is null");
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
	public static ModalFormula createNewModalFormula (Feature feat, Formula form) throws BinderException {
		
		if (feat == null || form == null) {
			throw new BinderException("error, feat or form is null");
		}
		
		return new ModalFormula(getNewNominal(), feat, form);
	}

	
	/**
	 * Create a Modal formula which points to an elementary formula with proposition "unknown"
	 * 
	 * @param feat the feature, described as a modal operator
	 * @return the modal formula
	 * @throws BinderException
	 * 			if feat is null
	 */
	public static ModalFormula createModalFormulaWithUnknownValue (Feature feat) throws BinderException {
		
		if (feat == null) {
			throw new BinderException("error, feat is null");
		}
		
		return new ModalFormula(getNewNominal(), feat, createNewElementaryFormula(UNKNOWN_PROP));
	}

	
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
