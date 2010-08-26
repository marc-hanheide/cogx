
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
 
package de.dfki.lt.tr.dialmanagement.utils;

import static org.junit.Assert.*;

import org.junit.Test;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.NegatedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;


/**
 * Set of tests for reading the structure of a logical formula
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class FormulaUtilsTest {


	@Test
	public void simpleDisjFormula () throws DialogueException {
		String str = "blabla v bloblo v blibli";
		dFormula formula = FormulaUtils.constructFormula(str);
		
		assertTrue(formula instanceof ComplexFormula);
		
		assertEquals (((ComplexFormula)formula).forms.size(),3);
		
		assertEquals (((ComplexFormula)formula).op, BinaryOp.disj);
	}
	
	
	@Test
	public void simpleConjFormula () throws DialogueException {
		String str = "blabla ^ bloblo ^ blibli";
		dFormula formula = FormulaUtils.constructFormula(str);
		
		System.out.println(formula.getClass().getCanonicalName());
		assertTrue(formula instanceof ComplexFormula);
		
		assertEquals (((ComplexFormula)formula).forms.size(),3);
		
		assertEquals (((ComplexFormula)formula).op, BinaryOp.conj);
	}
	
	
	@Test
	public void simpleElementaryFormula () throws DialogueException {
		String str = "\"blabla bla bli blou\"";
		dFormula formula = FormulaUtils.constructFormula(str);
		
		assertTrue(formula instanceof ElementaryFormula);
		
		assertEquals(((ElementaryFormula)formula).prop, str);
		
	}
	
	

	@Test
	public void simpleElementaryFormula2 () throws DialogueException {
		String str = "blabla bla bli blou BLO";		
		try {
		dFormula formula = FormulaUtils.constructFormula(str);	
		System.out.println("form: " + FormulaUtils.getString(formula));
		assertFalse(((ElementaryFormula)formula).prop.equals("blabla"));
		}
		catch (Exception e) {	}
		
	}
	
	
	@Test
	public void embeddedFormula() throws DialogueException {
		String str = "(blabla v (blibli ^ blo ^ bla) v blibli)";
		dFormula formula = FormulaUtils.constructFormula(str);
		
		assertTrue(formula instanceof ComplexFormula);
		
		assertEquals (((ComplexFormula)formula).forms.size(),3);
		
		assertEquals (((ComplexFormula)formula).op, BinaryOp.disj);
		
		assertTrue (((ComplexFormula)formula).forms.get(1) instanceof ComplexFormula);
		
		assertEquals (((ComplexFormula)((ComplexFormula)formula).forms.get(1)).forms.size(), 3); 
		
		assertEquals (((ComplexFormula)((ComplexFormula)formula).forms.get(1)).op, BinaryOp.conj);
		
	}
	
	
	@Test
	public void simpleModalFormula() throws DialogueException {
		
		String str = "<op> blabla";
		dFormula formula = FormulaUtils.constructFormula(str);

		assertTrue(formula instanceof ModalFormula);
		
		assertEquals(((ModalFormula)formula).op, "op");
		
		assertTrue(((ModalFormula)formula).form instanceof ElementaryFormula);
		
		assertEquals(((ElementaryFormula)((ModalFormula)formula).form).prop, "blabla");
	}
	
	
	@Test
	public void simpleModalFormula2() throws DialogueException {
		
		String str = "<op> (blabla ^ bloblo)";
		dFormula formula = FormulaUtils.constructFormula(str);

		assertTrue(formula instanceof ModalFormula);
		
		assertEquals(((ModalFormula)formula).op, "op");
		
		assertTrue(((ModalFormula)formula).form instanceof ComplexFormula);
		
		assertEquals(((ComplexFormula)((ModalFormula)formula).form).forms.size(), 2);
	}
	
	
	
	@Test
	public void complexFormula() throws DialogueException {
		
		String str = "(prop1 ^ <op1> (prop2 ^ <op2> prop3) ^ prop4)";
		dFormula formula = FormulaUtils.constructFormula(str);

		assertEquals (((ComplexFormula)formula).forms.size(),3);
		
		assertEquals (((ComplexFormula)formula).op, BinaryOp.conj);
		
		assertTrue (((ComplexFormula)formula).forms.get(1) instanceof ModalFormula);
		
		assertEquals (((ModalFormula)((ComplexFormula)formula).forms.get(1)).op, "op1");
		
		assertTrue (((ModalFormula)((ComplexFormula)formula).forms.get(1)).form instanceof ComplexFormula);
		
		assertEquals(((ComplexFormula)((ModalFormula)((ComplexFormula)formula).forms.get(1)).form).forms.size(), 2);
		
		assertTrue(((ComplexFormula)((ModalFormula)((ComplexFormula)formula).forms.get(1)).form).forms.get(1) instanceof ModalFormula);

	}
	
	
	@Test
	public void complexFormula2() throws DialogueException {
		String str = "<Op1>(prop1 ^ prop2) ^ <Op2>(prop3 v prop4)";
		dFormula formula = FormulaUtils.constructFormula(str);
		
		assertTrue (formula instanceof ComplexFormula);
	}
	
	
	@Test
	public void modalFormula() throws DialogueException {
		
		String str = "<Belief>(<Ref>context1_1 ^ <ObjectType>ball)";
		
		dFormula formula = FormulaUtils.constructFormula(str);
		
		assertTrue(((ModalFormula)formula).form instanceof ComplexFormula);
		assertEquals(((ModalFormula)formula).op, "Belief");
		
		assertEquals(((ComplexFormula)((ModalFormula)formula).form).forms.size(), 2);
	}
	

	@Test
	public void brackettedFormula() throws DialogueException {
		
		String str = "(prop1 ^ prop2)";
		
		dFormula formula = FormulaUtils.constructFormula(str);

		assertTrue(formula instanceof ComplexFormula);
		
		assertEquals (((ComplexFormula)formula).forms.size(),2);
	}
	
	@Test
	public void negatedFormula() throws DialogueException {
		
		String str = " !(prop1 ^ prop2) ";
		
		dFormula formula = FormulaUtils.constructFormula(str);

		assertTrue(formula instanceof NegatedFormula);
		
		assertTrue(((NegatedFormula)formula).negForm instanceof ComplexFormula);
	}
	
	
	@Test
	public void intFormula() throws DialogueException {
		
		String str = "34";
		
		dFormula formula = FormulaUtils.constructFormula(str);

		assertTrue(formula instanceof IntegerFormula);
		
		assertEquals(((IntegerFormula)formula).val, 34);
		
	}
	
	
	
	@Test
	public void floatFormula() throws DialogueException {
		
		String str = "0.36f";
		
		dFormula formula = FormulaUtils.constructFormula(str);

		assertTrue(formula instanceof FloatFormula);
		
		System.out.println(((FloatFormula)formula).val);
		
		assertEquals(((FloatFormula)formula).val, 0.36f, 0.01f);
		
	}
	
	
	
	
	@Test
	public void booleanFormula() throws DialogueException {
		
		String str = "false";
		
		dFormula formula = FormulaUtils.constructFormula(str);

		assertTrue(formula instanceof BooleanFormula);
		
		assertEquals(((BooleanFormula)formula).val, false);
		
	}
	
	
	
	@Test 
	public void compareFormula1() throws DialogueException {
		
		String str1 = "blabla1 ^ blabla2";
		dFormula formula1 = FormulaUtils.constructFormula(str1);
		
		String str2 = "blabla2 ^ blabla1";
		dFormula formula2 = FormulaUtils.constructFormula(str2);
		
		assertTrue(FormulaUtils.isEqualTo(formula1, formula2));
	}
	
	@Test
	public void compareFormula2() throws DialogueException {
		
		String str1 = "blabla1 v blabla2";
		dFormula formula1 = FormulaUtils.constructFormula(str1);
		
		String str2 = "bloblo1 v bloblo2";
		dFormula formula2 = FormulaUtils.constructFormula(str2);
		
		assertFalse(FormulaUtils.isEqualTo(formula1, formula2));
	}
	
	@Test
	public void compareFormula3() throws DialogueException {
		
		String str1 = "blabla5 ^ <Op1>(blabla1 v blabla2 v (blabla3 ^ blabla4))";
		
		dFormula formula1 = FormulaUtils.constructFormula(str1);
		
		String str2 = "<Op1>(blabla2 v blabla1 v (blabla4 ^ blabla3)) ^ blabla5";
		dFormula formula2 = FormulaUtils.constructFormula(str2);
		
		assertTrue(FormulaUtils.isEqualTo(formula1, formula2));
	}
	

	@Test
	public void compareFormula4() throws DialogueException {
		
		String str1 = "blabla5 ^ <Op1>(blabla1 v blabla2 v (blabla3 ^ blabla4))";
		
		dFormula formula1 = FormulaUtils.constructFormula(str1);
		
		String str2 = "<Op1>(blabla6 v blabla1 v (blabla4 ^ blabla2)) ^ blabla5";
		dFormula formula2 = FormulaUtils.constructFormula(str2);
		
		assertFalse(FormulaUtils.isEqualTo(formula1, formula2));
	}
	
	
	@Test
	public void compareFormula5() throws DialogueException {
		
		String str1 = "<Belief>(<Colour>Blue ^ <Shape>Cylindrical ^ <Position>(<XCoord>0.3 ^ <YCoord>2.5))";
		dFormula formula1 = FormulaUtils.constructFormula(str1);
		
		String str2 = "<Belief>(<Position>(<XCoord>0.3 ^ <YCoord>2.5) ^ <Colour>Blue ^ <Shape>Cylindrical)";
		dFormula formula2 = FormulaUtils.constructFormula(str2);
		
		assertTrue(FormulaUtils.isEqualTo(formula1, formula2));
	}
		
}
