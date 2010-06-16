package de.dfki.lt.tr.beliefs.factories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.LinkedList;

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.beliefs.data.Formulas;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.util.ProbFormula;

public class FormulasFactoryTest {

	private FormulasFactory factory;

	@Before
	public void setUp() throws Exception {
		factory = new FormulasFactory();
	}

	@Test
	public void testCreate() {
		Formulas s = factory.create();
		assertFalse(s.iterator().hasNext());
	}

	@Test
	public void testCreateObject() {
		FormulaValues pd = new FormulaValues(new LinkedList<FormulaProbPair>());
		Formulas s = factory.create(pd);
		assertFalse(s.iterator().hasNext());
		s.add(5, 0.5);
		assertEquals(0.5, s.getProb(5), 0.01);
		assertTrue(s.iterator().hasNext());
	}

	@Test
	public void testAddAndGetObject() {
		Formulas s = factory.create();
		assertFalse(s.iterator().hasNext());
		s.add(5, 0.5);
		assertEquals(0.5, s.getProb(5), 0.01);
		assertTrue(s.iterator().hasNext());

		for (ProbFormula i : s) {
			assertFalse(i.getFormula().isProposition());
			assertTrue(i.getFormula().get() instanceof IntegerFormula);
			assertEquals(5, i.getFormula().getInteger());
		}

	}

}
