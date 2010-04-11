package binder.ml;

import junit.framework.TestCase;

public class PredicateDataTest extends TestCase {
	
	private PredicateData predicate_types;
	
	protected void setUp() throws Exception {
		super.setUp();
		predicate_types = new PredicateData();
	}
	
	public void testAddPredicate() {
		
		String[] arg = new String[2];
		arg[0] = "belief_id";
		arg[1] = "color";
		
		try {
			predicate_types.addNewPredictate("1", new Predicate("Color", arg));
		} catch (MLException e) {
			assertTrue(false);
		} 
		
		assertTrue(predicate_types.hasPredicate("Color"));
		assertTrue(predicate_types.hasType("belief_id"));
		assertTrue(predicate_types.hasType("color"));
		assertTrue(predicate_types.hasBeliefForPredicate("Color", "1"));
	}
	
	public void testAddSecondBelief() {
		String[] arg = new String[2];
		arg[0] = "belief_id";
		arg[1] = "color";
		
		try {
			predicate_types.addNewPredictate("1", new Predicate("Color", arg));
		} catch (MLException e1) {
			assertTrue(false);
		}
		
		try {
			predicate_types.addPredicateToBelief("2", new Predicate("Color", arg));
		} catch (MLException e) {
			assertTrue(false);
		}
		
		assertTrue(predicate_types.hasBeliefForPredicate("Color", "2"));
		
	}
	
	public void testAddValuesForType() {
		String[] arg = new String[2];
		arg[0] = "belief_id";
		arg[1] = "color";
		
		try {
			predicate_types.addNewPredictate("1", new Predicate("Color", arg));
		} catch (MLException e) {
			assertTrue(false);
		}
		
		try {
			predicate_types.addValueForType("color", "Blue");
		} catch (MLException e) {
			assertTrue(false);
		}
		
		assertTrue(predicate_types.hasValueForType("color", "Blue"));
	}
	
	public void testAddValuesForTypeFail() {
		String[] arg = new String[2];
		arg[0] = "belief_id";
		arg[1] = "color";
		
		try {
			predicate_types.addNewPredictate("1", new Predicate("Color", arg));
		} catch (MLException e) {
			assertTrue(false);
		}
		
		try {
			predicate_types.addValueForType("color", "Blue");
		} catch (MLException e) {
			assertTrue(false);
		}
		
		assertFalse(predicate_types.hasValueForType("color", "Black"));
	}
	
	public void testAddDeletePredicate() {
		String[] arg = new String[2];
		arg[0] = "belief_id";
		arg[1] = "color";
		
		try {
			predicate_types.addNewPredictate("1", new Predicate("Color", arg));
		} catch (MLException e) {
			assertTrue(false);
		}
		
		predicate_types.removeBelief("1");
		
		assertFalse(predicate_types.hasPredicate("Color"));
		assertFalse(predicate_types.hasType("color"));
		assertFalse(predicate_types.hasType("belief_id"));
	}
	
	public void testAddTwoRemoveOne() {
		String[] arg1 = new String[2];
		arg1[0] = "belief_id";
		arg1[1] = "color";
		
		try {
			predicate_types.addNewPredictate("1", new Predicate("Color", arg1));
		} catch (MLException e) {
			assertTrue(false);
		}
		
		String[] arg2 = new String[2];
		arg2[0] = "belief_id";
		arg2[1] = "shape";
		
		try {
			predicate_types.addNewPredictate("2", new Predicate("Shape", arg2));
		} catch (MLException e) {
			assertTrue(false);
		}
		
		assertTrue(predicate_types.hasPredicate("Color"));
		assertTrue(predicate_types.hasPredicate("Shape"));
		assertTrue(predicate_types.hasType("color"));
		assertTrue(predicate_types.hasType("shape"));
		assertTrue(predicate_types.hasType("belief_id"));
		
		predicate_types.removeBelief("2");
		assertTrue(predicate_types.hasPredicate("Color"));
		assertFalse(predicate_types.hasPredicate("Shape"));
		assertTrue(predicate_types.hasType("color"));
		assertFalse(predicate_types.hasType("shape"));
		assertTrue(predicate_types.hasType("belief_id"));
		
		predicate_types.removeBelief("1");
		assertFalse(predicate_types.hasPredicate("Color"));
		assertFalse(predicate_types.hasPredicate("Shape"));
		assertFalse(predicate_types.hasType("color"));
		assertFalse(predicate_types.hasType("shape"));
		assertFalse(predicate_types.hasType("belief_id"));
	}
	
	protected void tearDown() throws Exception {
		super.tearDown();
	}

}
