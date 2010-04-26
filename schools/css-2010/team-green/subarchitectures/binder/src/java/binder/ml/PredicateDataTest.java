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
	
	private float EPSILON = 0.000001f;
	
	private Float convertProbabilityToWeight(float prob) throws MLException {
		if(Float.compare(prob, 1f) > 0 || Float.compare(prob, 0f) < 0) {
			throw new MLException("Value is not a probability: " + prob);
		}
		// handle the border case, where denominator approaches 0 or 1
		if(Float.compare(prob, 1f) >= 0) {
			return Float.MAX_VALUE;
		}
		if(Float.compare(prob, 0f) <= 0) {
			return Float.MIN_VALUE;
		}
		else {
			return new Float(Math.log(prob / (1f - prob)));
		}
	}

	/**
	 * Convert a weight baqck to a probability according to
	 * p = exp(w)/(1+exp(w))
	 * 
	 * @param weight
	 * @return the corresponding probability
	 * @throws MLException 
	 */
	private Float convertWeightToProbability(float weight) throws MLException {
		// handle the border cases
		if(Float.compare(weight, Float.MIN_VALUE) == 0) {
			return 0f;
		}
		if(Float.compare(weight, Float.MAX_VALUE) == 0) {
			return 1f;
		}
		
		float a = (float)Math.exp(weight);
		
		a = a/(a+1);
		
		if(Float.compare(a, Float.NaN) == 0) {
			throw new MLException("Error in probability conversion...");
		}
		
		return a;
	}
	
	public void testMath() {
		try {
			checkProbability(0f);
			checkProbability(0.0001f);
			checkProbability(0.0001f);
			checkProbability(0.5f);
			checkProbability(0.9999f);
			checkProbability(0.9999f);
			checkProbability(1f);
		} catch (MLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		try {
			checkProbability(-0.00001f);
			assertTrue(false);
		}catch (MLException e) {
			assertTrue(true);
		}
		
		try {
			checkProbability(1.00001f);
			assertTrue(false);
		}catch (MLException e) {
			assertTrue(true);
		}
	}
	
	private void checkProbability(float a) throws MLException {
		System.out.println("Input Probability: " + a);
		float weight = convertProbabilityToWeight(a);
		System.out.println("Weight: " + weight);
		float prob = convertWeightToProbability(weight);
		System.out.println("Probability: " + prob);
		System.out.println();
		assertTrue(Float.compare(a, prob) == 0);
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
