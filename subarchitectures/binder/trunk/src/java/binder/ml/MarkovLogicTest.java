/**
 * 
 */
package binder.ml;

import java.util.Set;
import java.util.Map;

import binder.autogen.featurecontent.*;
import binder.autogen.beliefs.Belief;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.DiscreteDistribution;
import binder.autogen.distribs.FeatureValueDistribution;
import binder.autogen.distribs.FeatureValueProbPair;
import binder.autogen.distribs.FormulaProbPair;
import binder.autogen.logicalcontent.BinaryOp;
import binder.autogen.logicalcontent.ComplexFormula;
import binder.autogen.logicalcontent.ElementaryFormula;
import binder.autogen.logicalcontent.Formula;
import binder.autogen.logicalcontent.ModalFormula;
import binder.autogen.logicalcontent.NegatedFormula;
import binder.autogen.logicalcontent.PointerFormula;
import binder.builders.FeatureValueBuilder;
import junit.framework.TestCase;

/**
 * @author Carsten Ehrler (carsten.ehrler@dfki.de)
 *
 */
public class MarkovLogicTest extends TestCase {
	
	private Belief belief;
	
	/* (non-Javadoc)
	 * @see junit.framework.TestCase#setUp()
	 */
	protected void setUp() throws Exception {
		super.setUp();
		String color = Feature.Colour.toString();
		
		ElementaryFormula elementary_formula = new ElementaryFormula();
		elementary_formula.prop = "Red";
		
		ModalFormula modal_formula = new ModalFormula();
		modal_formula.op = color;
		modal_formula.form = elementary_formula;
		
		NegatedFormula neg = new NegatedFormula();
		neg.negForm = modal_formula;
		
		PointerFormula pointer = new PointerFormula();
		pointer.beliefPointer = "1";
		
		ComplexFormula complex = new ComplexFormula();
		Formula[] formulae = {(Formula)pointer, (Formula)modal_formula};
		complex.forms = formulae;
		complex.op = BinaryOp.conj;
		
		FormulaProbPair pair = new FormulaProbPair();
		pair.form = complex;
		pair.prob = 0.8f;
		
		FormulaProbPair[] probs = {pair};
		
		DiscreteDistribution dist = new DiscreteDistribution();
		dist.pairs = probs;
		
		belief = new PerceptBelief();
		belief.content = dist;
		belief.id = "1";
	}
	
	public void testMarkovLogic() {
		MarkovLogic ml = new MarkovLogic();
		
		try {
			System.out.print(ml.addBeliefTEST(belief));
		}
		catch(MLException e) {
			assertTrue(false);
		}
		
	}
	
	public void testNamesToPredicates() {
		MarkovLogic ml = new MarkovLogic();
		try {
			ml.addBelief(belief);
		} catch (MLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		assertTrue(ml.testHasFeature(belief, Feature.Colour));
		
		Set<String> names = ml.testGetFeatureAlternatives(belief, Feature.Colour);
		assertTrue(names.contains("Red"));
	}
	
	public void testBinder1() {
		MarkovLogic ml = new MarkovLogic();
		
		MLPreferences preferences = new MLPreferences();
		
		Belief b1 = new PerceptBelief();
		Belief b2 = new PerceptBelief();
		
		b1.id = "1";
		b2.id = "2";
		
		FeatureValueProbPair[] f1 = new FeatureValueProbPair[1];
		FeatureValueProbPair[] f2 = new FeatureValueProbPair[1];
		
		f1[0] = new FeatureValueProbPair();
		f1[0].prob = 0.8f;
		f1[0].val = FeatureValueBuilder.createNewStringValue("Cylindrical");
		
		f2[0] = new FeatureValueProbPair();
		f2[0].prob = 0.9f;
		f2[0].val = FeatureValueBuilder.createNewStringValue("Mug");
		
		FeatureValueDistribution fvd1 = new FeatureValueDistribution();
		FeatureValueDistribution fvd2 = new FeatureValueDistribution();
		
		fvd1.feat = Feature.Shape;
		fvd1.values = f1;
		
		fvd2.feat = Feature.ObjectLabel;
		fvd2.values = f2;
		
		b1.content = fvd1;
		b2.content = fvd2;
		
		ml.init(preferences);
		try {
			ml.addBelief(b1);
			Map<Belief,Float> result = ml.infer(b2);
			System.out.println(result.toString());
		} catch (MLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/* (non-Javadoc)
	 * @see junit.framework.TestCase#tearDown()
	 */
	protected void tearDown() throws Exception {
		super.tearDown();
	}

}
