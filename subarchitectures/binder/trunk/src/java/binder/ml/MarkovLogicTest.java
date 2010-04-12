/**
 * 
 */
package binder.ml;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.Map;

import junit.framework.TestCase;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.FormulaProbPair;
import beliefmodels.autogen.distribs.FormulaValues;
import beliefmodels.autogen.logicalcontent.BinaryOp;
import beliefmodels.autogen.logicalcontent.ComplexFormula;
import beliefmodels.autogen.logicalcontent.ElementaryFormula;
import beliefmodels.autogen.logicalcontent.Formula;
import beliefmodels.autogen.logicalcontent.ModalFormula;
import beliefmodels.autogen.logicalcontent.NegatedFormula;
import beliefmodels.autogen.logicalcontent.PointerFormula;
import beliefmodels.builders.FeatureValueBuilder;
 
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
		
		ElementaryFormula elementary_formula = new ElementaryFormula();
		elementary_formula.prop = "Red";
		
		ModalFormula modal_formula = new ModalFormula();
		modal_formula.op = "colour";
		modal_formula.form = elementary_formula;
		
		NegatedFormula neg = new NegatedFormula();
		neg.negForm = modal_formula;
		
		PointerFormula pointer = new PointerFormula();
		pointer.beliefPointer = "1";
		
		ComplexFormula complex = new ComplexFormula();
		List<Formula> formulae = new LinkedList<Formula>();
		formulae.add(pointer);
		formulae.add(modal_formula);
		complex.forms = formulae;
		complex.op = BinaryOp.conj;
		
		FormulaProbPair pair = new FormulaProbPair();
		pair.form = complex;
		pair.prob = 0.8f;
		
		List<FormulaProbPair> probs = new LinkedList<FormulaProbPair>();
		probs.add(pair);
		
		BasicProbDistribution dist = new BasicProbDistribution();
		dist.values = new FormulaValues();
		((FormulaValues)dist.values).pairs = probs;
		
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
		
		assertTrue(ml.testHasFeature(belief, "colour"));
		
		Set<String> names = ml.testGetFeatureAlternatives(belief, "colour");
		assertTrue(names.contains("Red"));
	}
	
	public void testBinder1() {
		MarkovLogic ml = new MarkovLogic();
		
		MLPreferences preferences = new MLPreferences();
		
		Belief b1 = new PerceptBelief();
		Belief b2 = new PerceptBelief();
		
		b1.id = "1";
		b2.id = "2";
		
		FeatureValues f1 = new FeatureValues();
		f1.values = new ArrayList<FeatureValueProbPair>();
		FeatureValues f2 = new FeatureValues();
		f2.values = new ArrayList<FeatureValueProbPair>();
		
		FeatureValueProbPair f1_0 = new FeatureValueProbPair();
		f1_0.prob = 0.8f;
		try {
		f1_0.val = FeatureValueBuilder.createNewStringValue("Cylindrical");
		f1.values.add(f1_0);
		
		FeatureValueProbPair f2_0 = new FeatureValueProbPair();
		f2_0.prob = 0.9f;
		f2_0.val = FeatureValueBuilder.createNewStringValue("Mug");
		f2.values.add(f2_0);
		}
		catch (BeliefException e) {
			e.printStackTrace();
		}
		BasicProbDistribution fvd1 = new BasicProbDistribution();
		BasicProbDistribution fvd2 = new BasicProbDistribution();
		
	//	fvd1.feat = "shape";
		fvd1.values = f1;
		
	//	fvd2.feat = Feature.ObjectLabel;
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
