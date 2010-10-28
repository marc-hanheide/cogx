/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.examples;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Arrays;
import java.util.Map.Entry;

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.beliefs.data.BasicDistribution;
import de.dfki.lt.tr.beliefs.data.Belief;
import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentDistributionBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.ProbFormula;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class BeliefUsageExampleTest {

	private dBelief incomingIndependentBelief;
	private Belief<dBelief> genericIndependentBelief;

	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception {
		genericIndependentBelief = Belief.create(dBelief.class);
		genericIndependentBelief.setContent(IndependentDistribution.create());
		incomingIndependentBelief = genericIndependentBelief.get();
	}

	@Test
	public void accessAsIndependentBelief() {
		// create a specific proxy for the incoming data
		IndependentDistributionBelief<dBelief> castedProxy = IndependentDistributionBelief
				.create(dBelief.class, incomingIndependentBelief);
		// as we have a specific proxy, we can directly access the content
		// through the type safe proxy
		// NOTE: It's a <b>proxy</b> so we a still accessing the belief itself
		// through that proxy
		IndependentDistribution content = castedProxy.getContent();

		// in this example it should be empty at this point
		assertTrue(content.isEmpty());

		// let's create some property to put in here:
		// first, create a fresh universal BasicDistribution
		BasicDistribution bd = BasicDistribution.create();

		// due to the strong type safety this cannot directly be put it into the
		// content. We have to do that explicitly and can then put it using the
		// standard Map.put method:
		castedProxy.getContent().put("newProperty", bd.asDistribution());

		// same way back: we are no reading that from the universal
		// BasicDistribution and cast it into a BasicDistribution proxy.
		BasicDistribution bdReread = BasicDistribution.create(content
				.get("newProperty"));
		// in fact, the put operation also set the internatal id of the
		// BasicDistribution that is inserted, check it out:
		assertEquals("newProperty", bdReread.getId());

		// you can also remove it easily:
		content.remove("newProperty");

		// should be empty now
		assertTrue(content.isEmpty());

		// of course you can also do more complex stuff: Here we create a
		IndependentDistribution outerIndepDist = IndependentDistribution
				.create();
		IndependentDistribution innerIndepDist = IndependentDistribution
				.create();
		BasicDistribution innerBasicDist = BasicDistribution.create();

		outerIndepDist.put("innerIndepDist", innerIndepDist.asDistribution());
		outerIndepDist.put("innerBasicDist", innerBasicDist.asDistribution());

		// you can also iterate all the distributions
		for (Distribution<ProbDistribution> i : outerIndepDist.values()) {
			// if this is a proxy for a BasicProbDistribution we can do the
			// cast...
			if (i.isFor(BasicProbDistribution.class)) {
				try {
					BasicDistribution accessAsBD = BasicDistribution.create(i);
					assertNotNull(accessAsBD.getDistribution());
				} catch (ClassCastException e) {
					fail();
				}
			} else { // otherwise this is doomed to fail and a
				// ClassCastException should be thrown
				try {
					BasicDistribution.create(i);
					fail("should have thrown an exception here!");
				} catch (ClassCastException e) {
					assertTrue(
							"that was correct... a cast exception was thrown",
							true);
				}
			}
		}

		// let's no do something more convenient: Let's create a special Basic
		// Distribution of Formulas:
		FormulaDistribution fd = FormulaDistribution.create();
		// a very comfortable way is to set several formula in one single step:
		Object[][] initArray = { { "value1", 0.2 }, { 2, 0.8 } };
		fd.addAll(initArray);
		// now we should have two values set: (i) a proposition "value1" with a
		// probability of 0.2 and (ii) an IntegerFormula with probability 0.8.
		assertEquals(2, fd.size());

		// access is quit easy again:
		// First, a FormulaDistribution implements Iteratable
		for (ProbFormula pf : fd) {
			// the ProbFormula is a pair of the formula and probability
			// so we can check: if it is a proposition (see above) we should
			// expect a probability of 0.2
			if (pf.getFormula().isProposition()) {
				assertEquals(0.2, pf.getProbability(), 0.01);
			} else { // otherwise it should be a probability of 0.8
				assertEquals(0.8, pf.getProbability(), 0.01);
			}
		}

		// we can also put this distribution in our outerIndepDist:
		outerIndepDist.put("somePropertyOfAlternativeFormulas", fd
				.asDistribution());
	}

	/**
	 * let's create an example using the comfortability classes for the quite
	 * common use case of a Belief that has a conditionally independent
	 * distribution of Formula distributions
	 */
	@Test
	public void createStronglyTypedComfortableAccess() {
		IndependentFormulaDistributionsBelief<dBelief> belief = IndependentFormulaDistributionsBelief
				.create(dBelief.class);

		// nice thing is: this has assigned all the correct factories and proxy
		// classes, allowing very comfortable access while assuring that we
		// can't put anything "wrong" in here:
		FormulaDistribution d = FormulaDistribution.create();
		belief.getContent().put("firstFormulaDist", d);

		// and with a second line we can populate this property with values (aka
		// formulas):
		d.addAll(new Object[][] { { 3, 0.5 }, { 5, 0.5 } });
		assertEquals(2, d.size());

		// if you now want to submit this via ICE/CAST, you should access the
		// underlying datatype of the belief using get()
		assertTrue(belief.get() instanceof dBelief);
		// now ship the dBelief and you are done!
		// oh wait! Of course you can still populate the Belief with more
		// meaningful information:
		belief.setAttributed("human", Arrays.asList("self"));
		// a belief should have a unique id!
		belief.setId("hurga:0");
		// this encodes for a VisualObject
		belief.setType("VisualObject");

		// check is indeed attributed now:
		assertTrue(belief.isAttributed());

		// we can send it now...
	}

	/**
	 * let's assume we received a dBelief from somewhere, and we are confident
	 * it contains an IndepentFormulaDistribution. So we cast it into a specific
	 * proxy, awaiting a ClassCastException in case our assumption was wrong.
	 * 
	 */
	@Test
	public void receiveStronglyTypedComfortableAccess() {
		// ***********************************************
		// this is just to create some dBelief:
		IndependentFormulaDistributionsBelief<dBelief> hiddenBelief = IndependentFormulaDistributionsBelief
				.create(dBelief.class);
		FormulaDistribution d = FormulaDistribution.create();
		hiddenBelief.getContent().put("firstFormulaDist", d);
		d.addAll(new Object[][] { { 3, 0.4 }, { 5, 0.6 } });
		hiddenBelief.setAttributed("human", Arrays.asList("self"));
		// a belief should have a unique id!
		hiddenBelief.setId("hurga:0");
		// this encodes for a VisualObject
		hiddenBelief.setType("VisualObject");
		dBelief sliceBelief = hiddenBelief.get();
		// ***********************************************
		
		// so, now we have a valid dBelief, here starts the interesting bit:
		// cast the dBelief into our Proxy
		IndependentFormulaDistributionsBelief<dBelief> belief = IndependentFormulaDistributionsBelief
				.create(dBelief.class, sliceBelief);

		// we can use toString to see the content:
		assertEquals(
				"CondIndepedentFormulaBeliefProxy [_proxyFor=class de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief, getId()=hurga:0, getType()=VisualObject, distribution: IndependentFormulaDistributions [firstFormulaDist=>FormulaDistribution [firstFormulaDist={3=>0.4 5=>0.6 }]  ] ]",
				belief.toString());

		// remember: the content of the Belief is a
		// IndependentFormulaDistribution, which implements
		// Map<FormulaDistribution>:
		for (Entry<String, FormulaDistribution> fd : belief.getContent()
				.entrySet()) {
			// let's see if we have in there waht we put in:
			assertEquals("firstFormulaDist", fd.getKey());
			FormulaDistribution formulas = fd.getValue();
			// note: also the id of the underlying BasicDistribution should be
			// the same now:
			assertEquals("firstFormulaDist", formulas.getId());

			// here is another interesting method: FormulaDistribution.getProb()
			// It searches all the Formulas to find the given one and reports it
			// assigned probability. Be careful: It reports the first finding
			// only. It is an unchecked assumption that formulas in a
			// distribution are unique! This is not checked yet!
			assertEquals(0.4, formulas.getProb(3), 0.01);
			assertEquals(0.6, formulas.getProb(5), 0.01);
			// we can also use this to set probabilities of existing formulas:
			formulas.setProb(3, 0.3);
			// and even can add a new one immediately:
			formulas.setProb("unknown", 0.1);
			assertEquals(
					"FormulaDistribution [firstFormulaDist={3=>0.3 5=>0.6 unknown=>0.1 }] ",
					formulas.toString());
			// look at it:
			System.out.println(formulas);
		}

	}

}
