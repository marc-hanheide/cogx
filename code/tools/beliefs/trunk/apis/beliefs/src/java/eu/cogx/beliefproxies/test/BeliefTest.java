// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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

// =================================================================
// PACKAGE DEFINITION 
package eu.cogx.beliefproxies.test;

//=================================================================
//IMPORTS

// Java
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefproxies.factories.beliefs.CondIndependentFormulaBeliefFactory;
import eu.cogx.beliefproxies.proxies.beliefs.CondIndependentFormulaBeliefProxy;
import eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy;
import eu.cogx.beliefproxies.proxies.logicalcontent.FormulaValuesProxy;

//=================================================================
// TEST CLASS

/**
 * Unit tests for the <tt>de.dfki.lt.tr.beliefs.data.Belief</tt> class.
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100521
 * @started 100510
 */

public class BeliefTest {

	private CondIndependentFormulaBeliefProxy<dBelief> belief;

	@Before
	public void setUp() {
		try {
			belief = CondIndependentFormulaBeliefFactory.create(dBelief.class,
					"type", "id");
		} catch (InstantiationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/** A bare initialized belief can be given an identifier */
	@Test
	public void getNullContent() {
		CondIndependentFormulaDistributionsProxy content = belief.getContent();
		assertNull(content.get("test"));
	} // end test

	@Test
	public void setContent() {
		CondIndependentFormulaDistributionsProxy content = belief.getContent();
		content.initFeature("property1");
		assertNotNull(content.get("property1"));
		FormulaValuesProxy<?> f = content.getDistribution("property1");
		assertNotNull(f);
		f.add("value1", 0.5);
		assertEquals(0.5, f.getProb("value1"), 0.01);
		f.setProb("value1", 0.8);
		assertEquals(0.8, f.getProb("value1"), 0.01);
		assertEquals(0.0, f.getProb("value2"), 0.01);
		f.setProb("value2", 0.2);
		assertEquals(0.2, f.getProb("value2"), 0.01);

		content.initFeature("property2");
		assertNotNull(content.get("property2"));
		FormulaValuesProxy<?> f2 = content.getDistribution("property2");
		assertNotNull(f2);

		Object[][] init = { { "value1", new Double(0.8) },
				{ "value2", new Double(0.2) } };
		f2.setAll(init);

		System.out.println(belief.toString());

	}

	@Test
	public void setBelief() {
		belief.initFeature("property1");
		assertNotNull(belief.get("property1"));
		FormulaValuesProxy<?> f = belief.getDistribution("property1");
		assertNotNull(f);
		f.add("value1", 0.5);
		assertEquals(0.5, f.getProb("value1"), 0.01);
		f.setProb("value1", 0.8);
		assertEquals(0.8, f.getProb("value1"), 0.01);
		assertEquals(0.0, f.getProb("value2"), 0.01);
		f.setProb("value2", 0.2);
		assertEquals(0.2, f.getProb("value2"), 0.01);
		System.out.println(belief.toString());

	}

	@Test
	public void getForWm() {
		dBelief b = belief.get();
		assertEquals(b.id, "id");
		assertEquals(b.type, "type");
		assertNotNull(b);
		assertNotNull(b.content);
		assertNotNull(b.estatus);
		assertNotNull(b.frame);
		assertNotNull(b.hist);
	}

} // end class