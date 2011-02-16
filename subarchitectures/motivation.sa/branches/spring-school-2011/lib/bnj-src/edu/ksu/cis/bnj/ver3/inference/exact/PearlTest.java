/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 *//*
 * Created on Aug 1, 2004
 */
package edu.ksu.cis.bnj.ver3.inference.exact;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.util.driver.Options;
import junit.framework.TestCase;

/**
 * @author Julie Thornton
 * @author Charlie Thornton
 * 
 * Does six different unit tests on Pearl's algorithm:
 * 1) sprinkler2.xml - no evidence
 * 2) sprinkler2.xml - WetGrass = false
 * 3) asiaNoS_B.xml - no evidence
 * 4) asiaNoS_B.xml - TbOrCa = False
 * 5) alarmPoly.xml - no evidence
 * 6) alarmPoly.xml - Press = Normal
 * 
 * where sprinkler2.xml has edge cloudy->rain deleted,
 * asiaNoS_B.xml has edge smoking->bronchitis deleted, and
 * alarmPoly.xml is a polytree version of alarm.xml (I didn't
 * keep track of which edges I deleted.)
 */
public class PearlTest extends TestCase {

	private Inference pearlAlg;
	
	public PearlTest() {
		super("Tests For Pearl's Algorithm");
		pearlAlg = new Pearl();
	}
	
	public void setUp() {
		
	}
	
	public void tearDown() {
		
	}
	
	public void test_sprinklerNoEv() {
		BeliefNetwork network = Options.load("sprinkler2.xml");
		
		pearlAlg.run(network);
		double tol = 0.001;
		
		BeliefNode[] nodes = network.getNodes();
		CPF marginal = null;
		
		// wet grass
		marginal = pearlAlg.queryMarginal(nodes[0]);
		assertEquals(0.402, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.598, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// cloudy
		marginal = pearlAlg.queryMarginal(nodes[1]);
		assertEquals(0.5, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.5, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// sprinkler
		marginal = pearlAlg.queryMarginal(nodes[2]);
		assertEquals(0.7, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.3, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// rain
		marginal = pearlAlg.queryMarginal(nodes[3]);
		assertEquals(0.5, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.5, ((ValueDouble)marginal.get(1)).getValue(), tol);
	}
	
	public void test_3NoEv() {
		BeliefNetwork network = Options.load("pearlExample.xml");
		
		pearlAlg.run(network);
		double tol = 0.001;
		
		BeliefNode[] nodes = network.getNodes();
		CPF marginal = null;
		
		// wet grass
		marginal = pearlAlg.queryMarginal(nodes[0]);
		assertEquals(0.402, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.598, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// sprinkler
		marginal = pearlAlg.queryMarginal(nodes[1]);
		assertEquals(0.7, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.3, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// rain
		marginal = pearlAlg.queryMarginal(nodes[2]);
		assertEquals(0.5, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.5, ((ValueDouble)marginal.get(1)).getValue(), tol);
	}
	
	public void test_3DryGrass() {
		BeliefNetwork network = Options.load("pearlExample.xml");
		BeliefNode[] nodes = network.getNodes();
		DiscreteEvidence de = new DiscreteEvidence((Discrete) nodes[0].getDomain(), "false");
		nodes[0].setEvidence(de);
		
		pearlAlg.run(network);
		double tol = 0.001;
		
		CPF marginal = null;
		
		// wet grass
		marginal = pearlAlg.queryMarginal(nodes[0]);
		assertEquals(1, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// sprinkler
		marginal = pearlAlg.queryMarginal(nodes[1]);
		assertEquals(0.959, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.041, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// rain
		marginal = pearlAlg.queryMarginal(nodes[2]);
		assertEquals(0.909, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.091, ((ValueDouble)marginal.get(1)).getValue(), tol);
	}
	
	public void test_sprinklerWGev() {
		BeliefNetwork network = Options.load("sprinkler2.xml");
		BeliefNode[] nodes = network.getNodes();
		DiscreteEvidence de = new DiscreteEvidence((Discrete) nodes[0].getDomain(), "false");
		nodes[0].setEvidence(de);
		
		pearlAlg.run(network);
		double tol = 0.001;
		
		nodes = network.getNodes();
		CPF marginal = null;
		
		// wet grass
		marginal = pearlAlg.queryMarginal(nodes[0]);
		assertEquals(1.0, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.0, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// cloudy
		marginal = pearlAlg.queryMarginal(nodes[1]);
		assertEquals(0.377, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.623, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// sprinkler
		marginal = pearlAlg.queryMarginal(nodes[2]);
		assertEquals(0.959, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.041, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// rain
		marginal = pearlAlg.queryMarginal(nodes[3]);
		assertEquals(0.909, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.091, ((ValueDouble)marginal.get(1)).getValue(), tol);
	}
	
	public void test_asiaNoEv() {
		BeliefNetwork network = Options.load("asiaNoS_B.xml");
		
		pearlAlg.run(network);
		double tol = 0.001;
		
		BeliefNode[] nodes = network.getNodes();
		CPF marginal = null;
		
		// smoking
		marginal = pearlAlg.queryMarginal(nodes[0]);
		assertEquals(0.5, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.5, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// x-ray
		marginal = pearlAlg.queryMarginal(nodes[1]);
		assertEquals(0.11, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.89, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// dyspnea		 
		marginal = pearlAlg.queryMarginal(nodes[2]);
		assertEquals(0.439, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.561, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// visit asia
		marginal = pearlAlg.queryMarginal(nodes[3]);
		assertEquals(0.01, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.99, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// bronchitis
		marginal = pearlAlg.queryMarginal(nodes[4]);
		assertEquals(0.45, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.55, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// tuberculosis
		marginal = pearlAlg.queryMarginal(nodes[5]);
		assertEquals(0.01, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.99, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// cancer
		marginal = pearlAlg.queryMarginal(nodes[6]);
		assertEquals(0.055, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.945, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// tuberculosis or cancer
		marginal = pearlAlg.queryMarginal(nodes[7]);
		assertEquals(0.065, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.935, ((ValueDouble)marginal.get(1)).getValue(), tol);
	}
	
	public void test_asiaToCEv() {
		BeliefNetwork network = Options.load("asiaNoS_B.xml");
		BeliefNode[] nodes = network.getNodes();
		DiscreteEvidence de = new DiscreteEvidence((Discrete) nodes[7].getDomain(), "False");
		nodes[7].setEvidence(de);
		
		pearlAlg.run(network);
		double tol = 0.001;
		
		nodes = network.getNodes();
		CPF marginal = null;
		
		// smoking
		marginal = pearlAlg.queryMarginal(nodes[0]);
		assertEquals(0.476, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.524, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// x-ray
		marginal = pearlAlg.queryMarginal(nodes[1]);
		assertEquals(0.05, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.95, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// dyspnea		 
		marginal = pearlAlg.queryMarginal(nodes[2]);
		assertEquals(0.415, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.585, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// visit asia
		marginal = pearlAlg.queryMarginal(nodes[3]);
		assertEquals(0.01, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.99, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// bronchitis
		marginal = pearlAlg.queryMarginal(nodes[4]);
		assertEquals(0.45, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.55, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// tuberculosis
		marginal = pearlAlg.queryMarginal(nodes[5]);
		assertEquals(0.0, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(1.0, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// cancer
		marginal = pearlAlg.queryMarginal(nodes[6]);
		assertEquals(0.0, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(1.0, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// tuberculosis or cancer
		marginal = pearlAlg.queryMarginal(nodes[7]);
		assertEquals(0.0, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(1.0, ((ValueDouble)marginal.get(1)).getValue(), tol);
	}
	
/*	public void test_alarmPolyNoEv() {
		BeliefNetwork network = Options.load("alarmPoly.xml");
		pearlAlg.run(network);
		
		double tol = 0.001;
		BeliefNode[] nodes = network.getNodes();
		CPF marginal = null;	
		
		// KinkedTube
		marginal = pearlAlg.queryMarginal(nodes[11]);
		assertEquals(0.04, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.96, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// VentTube
		marginal = pearlAlg.queryMarginal(nodes[10]);
		assertEquals(0.067, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.028, ((ValueDouble)marginal.get(1)).getValue(), tol);
		assertEquals(0.877, ((ValueDouble)marginal.get(2)).getValue(), tol);
		assertEquals(0.028, ((ValueDouble)marginal.get(3)).getValue(), tol);
		
		// Press
		marginal = pearlAlg.queryMarginal(nodes[12]);
		assertEquals(0.087, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.068, ((ValueDouble)marginal.get(1)).getValue(), tol);
		assertEquals(0.021, ((ValueDouble)marginal.get(2)).getValue(), tol);
		assertEquals(0.825, ((ValueDouble)marginal.get(3)).getValue(), tol);
	}
	
	public void test_alarmPolyEv() {
		BeliefNetwork network = Options.load("alarmPoly.xml");
		BeliefNode[] nodes = network.getNodes();
		DiscreteEvidence de = new DiscreteEvidence((Discrete) nodes[12].getDomain(), "Normal");
		nodes[12].setEvidence(de);
		pearlAlg.run(network);
		
		double tol = 0.001;
		nodes = network.getNodes();
		CPF marginal = null;	
		
		// KinkedTube
		marginal = pearlAlg.queryMarginal(nodes[11]);
		assertEquals(0.113, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.887, ((ValueDouble)marginal.get(1)).getValue(), tol);
		
		// VentTube
		marginal = pearlAlg.queryMarginal(nodes[10]);
		assertEquals(0.039, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.362, ((ValueDouble)marginal.get(1)).getValue(), tol);
		assertEquals(0.490, ((ValueDouble)marginal.get(2)).getValue(), tol);
		assertEquals(0.109, ((ValueDouble)marginal.get(3)).getValue(), tol);
		
		// Press
		marginal = pearlAlg.queryMarginal(nodes[12]);
		assertEquals(0.0, ((ValueDouble)marginal.get(0)).getValue(), tol);
		assertEquals(0.0, ((ValueDouble)marginal.get(1)).getValue(), tol);
		assertEquals(1.0, ((ValueDouble)marginal.get(2)).getValue(), tol);
		assertEquals(0.0, ((ValueDouble)marginal.get(3)).getValue(), tol);
	}*/
}
