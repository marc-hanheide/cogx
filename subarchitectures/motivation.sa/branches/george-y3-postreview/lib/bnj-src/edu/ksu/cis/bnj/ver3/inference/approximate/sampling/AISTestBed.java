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
 * Created on Jul 26, 2004
 *
 * 
 */
package edu.ksu.cis.bnj.ver3.inference.approximate.sampling;


import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.util.driver.*;
import edu.ksu.cis.bnj.ver3.inference.exact.*;
import edu.ksu.cis.bnj.ver3.core.values.*;
import java.util.*;
/**
 * @author Andrew King
 *
 * This class is for running AIS w/o going through the main BNJ entry point (drivers.MASTER)
 * Don't run AIS from here unless you really need to, know what you are doing.
 */
public class AISTestBed {
	
	
	public static void main(String[] args){
		Vector appro_probs = new Vector();
		Vector exact_probs = new Vector();
		int samples = new Integer(args[1]).intValue();
		int interval = new Integer(args[2]).intValue();
		Options options = new Options();

		BeliefNetwork bn = Options.load(args[0]);
		BeliefNode[] nodes = bn.getNodes();

	

		AIS ais = new AIS();
		LS ls  = new LS();
		System.out.println("LOADED: " + args[0]);
		System.out.println("NET NAME: " + bn.getName());
		System.out.println("NUM NODES: " + bn.getNodes().length);

		ais.setNumSamples(samples);
		ais.setInterval(interval);

		//ais.setSamples(samples)
		//ais.setInterval(interval)

		ais.run(bn);
		ls.run(bn);
		
		//get probs from LS marginalize
		for(int i = 0; i < nodes.length; i++){
			CPF this_cpf = ls.queryMarginal(nodes[i]);
			Vector v = new Vector();
			for(int c = 0; c < this_cpf.size(); c++ ){
				ValueDouble val_doub = (ValueDouble)this_cpf.get(c);
				double d = val_doub.getValue();
				Double d_obj = new Double(d);
				v.add(d_obj);
			}
			exact_probs.add(v);
		}
		
		appro_probs = ais.probabilityArray;
		
		double d = ais.eval.computeRMSE_DforOneFromTwoProbs(exact_probs, appro_probs);
		System.out.println("Samples: " + samples + " Interval: " + interval + " RMSE: " + d);
	}
	

}
