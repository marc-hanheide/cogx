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
 * Created on Sep 15, 2004
 *
 * 
 */
package edu.ksu.cis.bnj.ver3.inference.approximate.sampling;
import java.util.*;
//import edu.ksu.cis.bnj.ver3.core.values.*;
import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.core.values.*;

/**
 * @author Andrew King
 *
 * This class contains utility methods for forward sampling
 */
public class ForwardSampling {
	
	static public int sampleForward(BeliefNode node,  BeliefNetwork network){
		if(node.hasEvidence()){ return(-1); }
		CPF this_cpf = node.getCPF();
		int diff = 0;
		Random generator = new Random();
		BeliefNode[] domain_product = node.getCPF().getDomainProduct();
		//the fist element in the domain_product array is the node we are
		//instantiating (sampling
		int[] logical_query = new int[domain_product.length];
		//construct the logical query to start at the right place
		
		for(int i = 0; i < logical_query.length; i++){
			if(!domain_product[i].hasEvidence()) { logical_query[i] = 0; }
			else{
				DiscreteEvidence e = (DiscreteEvidence)domain_product[i].getEvidence();
				logical_query[i] = e.getDirectValue();
			}
		}
		
		int real_addy0 = this_cpf.addr2realaddr(logical_query);
		logical_query[0] = 1;
		int real_addy1 = this_cpf.addr2realaddr(logical_query);
		diff = real_addy1 - real_addy0;
		//Make a prob array with #outcomes in domain  - 1
		Value[] prob_interval = new Value[domain_product[0].getDomain().getOrder() - 1];
		//walk the CPF and grab the appropiate values, given the parent instantiations
		for(int i = 0; i < prob_interval.length; i++){
			prob_interval[0] = this_cpf.get(real_addy0 + (i * diff));
		}
		//Now we have the interval setup, so we need to gen a random number between 0 and 1, then see where
		//it falls in the interval
		double random = generator.nextDouble();
		Value value_random = new ValueDouble(random);
		int new_ev_index = -1;
		for(int i = 0; i < prob_interval.length; i++){
			if(intervalCollide(prob_interval, i, value_random)) new_ev_index = i;
		}
		return(new_ev_index);
	}

	static private boolean intervalCollide(Value[] interval, int index, Value random){
		boolean ret = false;
		//check the corner cases
		if(index == 0){
			if(((ValueDouble)interval[0]).getValue() > ((ValueDouble)random).getValue()) return(true);
		}
		if(index == interval.length - 1){
			if(((ValueDouble)interval[0]).getValue() < ((ValueDouble)random).getValue()) return(true);
		}
		//done with corner, now the middle
		if(((ValueDouble)interval[index - 1]).getValue() < ((ValueDouble)random).getValue() &  ((ValueDouble)interval[index]).getValue() > ((ValueDouble)random).getValue()){
			return true;
		}
		return false;
	}
}
