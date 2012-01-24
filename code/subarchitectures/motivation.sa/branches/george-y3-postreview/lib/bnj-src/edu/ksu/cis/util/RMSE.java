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
 * Created on Jul 31, 2004
 *
 * 
 */
package edu.ksu.cis.util;

import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.core.values.*;

/**
 * @author Andrew King
 *
 * This class contains a static method to compute the RMSE of an array of approximate
 * and exact CPF values.
 */
public class RMSE {
	
	public static Value computeRMSE(CPF[] exact_probs, CPF[] approx_probs){
		
		Value rmse = new ValueDouble(0.0);
		long n = 0;
		
		for(int i = 0; i < exact_probs.length; i++){
			for(int c = 0; c < exact_probs[i].size(); c++){
				Value exact_val = exact_probs[i].get(c);
				Value approx_val = approx_probs[i].get(c);
				Value difference = Field.subtract(exact_val, approx_val);
				rmse = Field.add(rmse, Field.mult( difference, difference));
				n++;
			}
		}
		rmse = Field.divide(rmse, new ValueDouble(n));
		double rmse_d = Double.parseDouble(rmse.getExpr());
		rmse = new ValueDouble(Math.sqrt(rmse_d));
		return rmse;
	}

}
