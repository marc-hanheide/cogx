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
 */package edu.ksu.cis.bnj.ver3.learning.data;
/*!
 * \file SampleDataTest.java
 * \author Jeffrey M. Barber
 */
public class SampleDataTest
{
	public static void main(String[] args)
	{
		try
		{
			DiscreteDataSourceARFF data = new DiscreteDataSourceARFF("asia1000data.arff");
			System.out.println("data loaded\n");
			int[] q = data.getQueryRegister();
			
			// a Simple Query
			// P( Cancer = Present | Smoking = Smoker ) = P ( Cancer = Present AND Smoking = Smoker ) / P ( Smoking = Smoker )

			// need to access the columns from console
			int cCancer  = data.findColumn("Cancer");
			int cSmoking = data.findColumn("Smoking");

			// P ( Cancer = Present AND Smoking = Smoker )
			data.reset(q); // fill with -1
			q[cCancer] = data.getDiscrete(cCancer).findName("Present");
			q[cSmoking] = data.getDiscrete(cSmoking).findName("Smoker");

			System.out.println("calculating P ( Cancer = Present AND Smoking = Smoker )");
			int pCPSS = data.query(q);
			System.out.println(".. " + pCPSS);
			// P ( Cancer = Present AND Smoking = Smoker )

			data.reset(q); // fill with -1
			q[cSmoking] = data.getDiscrete(cSmoking).findName("Smoker");

			System.out.println("calculating P ( Smoking = Smoker )");
			int pSS = data.query(q);
			System.out.println(".. " + pSS);
			
			System.out.println(" " + pCPSS + " / " + pSS);
			System.out.println(" Result = " + ((double) pCPSS / (double) pSS));
			
		} catch (Exception e)
		{
			
		}
	}
}
