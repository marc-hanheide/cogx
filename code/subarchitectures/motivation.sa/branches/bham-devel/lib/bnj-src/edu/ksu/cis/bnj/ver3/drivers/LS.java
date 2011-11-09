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
 */package edu.ksu.cis.bnj.ver3.drivers;
import java.io.FileInputStream;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceList;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceSample;
import edu.ksu.cis.bnj.ver3.streams.Evidence_Reader;
import edu.ksu.cis.bnj.ver3.streams.xml.Converter_xmlevidence;
import edu.ksu.cis.util.driver.Options;
/**
 * file: LS.java
 * 
 * @author Jeffrey M. Barber
 */
public class LS
{
	public static void onbn(Options opt, BeliefNetwork bn, EvidenceList evidence)
	{
		if (evidence == null)
		{
			edu.ksu.cis.bnj.ver3.inference.exact.LS ls = new edu.ksu.cis.bnj.ver3.inference.exact.LS();
			int trials = opt.getInteger("trials", 1);
			if (trials == 1)
			{
				trials = opt.getInteger("t", 1);
			}
			Options.outputln("running " + trials + " trials");
			for (int i = 0; i < trials; i++)
			{
				ls.run(bn);
			}
			Options.outputln(opt.renderInferenceResult(bn, ls));
		}
		else
		{
			edu.ksu.cis.bnj.ver3.inference.exact.LS ls = new edu.ksu.cis.bnj.ver3.inference.exact.LS();
			BeliefNode[] bnodes = bn.getNodes();
			for(int k = 0; k < evidence.size(); k++)
			{
				EvidenceSample es = evidence.getSample(k);
				for(int s = 0; s < bnodes.length; s++)
				{
					String observe = es.getValue( bnodes[s].getName() );
					System.out.println("settting evidence for " + bnodes[s].getName());
					if( observe == null || observe.equals(""))
					{
						System.out.print(" to hidden\n");
						bnodes[s].setEvidence(null);
					}
					else
					{
						Discrete D = (Discrete) bnodes[s].getDomain();
						int V = D.findName(observe);
						if(V >= 0)
						{
							bnodes[s].setEvidence(new DiscreteEvidence(V));
							System.out.print(" to " + observe + "\n");
							
						}
						else
						{
							System.out.print(" to hidden\n");
							bnodes[s].setEvidence(null);
						}
					}
				}
				ls.run(bn);
				Options.outputln(opt.renderInferenceResult(bn, ls));
			}
		}
	}
	public static void exec(Options opt)
	{
		Options.outputln("LS");
		opt.begin();
		String EvidenceFile = opt.get("evidence");
		EvidenceList evidence = null;
		
		if(EvidenceFile != null)
		{
			if(EvidenceFile.length() >= 3)
			{
				try
				{
						Converter_xmlevidence evidenceLoad = new Converter_xmlevidence();
						Evidence_Reader ER = new Evidence_Reader();
						evidenceLoad.load(new FileInputStream(EvidenceFile), ER);
						evidence = ER.get(0);
				}
				catch (Exception e)
				{
					
				}
			}
		}
		
		
		String f;
		while ((f = opt.file()) != null)
		{
			BeliefNetwork bn = Options.load(f);
			if (bn != null)
			{
				opt.BeginPerfMeasure();
				onbn(opt, bn, evidence);
				opt.EndPerfMeasure();
				opt.outputPerformanceReport(bn.getName());
			}
		}
	}
}