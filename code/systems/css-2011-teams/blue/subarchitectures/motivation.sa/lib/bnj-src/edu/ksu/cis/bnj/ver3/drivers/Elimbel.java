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
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.util.driver.Options;
/**
 * file: Elimbel.java
 * 
 * @author Jeffrey M. Barber
 */
public class Elimbel
{
	public static void onbn(Options opt, BeliefNetwork bn)
	{
		edu.ksu.cis.bnj.ver3.inference.exact.Elimbel elimbel = new edu.ksu.cis.bnj.ver3.inference.exact.Elimbel();
		int trials = opt.getInteger("trials", 1);
		if (trials == 1)
		{
			trials = opt.getInteger("t", 1);
		}
		Options.outputln("running " + trials + " trials");
		for (int i = 0; i < trials; i++)
		{
			elimbel.run(bn);
		}
		Options.outputln(opt.renderInferenceResult(bn, elimbel));
	}
	public static void exec(Options opt)
	{
		
		Options.outputln("Elimbel");
		opt.begin();
		String f;
		while ((f = opt.file()) != null)
		{
			BeliefNetwork bn = Options.load(f);
			if (bn != null)
			{
				opt.BeginPerfMeasure();
				onbn(opt, bn);
				opt.EndPerfMeasure();
				opt.outputPerformanceReport(bn.getName());
			}
		}
		/*
		BeliefNetwork bn = new BeliefNetwork();
		String[] boolDomain = {"true","false"};
		
		BeliefNode Cloudy		= new BeliefNode("Cloudy"	, new Discrete( boolDomain ));
		BeliefNode Rain			= new BeliefNode("Rain"		, new Discrete( boolDomain ));
		BeliefNode Sprinkler 	= new BeliefNode("Sprinkler", new Discrete( boolDomain ));
		BeliefNode WetGrass		= new BeliefNode("WetGrass"	, new Discrete( boolDomain ));
		
		bn.addBeliefNode(Rain);
		bn.addBeliefNode(Cloudy);
		bn.addBeliefNode(WetGrass);
		bn.addBeliefNode(Sprinkler);
		
		bn.connect(Cloudy, Rain);
		bn.connect(Cloudy, Sprinkler);
		bn.connect(Sprinkler, WetGrass);
		bn.connect(Rain, WetGrass);
		
		Cloudy.getCPF().put(0,new ValueDouble(0.5));
		Cloudy.getCPF().put(1,new ValueDouble(0.5));
		
  		Sprinkler.getCPF().put(0,new ValueDouble(0.1));
		Sprinkler.getCPF().put(1,new ValueDouble(0.5));
		Sprinkler.getCPF().put(2,new ValueDouble(0.9));
		Sprinkler.getCPF().put(3,new ValueDouble(0.5));

 
		Rain.getCPF().put(0,new ValueDouble(0.8));
		Rain.getCPF().put(1,new ValueDouble(0.2));
		Rain.getCPF().put(2,new ValueDouble(0.2));
		Rain.getCPF().put(3,new ValueDouble(0.8));

		WetGrass.getCPF().put(0,new ValueDouble(0.99));
		WetGrass.getCPF().put(1,new ValueDouble(0.9));
		WetGrass.getCPF().put(2,new ValueDouble(0.9));
		WetGrass.getCPF().put(3,new ValueDouble(0.0));
		WetGrass.getCPF().put(4,new ValueDouble(.01));
		WetGrass.getCPF().put(5,new ValueDouble(0.1));
		WetGrass.getCPF().put(6,new ValueDouble(0.1));
		WetGrass.getCPF().put(7,new ValueDouble(1.0));
		
		//Rain.setEvidence(new DiscreteEvidence((Discrete)Rain.getDomain(), "true"));;
		//WetGrass.setEvidence(new DiscreteEvidence((Discrete)Sprinkler.getDomain(), "true"));;
		
		//Rain.setEvidence(new PolyEvidence());
		//WetGrass.setEvidence(new PolyEvidence());
		Runtime r = Runtime.getRuntime();
		r.gc();
		
		long origfreemem = r.totalMemory() - r.freeMemory();

		long freemem;
		edu.ksu.cis.bnj.ver3.inference.exact.Elimbel VE = new edu.ksu.cis.bnj.ver3.inference.exact.Elimbel();
		long origTime = System.currentTimeMillis();

		for(int i = 0; i < 2000; i++)
		VE.run(bn);
			
		long nextmem = r.totalMemory() - r.freeMemory();
		freemem = nextmem - origfreemem;

		long learnTime = System.currentTimeMillis();
		System.out.println("Inference time = "+((learnTime - origTime) / 1000.0));
		System.out.println("Memory needed for running VarElim = "+freemem);

		r.gc();
		nextmem = r.totalMemory() - r.freeMemory();
		freemem = nextmem - origfreemem;
		System.out.println("Memory needed for the storage VarElim = "+freemem);
		
		System.out.println("r:"+ Cloudy.getName() + ":\n" + CPF.getString(VE.queryMarginal(Cloudy)));
		System.out.println("r:"+ Sprinkler.getName() + ":\n" + CPF.getString(VE.queryMarginal(Sprinkler)));
		System.out.println("r:"+ Rain.getName() + ":\n" + CPF.getString(VE.queryMarginal(Rain)));
		System.out.println("r:"+ WetGrass.getName() + ":\n" + CPF.getString(VE.queryMarginal(WetGrass)));		
		*/
	}
}