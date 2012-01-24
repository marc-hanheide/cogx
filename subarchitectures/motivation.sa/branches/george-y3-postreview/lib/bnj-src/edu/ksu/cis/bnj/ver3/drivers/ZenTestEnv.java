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
import java.io.FileOutputStream;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceList;
import edu.ksu.cis.bnj.ver3.dynamic.DynamicTyping;
import edu.ksu.cis.bnj.ver3.dynamic.Filtering;
import edu.ksu.cis.bnj.ver3.dynamic.UnRoll;
import edu.ksu.cis.bnj.ver3.influence.nfseq.SequentialNonForgetting;
import edu.ksu.cis.bnj.ver3.plugin.IOPlugInLoader;
import edu.ksu.cis.bnj.ver3.streams.Evidence_Reader;
import edu.ksu.cis.bnj.ver3.streams.Evidence_Writer;
import edu.ksu.cis.bnj.ver3.streams.Exporter;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Writer;
import edu.ksu.cis.bnj.ver3.streams.xml.Converter_xmlevidence;

import edu.ksu.cis.util.GlobalOptions;
import edu.ksu.cis.util.driver.Options;



/**
 * file: ZenTestEnv.java
 * @author Jeffrey M. Barber
 */
public class ZenTestEnv
{
	
	
	public static void onbn(Options opt, BeliefNetwork bn, EvidenceList E)
	{
//		SequentialNonForgetting snf = new SequentialNonForgetting();
//		snf.solve(bn);
		
		/*
		BeliefNetwork res = UnRoll.execute(bn, 10);
		
		try
		{
		
			IOPlugInLoader pil = IOPlugInLoader.getInstance();
			Exporter EXP = pil.GetExportersByExt(pil.GetExt(".xml"));
			EXP.save(new FileOutputStream("test_dbn.xml"));
			OmniFormatV1_Writer.Write( res , EXP.getStream1());
			
		} catch (Exception e)
		{
		}
		*/
		
		
		
		
//		 edu.ksu.cis.bnj.ver3.inference.exact.distributed.LS ls = new  edu.ksu.cis.bnj.ver3.inference.exact.distributed.LS();
//			opt.outputln(opt.renderInferenceResult(bn, ls2));
		 
//		edu.ksu.cis.bnj.ver3.inference.exact.mtls.LS ls = new  edu.ksu.cis.bnj.ver3.inference.exact.mtls.LS();
//		ls.run(bn);
//		edu.ksu.cis.bnj.ver3.inference.exact.LS ls2 = new  edu.ksu.cis.bnj.ver3.inference.exact.LS();
//		ls2.run(bn);
			
			
//		ls.waitUntilRecieved();
		//for(int i = 0; i < 2000; i++)
		{
		}
//		opt.outputln(opt.renderInferenceResult(bn, ls2));
//		opt.outputln(opt.renderInferenceResult(bn, ls));
		
//		edu.ksu.cis.bnj.ver3.inference.exact.Elimbel elimbel = new edu.ksu.cis.bnj.ver3.inference.exact.Elimbel();
//		int trials = opt.getInteger("trials", 1);
//		if (trials == 1)
//		{
//			trials = opt.getInteger("t", 1);
//		}
//		opt.outputln("running " + trials + " trials");
//		IsConnected IC = new IsConnected();
//		IC.setGraph(bn.getGraph());
		
//		DetectCycles DC = new DetectCycles();
//		DC.setGraph(bn.getGraph());
		//for (int i = 0; i < trials; i++)
//		{
//			IC.execute();
//			TS.execute();
//			DC.execute();
//		}
//		opt.outputln((IC.connected ? "this graph is connected " : "it is not connected"));
		/*

		int[] order = TS.alpha;
		for(int i = 0; i < order.length; i++ )
		{
			System.out.print(" " + order[i]);
		}
		System.out.println();
		System.out.println();
		BeliefNode V[] = bn.getNodes();
		for(int i = 0; i < V.length; i++)
		{
			BeliefNode C[] = bn.getChildren(V[i]);
			for(int j = 0; j < C.length; j++)
			{
				System.out.println("" + V[i].loc() + " , " + C[j].loc());
			}
		}
		*/
//		opt.outputln((DC.hasCycles ? "this graph has cycles" : "it does not have cycles"));
//		opt.outputln(opt.renderInferenceResult(bn, elimbel));
	}
	
	public static void exec(Options opt)
	{
		
		GlobalOptions GO = GlobalOptions.getInstance();
//		GO.save("test.ini");
		String f;
		opt.begin();
		
		try
		{
		
		    BeliefNetwork bn = Options.load("switch.xml");
		    if(DynamicTyping.IsDynamic(bn,true))
		    {
		        bn = Filtering.BuildFilterNetwork(bn);
		    }
//		    Filtering.Filter(bn);

		    IOPlugInLoader pil = IOPlugInLoader.getInstance();
			Exporter EXP = pil.GetExportersByExt(pil.GetExt("Dbn-Filter.xml"));
			EXP.save(new FileOutputStream("Dbn-Filter.xml"));
			OmniFormatV1_Writer.Write( bn, EXP.getStream1());
		    
//			Converter_xmlevidence evidenceLoad = new Converter_xmlevidence();
	//		Evidence_Reader ER = new Evidence_Reader();
		//	evidenceLoad.load(new FileInputStream("asia-evidence.xml"), ER);
		}
		catch (Exception e)
		{
			
		}
		
//		Converter_xmlevidence evidenceLoad = new xmlEvidence();
		//Evidence_Reader ER = new Evidence_Reader();
		//ER.
//		Evidence_Writer EW = new Evidence_Writer();
//		evidenceLoad.BeginEvidence();
	//	evidenceLoad.
		
		
		/*
		opt.BeginPerfMeasure();

		
		ImportARFF IARFF = new ImportARFF();
		for(int i = 0; i < 100; i++)
		{
			IARFF.load("asia1000data.arff",true); // the true turns on compaction which turns off the samples TreeMaps
		}
		*/
		
/*
		EvidenceList data = IARFF.getData(); // extract the loaded data
		EvidenceStructure struct = IARFF.getStructure(); // get the structure (attributes)
		BeliefNode[] nodes = struct.getNodes(); // get the nodes
		for(int i = 0; i < data.size(); i++)
		{
			int[] evidence = data.getSample(i).getDiscreteCache();	// get the evidence array
			for(int k = 0; k < nodes.length; k++)
			{
				// do something, lets print it out
				//System.out.print( " " + nodes[k].getName() + " := " + nodes[k].getDomain().getName(evidence[k]) + " " );
			}
			System.out.println();
		}
	*/	
//		opt.EndPerfMeasure();
//		opt.outputPerformanceReport("compact = true");
		
		/*
		for(Iterator it = data.getSample(i).getKeyIterator(); it.hasNext(); )
		{
			String key = data.getSample(i).getKey(it);
			String value = data.getSample(i).getValue(key);
			String res = " " + key + " := " + value + " ";
			//System.out.print(" " + key + " := " + value + " ");
		}			
		for(int i = 0; i < data.size(); i++)
		{
			for(Iterator it = data.getSample(i).getKeyIterator(); it.hasNext(); )
			{
				String key = data.getSample(i).getKey(it);
				String value = data.getSample(i).getValue(key);
				System.out.print(" " + key + " := " + value + " ");
			}
		}
		*/
		while ((f = opt.file()) != null)
		{
			BeliefNetwork bn = Options.load(f);

			//onbn(opt, bn);
			/*
			//LanguageUnicodeParser OP = LanguageUnicodeParser.getInstance();
			if (bn != null)
			{
				opt.BeginPerfMeasure();
				onbn(opt, bn);
				opt.EndPerfMeasure();
				opt.outputPerformanceReport(bn.getName());
			}
			*/
		}
		IOPlugInLoader IOPL = IOPlugInLoader.getInstance();
	}
}
