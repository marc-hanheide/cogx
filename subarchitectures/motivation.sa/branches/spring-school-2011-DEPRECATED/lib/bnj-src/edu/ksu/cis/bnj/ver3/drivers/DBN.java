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

import java.io.FileOutputStream;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.dynamic.UnRoll;
import edu.ksu.cis.bnj.ver3.plugin.IOPlugInLoader;
import edu.ksu.cis.bnj.ver3.streams.Exporter;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Writer;
import edu.ksu.cis.util.GlobalOptions;
import edu.ksu.cis.util.driver.Options;
/*!
 * \file DBN.java
 * \author Jeffrey M. Barber
 */
public class DBN
{
	public static void onbn(Options opt, BeliefNetwork bn, String file, int maxtime)
	{
		BeliefNetwork res = UnRoll.execute(bn, maxtime);
		
		try
		{
			IOPlugInLoader pil = IOPlugInLoader.getInstance();
			Exporter EXP = pil.GetExportersByExt(pil.GetExt(file));
			EXP.save(new FileOutputStream(file));
			OmniFormatV1_Writer.Write( res , EXP.getStream1());
			
		} catch (Exception e)
		{
			System.out.println("could not write file: " + file);
		}
	}
	
	public static void exec(Options opt)
	{
		GlobalOptions GO = GlobalOptions.getInstance();
		String f;
		opt.begin();
		
		while ((f = opt.file()) != null)
		{
			BeliefNetwork bn = Options.load(f);
			String outFile = opt.get("out");
			onbn(opt, bn, outFile, opt.getInteger("maxtime", 10));
		}
	}
	
}
