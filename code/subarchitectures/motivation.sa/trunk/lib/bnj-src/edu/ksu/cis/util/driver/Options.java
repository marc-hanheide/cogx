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
 */package edu.ksu.cis.util.driver;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.lazy.CacheManager;
import edu.ksu.cis.bnj.ver3.inference.Inference;
import edu.ksu.cis.bnj.ver3.plugin.IOPlugInLoader;
import edu.ksu.cis.bnj.ver3.streams.Importer;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Reader;
/**
 * file: Options.java
 * 
 * @author Jeffrey M. Barber
 */
public class Options
{
	private ArrayList _files;
	public HashMap	_map;
	public boolean _RENDERHTML;
	public static boolean  _RATIONAL = false;
	public static boolean  _FLOAT = false;
	
	public Iterator it;
	public Options()
	{
		CacheManager.getInstance();
		_map = new HashMap();
		_files = new ArrayList();
		_RENDERHTML = false;
	}
	
	public Options(String[] args)
	{
		CacheManager.getInstance();
		_map = new HashMap();
		_files = new ArrayList();
		_RENDERHTML = false;
		parseArgs(args);
	}
	
	public static void output(String x)
	{
		System.out.print(x);
	}
	public static void outputln(String x)
	{
		System.out.println(x);
	}
	
	public String renderCPF(CPF cpf)
	{
		String res = _RENDERHTML ? Options.getHTML(cpf) : Options.getString(cpf);
		return res;
	}
	
	public void begin()
	{
		it = _files.iterator();
	}
	
	public String file()
	{
		if(it.hasNext())
		{
			return (String) it.next();
		}
		else
		{
			return null;
		}
	}
	
	public void parseArgs(String[] args)
	{
		for (int i = 1; i < args.length; i++)
		{
			if (args[i].toUpperCase().equals("-HTML"))
			{
				_RENDERHTML = true;
			}
			else if (args[i].toUpperCase().equals("-R"))
			{
				Options._FLOAT = false;
				Options._RATIONAL = true;
			}
			else if (args[i].toUpperCase().equals("-F"))
			{
				Options._FLOAT = true;
				Options._RATIONAL = false;
			}
			else if (args[i].toUpperCase().equals("-D"))
			{
				Options._FLOAT = false;
				Options._RATIONAL = false;
			}
			else if (args[i].substring(0, 1).equals("-"))
			{
				_map.put(args[i].substring(1), args[i + 1]);
				i++;
			}
			else
			{
				_files.add(args[i]);
			}
		}
	}
	public String get(String name)
	{
		try
		{
			return (String) _map.get(name);
		}
		catch (Exception e)
		{
			return "";
		}
	}
	public boolean getBoolean(String name)
	{
		String v = get(name);
		v.toUpperCase();
		return (v.equals("T") || v.equals("TR") || v.equals("TRU") || v.equals("TRUE"));
	}
	public int getInteger(String name, int def)
	{
		String v = get(name);
		try
		{
			return Integer.parseInt(v);
		}
		catch (Exception e)
		{
			return def;
		}
	}
	public double getDouble(String name)
	{
		String v = get(name);
		try
		{
			return Double.parseDouble(v);
		}
		catch (Exception e)
		{
			return 0;
		}
	}
	
	private long OriginalMemory;
	private long StartTime;
	
	
	public long TemporaryMemoryUsed;
	public long MemoryNeeded;
	public long ProcessingTime;
	
	public void BeginPerfMeasure()
	{
		Runtime r = Runtime.getRuntime();
		r.gc();
		OriginalMemory = r.totalMemory() - r.freeMemory();
		StartTime = System.currentTimeMillis();
	}
	
	public void EndPerfMeasure()
	{
		Runtime r = Runtime.getRuntime();
		long nextmem = r.totalMemory() - r.freeMemory();
		long usedmem = nextmem - OriginalMemory;
		r.gc();
		nextmem = r.totalMemory() - r.freeMemory();
		long finalmem = nextmem - OriginalMemory;
		TemporaryMemoryUsed = usedmem - finalmem;
		MemoryNeeded = finalmem;
		ProcessingTime = System.currentTimeMillis() - StartTime;
	}
	
	public void outputPerformanceReport(String name)
	{
		outputln("---- perf report for " + name);
		outputln(" temp mem = " + TemporaryMemoryUsed);
		outputln("      mem = " + MemoryNeeded);
		outputln("     time = " + ProcessingTime);
		outputln("--------------------------------------");
	}
	
	public String renderInferenceResult(BeliefNetwork bn, Inference Inf)
	{
		String result = "";
		BeliefNode[] nodes = bn.getNodes();
		if(_RENDERHTML)
		{
			result+="<table>";
		}
		for(int i = 0; i < nodes.length; i++)
		{
			BeliefNode X = nodes[i];
			if(_RENDERHTML)
			{
				result+="<tr>";
			}
			else
			{
				result+= X.getName() + ":\n";
			}
			result+=renderCPF(Inf.queryMarginal(X));
			if(_RENDERHTML)
			{
				result+="</tr>";
			}
			else
			{
				result+=  "\n";
			}
		}
		if(_RENDERHTML)
		{
			result+="</table>";
		}
		return result;
	}
	public static BeliefNetwork load(FileInputStream FIS, String file)
	{
		IOPlugInLoader pil = IOPlugInLoader.getInstance();
		Importer IMP = pil.GetImporterByExt(pil.GetExt(file));
		OmniFormatV1_Reader ofv1r = new OmniFormatV1_Reader();
		IMP.load(FIS, ofv1r);
		BeliefNetwork bn = ofv1r.GetBeliefNetwork(0);
		if(_RATIONAL)
		{
			BeliefNode[] n = bn.getNodes();
			for(int i = 0; i < n.length; i++)
				n[i].getCPF().convertDouble2Rational();
		}
		if(_FLOAT)
		{
			BeliefNode[] n = bn.getNodes();
			for(int i = 0; i < n.length; i++)
				n[i].getCPF().convertDouble2Float();
		}
		
		return bn;
	}
	public static BeliefNetwork load(String file)
	{
		try
		{
			IOPlugInLoader pil = IOPlugInLoader.getInstance();
			Importer IMP = pil.GetImporterByExt(pil.GetExt(file));
			FileInputStream FIS = new FileInputStream(file);
			OmniFormatV1_Reader ofv1r = new OmniFormatV1_Reader();
			IMP.load(FIS, ofv1r);
			BeliefNetwork bn = ofv1r.GetBeliefNetwork(0);
			if(_RATIONAL)
			{
				BeliefNode[] n = bn.getNodes();
				for(int i = 0; i < n.length; i++)
					n[i].getCPF().convertDouble2Rational();
			}
			if(_FLOAT)
			{
				BeliefNode[] n = bn.getNodes();
				for(int i = 0; i < n.length; i++)
					n[i].getCPF().convertDouble2Float();
			}			
			return bn;
		}
		catch (Exception e)
		{
			outputln("file: `" + file + "` was not found");
			return null;
		}
	}	
	/*! Convert a cpf into s string for printing
	 * \@param[in] cpf the CPF
	 * @return the String
	 */
	public static String getString(CPF cpf)
	{
		int[] q = cpf.realaddr2addr(0);
		BeliefNode[] bd = cpf.getDomainProduct();
		String s = "";
		boolean done = false;
		while(!done)
		{
			s += "\t{";
			for (int j = 0; j < q.length; j++)
			{
				if (j != 0) s += ",";
				s += bd[j].getName() + "=" + bd[j].getDomain().getName(q[j]);
			}
			s += "} := " + cpf.get(q).getExpr() + "\n";
			done = cpf.addOne(q);
		}
		/*
		int n = cpf.size();
		for (int i = 0; i < n; i++)
		{
			int[] l = cpf.realaddr2addr(i);
			s += "\t{";
			for (int j = 0; j < l.length; j++)
			{
				if (j != 0) s += ",";
				s += bd[j].getName() + "=" + bd[j].getDomain().getName(l[j]);
			}
			s += "} := " + cpf.get(l).getExpr() + "\n";
		}
		*/
		return s;
	}
	/*! Convert a cpf into s HTML encoded string for printing
	 * \@param[in] cpf the CPF
	 * @return the HTML encoded String
	 */
	public static String getHTML(CPF cpf)
	{
		String res = "";
		BeliefNode[] bdp = cpf.getDomainProduct();
		int bs = 1;
		int sk = bdp[0].getDomain().getOrder();
		int dv = cpf.size() / sk;
		res = "<table border=1>";
		res += "<tr><td colspan=" + (dv * sk + 1) + "><b>" + bdp[0].getName() + "</b></td></tr>";
		int k;
		for (k = 1; k < bdp.length; k++)
		{
			res += "<tr><td></td>";
			int dz = (dv / bs) / bdp[k].getDomain().getOrder();
			for (int i = 0; i < bs; i++)
			{
				for (int j = 0; j < bdp[k].getDomain().getOrder(); j++)
				{
					res += "<td colspan=" + dz + ">" + bdp[k].getDomain().getName(j) + "</td>";
				}
			}
			bs *= bdp[k].getDomain().getOrder();
			res += "</tr>";
		}
		for (k = 0; k < sk; k++)
		{
			res += "<tr><td>" + bdp[0].getDomain().getName(k) + "</td>";
			int p = k * dv;
			for (int j = 0; j < dv; j++)
			{
				res += "<td>" + cpf.get(p + j).getExpr() + "</td>";
			}
			res += "</tr>";
		}
		res += "</table>";
		return res;
	}	
}