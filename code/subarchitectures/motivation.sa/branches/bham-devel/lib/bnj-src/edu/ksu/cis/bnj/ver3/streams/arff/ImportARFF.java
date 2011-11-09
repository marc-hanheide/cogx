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
 */package edu.ksu.cis.bnj.ver3.streams.arff;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.Domain;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceList;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceSample;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceStructure;

/*!
 * \file ImportARFF.java
 * \author Jeffrey M. Barber
 */
public class ImportARFF
{
	private EvidenceList _Data;
	private EvidenceStructure _Structure;
	private String	_Name;
	private boolean _DataMode;
	private int	_Width;
	private ArrayList _Names;
	private ArrayList _Domains;
	private int _Time;
	private boolean _Compact;
	private BeliefNode[] _Nodes;
	public EvidenceList getData()
	{
		return _Data;
	}
	public EvidenceStructure getStructure()
	{
		return _Structure;
	}
	
	public void onLine(String line)
	{
		if(line.length() < 1) return;
		if(!_DataMode)
		{
			int lLen = line.length();
			if(lLen >= 9)
			{
				if(line.substring(0,10).toUpperCase().equals("@ATTRIBUTE"))
				{
					String data = line.substring(10).trim();
					int k = data.indexOf("{");
					String domainName = data.substring(0,k).trim();
					data = data.substring(k+1);
					k = data.indexOf("}");
					data = data.substring(0,k).trim();
					Discrete D = new Discrete();
					k = data.indexOf(",");
					while(k>0)
					{
						String dName = data.substring(0,k).trim();
						D.addName(dName);
						data = data.substring(k+1).trim();
						k = data.indexOf(",");
					}
					D.addName(data);
					_Names.add(domainName);
					_Domains.add(D);
				}
				else if(line.substring(0,9).toUpperCase().equals("@RELATION"))
				{
					_Name = line.substring(9).trim();
				}
			}
			else if(lLen >= 5)
			{
				if(line.substring(0,5).toUpperCase().equals("@DATA"))
				{
					// compile the information MAN!
					_Width = _Names.size();
					String[] Names = new String[_Width];
					Domain[] Domains = new Domain[_Width];
					for(int i = 0; i < _Width; i++)
					{
						Names[i] = (String) _Names.get(i);
						Domains[i] = (Domain) _Domains.get(i);
					}
					_Structure = new EvidenceStructure(Names,Domains);
					_Nodes = _Structure.getNodes();
					_DataMode = true;
				}
			}
		}
		else
		{
			
			if(_DataMode)
			{
				int[] Cache = new int[_Width];
				EvidenceSample ES = new EvidenceSample(_Time);
				String data = line;
				int k = data.indexOf(",");
				int idx = 0;
				String val;
				while(k>0)
				{
					val = data.substring(0,k).trim();
					if(!_Compact) ES.map((String) _Names.get(idx) , val);
					Cache[idx] = ((Discrete)_Nodes[idx].getDomain()).findName(val);
					data = data.substring(k+1);
					k = data.indexOf(",");
					idx++;
				}
				val = data.trim();
				if(!_Compact) ES.map((String) _Names.get(idx) , val);
				Cache[idx] = ((Discrete)_Nodes[idx].getDomain()).findName(val);
				ES.writeDiscreteCache(Cache);
				_Time++;
				_Data.add(ES);
			}
	
		}
	}
	
	
	
	public void load(String name, boolean compact)
	{
		_Compact = compact;
		_Time = 0;
		_DataMode = false;
		_Names = new ArrayList();
		_Domains = new ArrayList();
		_Data = new EvidenceList();
		try
		{
			FileReader F = new FileReader(name);
			BufferedReader BR = new BufferedReader(F);
			while(BR.ready())
			{
				onLine(BR.readLine());
			}
			F.close();
		} catch (Exception e)
		{
			System.out.println("Error?" + e.getMessage());
		}
	}
}
