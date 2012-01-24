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
 */package edu.ksu.cis.bnj.ver3.streams;

import java.util.Iterator;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceList;
import edu.ksu.cis.bnj.ver3.core.evidence.EvidenceSample;

/*!
 * \file Evidence_Writer.java
 * \author Jeffrey M. Barber
 */
public class Evidence_Writer
{
	public void BeginCollectSamples(EvidenceStream out)
	{
		out.BeginEvidence();
	}
	
	public void WriteSample(BeliefNetwork bn, EvidenceStream out, int time)
	{
		out.BeginSample(time);
		BeliefNode[] nodes = bn.getNodes();
		for(int i = 0; i < nodes.length; i++)
		{
			if(nodes[i].hasEvidence())
			{
				out.Witness(nodes[i].getName(), nodes[i].getEvidence().getName(nodes[i].getDomain()));
			}
		}
		out.EndSample();
	}
	
	public void EndCollectSamples(EvidenceStream out)
	{
		out.EndEvidence();
	}
	
	public void WriteEvidence(BeliefNetwork bn, EvidenceStream out)
	{
		out.Start();
		out.BeginEvidence();
		out.BeginSample(0);
		BeliefNode[] nodes = bn.getNodes();
		for(int i = 0; i < nodes.length; i++)
		{
			if(nodes[i].hasEvidence())
			{
				out.Witness(nodes[i].getName(), nodes[i].getEvidence().getName(nodes[i].getDomain()));
			}
		}
		out.EndSample();
		out.EndEvidence();
		out.End();
	}
	
	public void WriteEvidenceList(EvidenceList el, EvidenceStream out)
	{
		out.Start();
		out.BeginEvidence();
		for(int i = 0; i < el.size(); i++)
		{
			EvidenceSample es = el.getSample(i);
			out.BeginSample(i);
			for(Iterator it = es.getKeyIterator(); it.hasNext(); )
			{
				String key = es.getKey(it);
				String value = es.getValue(key);
				out.Witness(key,value);
			}
			out.EndSample();
		}
		out.EndEvidence();
		out.End();
	}
}
