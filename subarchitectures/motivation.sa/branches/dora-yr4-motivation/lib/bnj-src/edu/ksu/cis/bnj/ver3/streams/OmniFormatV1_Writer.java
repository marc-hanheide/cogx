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
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Domain;
/**
 * file: OmniFormatV1_Reader.java
 * 
 * @author Jeffrey M. Barber
 */
public class OmniFormatV1_Writer
{
	/**
	 * @param bn
	 * @param out
	 */
	public static void Write(BeliefNetwork bn, OmniFormatV1 out)
	{
		BeliefNode[] nodes = bn.getNodes();
		out.Start();
		out.CreateBeliefNetwork(0);
		out.SetBeliefNetworkName(0, bn.getName());
		for (int i = 0; i < nodes.length; i++)
		{
			//System.out.println("writing out:" + nodes[i].getName());
			out.BeginBeliefNode(nodes[i].loc());
			if(nodes[i].getType() == BeliefNode.NODE_CHANCE) out.SetType("chance");
			if(nodes[i].getType() == BeliefNode.NODE_DECISION) out.SetType("decision");
			if(nodes[i].getType() == BeliefNode.NODE_UTILITY) out.SetType("utility");
			out.SetBeliefNodeName(nodes[i].getName());
			Domain D = nodes[i].getDomain();
			for (int j = 0; j < D.getOrder(); j++)
			{
				out.BeliefNodeOutcome(D.getName(j));
			}
			out.SetBeliefNodePosition(nodes[i].getOwner().getx(), nodes[i].getOwner().gety());
			out.EndBeliefNode();
		}
		for (int i = 0; i < nodes.length; i++)
		{
			int j;
			BeliefNode[] children = bn.getChildren(nodes[i]);
			for (j = 0; j < children.length; j++)
			{
				//System.out.println("attempt connect:" + nodes[i].getName() + " to " + children[j].getName());
				out.Connect(i,children[j].loc());
			}
			//.out.println(" name: " + nodes[i].getName() + ":>" + nodes[i].getCPF().size());
		}
		for (int i = 0; i < nodes.length; i++)
		{
			int j;
			out.BeginCPF(nodes[i].loc());
			int max = nodes[i].getCPF().size();
			int tab = nodes[i].getCPF().getDomainProduct()[0].getDomain().getOrder();
			for (j = 0; j < max / tab; j++)
			{
				for (int k = 0; k < tab; k++)
				{
					out.ForwardFlat_CPFWriteValue(nodes[i].getCPF().get(k * max / tab + j).getExpr());
				}
			}
			out.EndCPF();
		}
		out.Finish();
	}
}