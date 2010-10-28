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
import java.util.HashMap;
import edu.ksu.cis.bnj.ver3.core.*;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
/**
 * file: OmniFormatV1_Writer.java
 * 
 * @author Jeffrey M. Barber
 */
public class OmniFormatV1_Reader implements OmniFormatV1
{
	private HashMap		beliefmap;
	private HashMap		nodemap;
	private int			currentBeliefGraph;
	private int			currentBeliefNode;
	private String		curBeliefNodeName;
	private Domain		curBeliefNodeDomain;
	private int			curType;
	private int			currentCPF		= 0;
	private int			curBeliefNodeX	= 0;
	private int			curBeliefNodeY	= 0;
	/**
	 * @param idx
	 * @return
	 */
	public BeliefNetwork GetBeliefNetwork(int idx)
	{
		return (BeliefNetwork) beliefmap.get(new Integer(idx));
	}
	/*
	 * (non-Javadoc)
	 * 
	 * @see edu.ksu.cis.bnj.ver3.streams.OmniFormatV1#Start()
	 */
	public void Start()
	{
		beliefmap = new HashMap();
	}
	/*
	 * (non-Javadoc)
	 * 
	 * @see edu.ksu.cis.bnj.ver3.streams.OmniFormatV1#Finish()
	 */
	public void Finish()
	{
		//... do nothing
		nodemap = null;
		curBeliefNodeDomain = null;
		curBeliefNodeName = null;
		currentBeliefNode = 0;
		currentBeliefGraph = 0;
	}
	public void CreateBeliefNetwork(int idx)
	{
		//System.out.println("CreateBeliefNetwork(" + idx + ");");
		beliefmap.put(new Integer(idx), new BeliefNetwork());
		nodemap = new HashMap();
		currentBeliefGraph = idx;
	}
	public void SetBeliefNetworkName(int idx, String name)
	{
		//System.out.println("SetBeliefNetworkName(" + idx + ",\"" + name +
		// "\");");
		BeliefNetwork bn = (BeliefNetwork) beliefmap.get(new Integer(idx));
		bn.setName(name);
	}
	public void BeginBeliefNode(int idx)
	{
		//System.out.println("BeginBeliefNode(" + idx + ");");
		currentBeliefNode = idx;
		curBeliefNodeDomain = new Discrete();
		curType = BeliefNode.NODE_CHANCE;
	}
	public void MakeContinuous(String var)
	{
		curBeliefNodeDomain = new Continuous(var);
	}
	public void SetType(String type)
	{
		curType = BeliefNode.NODE_CHANCE;
		if(type.toLowerCase().equals("decision"))
			curType = BeliefNode.NODE_DECISION;
		if(type.toLowerCase().equals("utility"))
			curType = BeliefNode.NODE_UTILITY;
	}
	public void SetBeliefNodePosition(int x, int y)
	{
		curBeliefNodeX = x;
		curBeliefNodeY = y;
	}
	public void BeliefNodeOutcome(String outcome)
	{
		//System.out.println("BeliefNodeOutcome(\"" + outcome + "\");");
		((Discrete)curBeliefNodeDomain).addName(outcome);
	}
	public void SetBeliefNodeName(String name)
	{
		//System.out.println("SetBeliefNodeName(\"" + name + "\");");
		curBeliefNodeName = name;
	}
	public void EndBeliefNode()
	{
		//System.out.println("EndBeliefNode();");
		if(curType == BeliefNode.NODE_UTILITY)
			curBeliefNodeDomain = new Discrete(new String[] {"v"} );
		if(curType == BeliefNode.NODE_DECISION && curBeliefNodeDomain.getOrder() == 0)
			curBeliefNodeDomain = new Discrete(new String[] {"true","false"} );
		BeliefNode bnode = new BeliefNode(curBeliefNodeName, curBeliefNodeDomain);
		bnode.setType(curType);
		nodemap.put(new Integer(currentBeliefNode), bnode);
		BeliefNetwork bn = (BeliefNetwork) beliefmap.get(new Integer(currentBeliefGraph));
		bn.addBeliefNode(bnode);
		bnode.setPosition(curBeliefNodeX, curBeliefNodeY);
	}
	public void Connect(int par_idx, int chi_idx)
	{
		//System.out.println("Connect(" + par_idx + "," + chi_idx + ");");
		BeliefNetwork bn = (BeliefNetwork) beliefmap.get(new Integer(currentBeliefGraph));
		BeliefNode Parent = (BeliefNode) nodemap.get(new Integer(par_idx));
		BeliefNode Child = (BeliefNode) nodemap.get(new Integer(chi_idx));
		
		//System.out.println("connecting: " + Parent.getName() + " to " + Child.getName());
		//System.out.println("e: " + Parent.getName() + " :> " +
		// Child.getName());
		bn.connect(Parent, Child);
		
		//System.out.println("name growth: " + Parent.getName() + " :> + " + Parent.getCPF().size());
	}
	public void BeginCPF(int idx)
	{
		//System.out.println("SelectBeliefNodeForCPF(" + idx + ");");
		currentBeliefNode = idx;
		currentCPF = 0;
		BeliefNode bnode = (BeliefNode) nodemap.get(new Integer(currentBeliefNode));
	}
	public void ForwardFlat_CPFWriteValue(String x)
	{
		//System.out.println("ForwardFlat_CPFWriteValue(" + x + ");");
		BeliefNode bnode = (BeliefNode) nodemap.get(new Integer(currentBeliefNode));
		CPF cpf = bnode.getCPF();
		ValueDouble V = new ValueDouble(Double.parseDouble(x));
		//cpf.put(currentCPF, V);
		int z = currentCPF % bnode.getDomain().getOrder();
		int z2 = currentCPF / bnode.getDomain().getOrder();
		int z3 = cpf.size() / bnode.getDomain().getOrder();
		cpf.put(z * z3 + z2, V);
		
		currentCPF++;
	}
	public int GetCPFSize()
	{
		//System.out.println("GetCPFSize();");
		BeliefNode bnode = (BeliefNode) nodemap.get(new Integer(currentBeliefNode));
		CPF cpf = bnode.getCPF();
		return cpf.size();
	}
	public void EndCPF()
	{}
}