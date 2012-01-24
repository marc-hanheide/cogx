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
 */package edu.ksu.cis.bnj.ver3.dynamic;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.util.graph.core.Vertex;

/*!
 * \file UnRoll.java
 * \author Jeffrey M. Barber
 */
public class UnRoll
{
	
	public static String simpleReplaceAll(String data, String from, String to )
	{
		int len = from.length();
		String r = data;
		int k = r.indexOf(from);
		while( k >= 0)
		{
			r = data.substring(0,k) + to + data.substring(k+len);
			k = r.indexOf(from, k);
		}
		return r;
		
	}
	
	public static String Instant(String name, int time)
	{
		String name2 = (name).replaceAll(" ", "");
		name2 = simpleReplaceAll(name2, "[t-1]", "[" + (time - 1) + "]");
		name2 = simpleReplaceAll(name2, "[t]"  , "[" + (time) + "]");
		return name2;
	}
	
	public static BeliefNetwork execute(BeliefNetwork bn, int MaxTime)
	{
		// we seperate the network into an initial network and a transitional network
		// create a new network
		// add and connect initial network
		// from 1 to MaxTime inclusive
		   // add and connect a transitional network
		BeliefNetwork 	_NewNetwork = new BeliefNetwork();
		BeliefNode[]	_OriginalNodes = bn.getNodes();
		
		int[] Type = new int[_OriginalNodes.length];
		// 0 = base
		// 1 = t - 1
		// 2 = t
		
		// classify each node
		for(int i = 0; i < _OriginalNodes.length; i++)
		{
			String name = _OriginalNodes[i].getName(); 
			name = name.replaceAll(" ", "");
			if(     name.indexOf("[t-1]") >= 0)
			{	Type[i] = 1;	}
			else if(name.indexOf("[t]") >= 0)
			{	Type[i] = 2;	}
			else
			{	Type[i] = 0;	}
			System.out.println(name + ":" + Type[i]);
		}
		
		// add the base nodes
		BeliefNode[] Cache = new BeliefNode[_OriginalNodes.length];
		BeliefNode[] TransitionalModel = new BeliefNode[_OriginalNodes.length];
		int[] Translation = new int[_OriginalNodes.length];

		for(int i = 0; i < _OriginalNodes.length; i++)
		{
			if(Type[i] == 0)
			{
				BeliefNode newNode = new BeliefNode(_OriginalNodes[i].getName(), _OriginalNodes[i].getDomain());
				_NewNetwork.addBeliefNode(newNode);
				newNode.setType(_OriginalNodes[i].getType());
				
				Cache[i] = newNode;

				Vertex OV = _OriginalNodes[i].getOwner();
				Vertex V = newNode.getOwner();
				V.setAttrib(0, OV.getx());
				V.setAttrib(1, OV.gety());
			}
			Translation[i] = -1;
		}
		
		
		for(int i = 0; i < _OriginalNodes.length; i++)
		{
			String name = _OriginalNodes[i].getName();
			name = name.replaceAll(" ", "");

			if(Type[i] == 0)
			{
				BeliefNode[] parents = bn.getParents(_OriginalNodes[i]);
				for(int p = 0; p < parents.length; p++)
				{
					if( Type[parents[p].loc()] == 0)
					{
						_NewNetwork.connect(Cache[parents[p].loc()], Cache[i]);
					}
					else
					{
						System.out.println("un seperated dynamic network, bad!");
						return null;
					}
				}
				Cache[i].setCPF(_OriginalNodes[i].getCPF());
				
				for(int j = 0; j < _OriginalNodes.length; j++)
				{
					if(Type[j]==1)
					{
						String name2 = _OriginalNodes[j].getName().replaceAll(" ","");
						name2 = simpleReplaceAll(name2, "[t-1]", "[0]");
//						name2 = name2.replaceAll("t-1","0");
						if(name2.equals(name))
						{
							TransitionalModel[j] = Cache[i];
							j = _OriginalNodes.length + 1;
						}
					}
				}
			}
			if(Type[i]==2)
			{
				for(int j = 0; j < _OriginalNodes.length; j++)
				{
					if(Type[j]==1)
					{
						String name2 = _OriginalNodes[j].getName().replaceAll(" ","");
						name2 = simpleReplaceAll(name2, "[t-1]", "[t]");
//						name2 = name2.replaceAll("t-1","0");
						if(name2.equals(name))
						{
							Translation[i] = j;
							j = _OriginalNodes.length + 1;
						}
					}
				}
				// h
			}
		}
		
		for(int time = 1; time <= MaxTime; time++)
		{
			for(int j = 0; j < _OriginalNodes.length; j++)
			{
				if(Type[j]==2)
				{
					String name2 = _OriginalNodes[j].getName().replaceAll(" ","");
					//name2 = name2.replaceAll( ) ("[t]","["+time+"]");
					name2 = simpleReplaceAll(name2, "[t]", "["+time+"]");
					BeliefNode nextStep = new BeliefNode(name2,_OriginalNodes[j].getDomain());
					
					nextStep.setType(_OriginalNodes[j].getType());

					_NewNetwork.addBeliefNode(nextStep);
					Vertex OV = _OriginalNodes[j].getOwner();
					Vertex V = nextStep.getOwner();
					V.setAttrib(0, OV.getx() + 250 * time);
					V.setAttrib(1, OV.gety());
					Cache[j] = nextStep;
				}
			}
			// connect them
			for(int j = 0; j < _OriginalNodes.length; j++)
			{
				if(Type[j]==2)
				{
					BeliefNode[] parents = bn.getParents(_OriginalNodes[j]);
					for(int k = 0; k < parents.length; k++)
					{
						if(Type[parents[k].loc()]==2)
						{
							_NewNetwork.connect(Cache[parents[k].loc()], Cache[j]);
						}
						else if(Type[parents[k].loc()]==1)
						{
							// look up in the translational model
							_NewNetwork.connect(TransitionalModel[parents[k].loc()], Cache[j]);
						}
					}
					BeliefNode[] FromP = _OriginalNodes[j].getCPF().getDomainProduct();
					BeliefNode[] ToP = Cache[j].getCPF().getDomainProduct();
					/*
					for(int i = 0; i < ToP.length; i++)
					{
						String name2 = (FromP[i].getName()).replaceAll(" ", "");
						name2 = simpleReplaceAll(name2, "[t-1]", "[" + (time - 1) + "]");
						name2 = simpleReplaceAll(name2, "[t]"  , "[" + (time) + "]");
						System.out.println("" + (ToP[i].getName()) + " vs. " + name2 );
					}
					*/
					
					/*		int[] map = new int[subset.length];
		for (int k = 0; k < subset.length; k++)
		{
			map[k] = -1;
			for (int k2 = 0; k2 < original.length; k2++)
			{
				if (subset[k] == original[k2])
				{
					map[k] = k2;
					k2 = original.length;
				}
			}
		}
		return map;*/
					
					int[] map = new int[ToP.length];
					map[0] = 0;
					for (int k = 0; k < ToP.length; k++)
					{
						String name1 = Instant(ToP[k].getName(),time);
						map[k] = -1;
						for(int k2 = 0; k2 < FromP.length; k2++)
						{
							String name2 = Instant(FromP[k2].getName(),time);
							//name2 = simpleReplaceAll(name2, "[t]", "["+time+"]");
							if(name1.equals(name2))
							{
								map[k] = k2;
								k2 = FromP.length;
							}
						}
					}
					
					/*
					for(int i = 0; i < map.length; i++)
						System.out.print("" + map[i] + "-");
					System.out.println("");
					
					*/
					
//					BeliefNode[] z
					CPF O = _OriginalNodes[j].getCPF();
					CPF N = Cache[j].getCPF();
					int[] q = O.realaddr2addr(0);
					int[] q2 = N.realaddr2addr(0);
					for(int i = 0; i < O.size(); i++)
					{
						CPF.applyProjectionMapping(q, map, q2);
						N.put(q2, O.get(i));
						O.addOne(q);
					}
					
					//Cache[j].setCPF(O.expand(p));
					//Cache[j].setCPF(_OriginalNodes[j].getCPF());
				}
			}

			// update model
			for(int j = 0; j < _OriginalNodes.length; j++)
			{
				if(Type[j]==2)
				{
					if(Translation[j] >= 0)
					{
						TransitionalModel[Translation[j]] = Cache[j];
					}
				}
			}
			
			
		}
		
		return _NewNetwork;
		
		
		
		// base model is uploaded

		
		
									 
		
		//return null;
		
	}
}
