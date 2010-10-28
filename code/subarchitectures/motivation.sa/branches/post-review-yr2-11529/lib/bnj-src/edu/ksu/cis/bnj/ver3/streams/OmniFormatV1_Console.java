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
/*!
 * \file OmniFormatV1_Console.java
 * \author Jeffrey M. Barber
 */
public class OmniFormatV1_Console implements OmniFormatV1
{
	/**
	 * @param idx
	 */
	public void CreateBeliefNetwork(int idx)
	{
		System.out.println("cbn "+idx);
	}
	/**
	 * @param idx
	 * @param name
	 */
	public void SetBeliefNetworkName(int idx, String name)
	{
		System.out.println("nam " + idx + " " + name);
	}
	/**
	 * @param idx
	 */
	public void BeginBeliefNode(int idx)
	{
		System.out.println("bbn " + idx);
	}
	/**
	 * @param type
	 */
	public void SetType(String type)
	{
		System.out.println("typ " + type);
	}
	/**
	 * @param var
	 */
	public void MakeContinuous(String var)
	{
		System.out.println("cts " + var);
	}
	/**
	 * @param outcome
	 */
	public void BeliefNodeOutcome(String outcome)
	{
		System.out.println("bno " + outcome);
	}
	/**
	 * @param name
	 */
	public void SetBeliefNodePosition(int x, int y)
	{
		System.out.println("pos " + x + " " + y);
	}
	/**
	 * @param name
	 */
	public void SetBeliefNodeName(String name)
	{
		System.out.println("sbn " + name);
	}
	/**
	 * 
	 */
	public void EndBeliefNode()
	{
		System.out.println("ebn");
	}
	/**
	 * @param par_idx
	 * @param chi_idx
	 */
	public void Connect(int par_idx, int chi_idx)
	{
		System.out.println("con " + par_idx + " " + chi_idx);
	}
	/**
	 * @param idx
	 */
	public void BeginCPF(int idx)
	{
		System.out.println("cpf " + idx);
	}
	/**
	 * @param x
	 */
	public void ForwardFlat_CPFWriteValue(String x)
	{
		System.out.println("flat " + x);
	}
	/**
	 * 
	 */
	public void EndCPF()
	{
		System.out.println("com");
	}
	/**
	 * 
	 */
	public void Start()
	{
		System.out.println("beg");
	}
	/**
	 * 
	 */
	public void Finish()
	{
		System.out.println("fin");
	}
}
