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
 */package edu.ksu.cis.bnj.ver3.core.evidence;
import java.util.Iterator;
import java.util.TreeMap;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Discrete;
/*!
 * \file EvidenceSample.java
 * \author Jeffrey M. Barber
 */
public class EvidenceSample
{
	private TreeMap	_Evidence;
	private int		_Time;
	private int		_Loc;
	private int[]	DiscreteCache;
	/*! create a new sample at this time
	 * \param[in] time
	 */
	public EvidenceSample(int time)
	{
		_Evidence = new TreeMap();
		_Time = time;
		_Loc = -1;
	}
	/*! map key to value
	 * \param[in] key
	 * \param[in] value
	 */
	public void map(String key, String value)
	{
		_Evidence.put(key, value);
	}
	/*! get the key iterator
	 * \return an iterator for the keys
	 */
	public Iterator getKeyIterator()
	{
		return _Evidence.keySet().iterator();
	}
	/*! extract the key
	 * \param[in] it the iterator
	 * \return the key
	 */
	public String getKey(Iterator it)
	{
		return (String) it.next();
	}
	/*! get the value associated to the key: key
	 * \param[in] key the key to value
	 * \return the value associated to key
	 */
	public String getValue(String key)
	{
		return (String) _Evidence.get(key);
	}
	/*! given an array of belief nodes with discrete domains, build a discrete cache (subject to change)
	 * \param[in] Nodes array of belief nodes
	 */
	public void createDiscreteCache(BeliefNode[] Nodes)
	{
		DiscreteCache = new int[Nodes.length];
		for (int i = 0; i < Nodes.length; i++)
		{
			DiscreteCache[i] = ((Discrete) Nodes[i].getDomain()).findName(getValue(Nodes[i].getName()));
		}
	}
	/*! get the discrete cache (subject to change)
	 * \return
	 */
	public int[] getDiscreteCache()
	{
		return DiscreteCache;
	}
	/*! write a discrete cache out
	 * \param[in] cache
	 */
	public void writeDiscreteCache(int[] cache)
	{
		DiscreteCache = cache;
	}
	/*! query the discrete cache
	 * \param[in] K
	 * \return
	 */
	public int getDiscreteValue(int K)
	{
		return DiscreteCache[K];
	}
	/*! set the location of this sample
	 * \param[in] k the location
	 */
	public void setLOC(int k)
	{
		_Loc = k;
	}
	/*! query the location
	 * \return
	 */
	public int loc()
	{
		return _Loc;
	}
}