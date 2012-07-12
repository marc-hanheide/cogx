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
 */package edu.ksu.cis.bnj.ver3.core.lazy;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.Field;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;
/*!
 * \file Projection.java
 * \author Jeffrey M. Barber
 */
public class Projection extends CPF
{
	private CPF				_CPF;
	private BeliefNode[]	_Set;
	private int[]			_Projection;
	private int[]			_CacheAddress;
	private int[]			_SubSizeBuffer;
	/*! query the projection
	 * \see edu.ksu.cis.bnj.ver3.core.CPF#get(int[])
	 */
	public Value get(int[] query)
	{
		CPF.applyProjectionMapping(query, _Projection, _CacheAddress);
		return getZ(_CacheAddress);
	}
	/*! query the sum of the projection
	 * \see edu.ksu.cis.bnj.ver3.core.CPF#get(int[])
	 */
	public Value getZ(int[] query)
	{
		for (int k = 0; k < query.length; k++)
		{
			if (query[k] == -1)
			{
				Value s = ValueZero.SingletonZero;
				for (int j = 0; j < _SubSizeBuffer[k]; j++)
				{
					query[k] = j;
					Value p = getZ(query);
					s = Field.add(s, p);
				}
				query[k] = -1;
				return s;
			}
		}
		return _CPF.get(query);
	}
	/*! Projectiion from an CPF (original) to a new set
	 * \param[in] original CPF
	 * \param[in] set the new domain product
	 */
	public Projection(CPF original, BeliefNode[] set)
	{
		_DomainProduct = set;
		_SizeBuffer = new int[_DomainProduct.length];
		for (int i = 0; i < _SizeBuffer.length; i++)
		{
			_SizeBuffer[i] = _DomainProduct[i].getDomain().getOrder();
		}
		_SubSizeBuffer = new int[original.getDomainProduct().length];
		for (int i = 0; i < original.getDomainProduct().length; i++)
		{
			_SubSizeBuffer[i] = original.getDomainProduct()[i].getDomain().getOrder();
		}
		_CPF = original;
		_Set = set;
		_Projection = CPF.getProjectionMapping(set, original.getDomainProduct());
		_CacheAddress = original.realaddr2addr(0);
	}
	/*! this class is a sub class
	 * \see edu.ksu.cis.bnj.ver3.core.CPF::isSubClass()
	 */
	public boolean isSubClass()
	{
		return true;
	}
}