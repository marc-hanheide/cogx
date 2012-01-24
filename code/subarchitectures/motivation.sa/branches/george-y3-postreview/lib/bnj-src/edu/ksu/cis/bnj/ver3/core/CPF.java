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
 */package edu.ksu.cis.bnj.ver3.core;
import edu.ksu.cis.bnj.ver3.core.lazy.CacheManager;
import edu.ksu.cis.bnj.ver3.core.lazy.Divide;
import edu.ksu.cis.bnj.ver3.core.lazy.Projection;
import edu.ksu.cis.bnj.ver3.core.lazy.Multiply;
import edu.ksu.cis.bnj.ver3.core.sparse.Sparse;
import edu.ksu.cis.bnj.ver3.core.values.Field;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.core.values.ValueFloat;
import edu.ksu.cis.bnj.ver3.core.values.ValueRational;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;
/*!
 * \file CPF.java
 * \author Jeffrey M. Barber
 */
public class CPF
{
	public static boolean	_LazyEvalEnabled;
	protected Value[]		_Values;
	protected BeliefNode[]	_DomainProduct;
	protected int[]			_SizeBuffer;
	/*! Creates an empty CPF
	 */
	public CPF()
	{
		_Values = null;
		_DomainProduct = null;
	}
	/*! Get the domain product
	 * \return the product domain
	 */
	public BeliefNode[] getDomainProduct()
	{
		return _DomainProduct;
	}
	/*! Get the size of the table
	 * \return the number of elements in the table
	 */
	public int size()
	{
		return _Values.length;
	}
	/*! Construct an empty CPF from a prebuilt domain
	 * \param[in] productdomain The new product domain
	 * \param[in] zeroOut should the values be set to zero
	 */
	private void buildZero(BeliefNode[] productdomain, boolean zeroOut)
	{
		_DomainProduct = productdomain;
		_SizeBuffer = new int[_DomainProduct.length];
		int i;
		int s = 1;
		for (i = 0; i < productdomain.length; i++)
		{
			_SizeBuffer[i] = productdomain[i].getDomain().getOrder();
			s *= productdomain[i].getDomain().getOrder();
		}
		_Values = new Value[s];
		if(zeroOut)
		{
			Value zero = ValueZero.SingletonZero;
			for (i = 0; i < s; i++)
			{
				_Values[i] = zero;
			}
		}
	}
	/*! Construct an empty CPF from a domain
	 * \param[in] nodes		the product domain
	 */
	public CPF(BeliefNode[] nodes)
	{
		buildZero(nodes,true);
	}
	/*! Construct a null CPF from a domain
	 * \param[in] nodes		the product domain
	 * \param[in] isNull	should the nodes be inited
	 */
	public CPF(BeliefNode[] nodes, boolean isNull)
	{
		buildZero(nodes,!isNull);
	}	
	/*! Get the sum of the values for a query, q[i] = k, k >= 0 -> specific value, -1 all values, per domain
	 * \param[in] query	   the query variable
	 * \return the value or sum of values
	 */
	public Value get(int[] query)
	{
		for (int k = 0; k < query.length; k++)
		{
			if (query[k] == -1)
			{
				Value s = ValueZero.SingletonZero;
				for (int j = 0; j < _SizeBuffer[k]; j++)
				{
					query[k] = j;
					Value p = get(query);
					s = Field.add(s, p);
				}
				query[k] = -1;
				return s;
			}
		}
		return _Values[addr2realaddr(query)];	
	}
	/*! Put a value into a cell/multiple cells
	 * \todo remove recursion
	 * \param[in] query  the address/es for which to place the value
	 * \param[in] v		the value to put/replicate in places
	 */
	public void put(int[] query, Value v)
	{
		for (int k = 0; k < query.length; k++)
		{
			if (query[k] == -1)
			{
				for (int j = 0; j < _SizeBuffer[k]; j++)
				{
					query[k] = j;
					put(query, v);
				}
				query[k] = -1;
				return;
			}
		}
		_Values[addr2realaddr(query)] = v;
	}
	/*! Get an absolute/real address
	 * \param[in] realaddr the absolute address
	 * \return the value at the location
	 */
	public Value get(int realaddr)
	{
		return _Values[realaddr];
	}
	/*! Pput a value at an absolute/real address
	 * \param[in] realaddr the absolute address
	 * \param[in] v the value to put
	 */
	public void put(int realaddr, Value v)
	{
		_Values[realaddr] = v;
	}
	/*! Convert between logical to real address (absolute) [MULT-adder] [fairly expensive]
	 * \param[in] addr the logical address
	 * \return the absolute/real
	 */
	public int addr2realaddr(int[] addr)
	{
		int realAddr = 0;
		for (int i = 0; i < addr.length; i++)
		{
			realAddr *= _SizeBuffer[i]; //_DomainProduct[i].getDomain().getOrder();
			realAddr += addr[i];
		}
		return realAddr;
	}
	/*! Convert between a real address and a logical address [Div-modul] [very expensive]
	 * \param[in] realaddr the absolute/real address
	 * \return the logical address
	 */
	public int[] realaddr2addr(int realaddr)
	{
		int[] addr = new int[_DomainProduct.length];
		int run = realaddr;
		for (int i = _DomainProduct.length - 1; i >= 0; i--)
		{
			addr[i] = run % _SizeBuffer[i];
			run = (run - addr[i]) / _SizeBuffer[i];
		}
		return addr;
	}
	/*! Transform a query formed in another cpf to this one
	 * \param query      the query
	 * \param original	the original product space
	 * \return	the subquery
	 */
	public int[] getSubQuery(int[] query, BeliefNode[] original)
	{
		return CPF.getSubQuery(query, original, _DomainProduct);
	}
	/*! Transform a query formed in another cpf to this one
	 * \param[in] query      the query
	 * \param[in] original	the original product space
	 * \param[in] subset 	the subset product space
	 * \return	the subquery
	 */
	public static int[] getSubQuery(int[] query, BeliefNode[] original, BeliefNode[] subset)
	{
		int[] map = new int[subset.length];
		for (int k = 0; k < subset.length; k++)
		{
			map[k] = -1;
			for (int k2 = 0; k2 < original.length; k2++)
			{
				if (subset[k] == original[k2])
				{
					map[k] = query[k2];
					k2 = original.length;
				}
			}
		}
		return map;
	}
	/*! Faster opt for doing subqueries, factors subqueries processing from n^2 to n
	 * \param[in] original	  the original product space
	 * \param[in] subset	  the sub product space
	 * \return
	 */
	public static int[] getProjectionMapping(BeliefNode[] original, BeliefNode[] subset)
	{
		int[] map = new int[subset.length];
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
		return map;
	}
	/*! Apply a projection mapping to build a sub query
	 * \param[in] 	OriginalQuery		the original query
	 * \param[in] 	Projection		the projection
	 * \param[out] 	QueryResult		the result
	 */
	public static void applyProjectionMapping(int[] OriginalQuery, int[] Projection, int[] QueryResult)
	{
		for (int i = 0; i < Projection.length; i++)
		{
			if (Projection[i] >= 0)
			{
				QueryResult[i] = OriginalQuery[Projection[i]];
			}
			else
			{
				QueryResult[i] = -1;
			}
		}
	}
	/*! Add one to an addr isomorphic to addOne(q) =>  realaddr(addr2realaddr(q)+1), but FASTER
	 * \param[in,out] addr the address
	 */
	public boolean addOne(int addr[])
	{
		for(int k = addr.length - 1; k >= 0; k--)
		{
			addr[k]++;
			if(addr[k] >= _SizeBuffer[k])
			{
				addr[k] = 0;
				if(k==0)
				{
					return true;
				}
			}
			else
			{
				return false;
			}
		}
		return false;
	}
	
	public double Ratio()
	{
	    int n = 0;
	    for(int i = 0; i < _Values.length; i++)
	    {
	        if(_Values[i] instanceof ValueZero)
	        {
	            n++;
	        }
	        else if(_Values[i] instanceof ValueDouble)
	        {
	            double v = ((ValueDouble)_Values[i]).getValue();
	            if(Math.abs(v) < 0.00001)
	                n++;
	        }
	    }
	    double R = n / (double) _Values.length; 
	    System.out.println("On Cache, #zeros/"+_Values.length + " = " + R);
	    return R;
	}
	
	public static CPF SparseTest(CPF x)
	{
	    return x;
	}
	
	/*! Extract a cpf from the given subset
	 * \param[in] subset	 the sub product space
	 * \return the new CPF founded on the sub product space
	 */
	public CPF extract(BeliefNode[] subset)
	{
		if(this.isSubClass())
		{
			CPF next = new Projection(this, subset);
			if(next.canCache())
			{
			    return SparseTest(next.hardcopy());
			}
			return next;			
		}		
		CPF nCPF = new CPF(subset,false);
		int[] projection = CPF.getProjectionMapping(_DomainProduct, subset);
		int[] nQ = nCPF.realaddr2addr(0);
		int[] Q = realaddr2addr(0);
		for (int i = 0; i < size(); i++)
		{
			Value mV = get(i);
			CPF.applyProjectionMapping(Q, projection, nQ);
			Value oV = nCPF.get(nQ);
			nCPF.put(nQ, Field.add(mV, oV));
			addOne(Q);
		}
		return SparseTest(nCPF);
	}
	/*! Expand by replication (can do extract, not as fast)
	 * \param[in] superset	 the super product space
	 * \return the new CPF founded on the super product space
	 */
	public CPF expand(BeliefNode[] superset)
	{
		if(this.isSubClass())
		{
			CPF next = new Projection(this, superset);
			if(next.canCache())
			{
			    return SparseTest(next.hardcopy());
			}
			return next;
		}
		CPF nCPF = new CPF(superset,true);
		int[] projection = CPF.getProjectionMapping(superset, _DomainProduct);
		int[] nQ = realaddr2addr(0);
		int[] Q = nCPF.realaddr2addr(0);
		for (int i = 0; i < nCPF.size(); i++)
		{
			CPF.applyProjectionMapping(Q, projection, nQ);
			nCPF.put(i, get(nQ));
			nCPF.addOne(Q);
		}
		return SparseTest(nCPF);
	}
	/*! Multiply lhs by rhs
	 * \param[in] lhs 
	 * \param[in] rhs 
	 * \return lhs * rhs
	 */
	static public CPF multiply(CPF lhs, CPF rhs)
	{
		if(lhs.isSubClass() || rhs.isSubClass())
		{
			CPF next = new Multiply(lhs,rhs);
			if(next.canCache())
			{
			    return SparseTest(next.hardcopy());
			}
			return next;
		}
		CPF rhscomp = CPF.getCompatible(lhs, rhs);
		for (int i = 0; i < lhs.size(); i++)
		{
			Value a = lhs.get(i);
			Value b = rhscomp.get(i);
			lhs.put(i, Field.mult(a, b));
		}
		return lhs;
	}
	/*! Divide lhs by rhs
	 * \param[in] lhs 
	 * \param[in] rhs 
	 * \return lhs / rhs
	 */
	static public CPF divide(CPF lhs, CPF rhs)
	{
		if(lhs.isSubClass() || rhs.isSubClass())
		{
			CPF next = new Divide(lhs,rhs);
			if(next.canCache())
			{
			    return SparseTest(next.hardcopy());
			}
			return next;
		}
		CPF rhscomp = CPF.getCompatible(lhs, rhs);
		for (int i = 0; i < lhs.size(); i++)
		{
			Value a = lhs.get(i);
			Value b = rhscomp.get(i);
			lhs.put(i, Field.divide(a, b));
		}
		return lhs;
	}

	/*! normalize with lazy, assumes cacheable and does hardcopy which will trigger
	 * if cpf is subclass, return new normalized CPF, else cpf is normalized
	 */
	public static CPF normalize(CPF cpf)
	{
		if(cpf.isSubClass())
		{
			CPF norm = cpf.hardcopy();
			norm.normalize();
			return norm;
		}
		else
		{
			cpf.normalize();
			return cpf;
		}
	}
	
	/*! Normalize by the entire sum of the CPF
	 * this := cpf divide / sum(cpf)
	 */
	public void normalize()
	{
		if(isSubClass())
		{
			System.out.println("lazy not done (norm)"); return;
		}
		Value rSum = new ValueZero();
		for (int i = 0; i < size(); i++)
		{
			Value a = get(i);
			rSum = Field.add(rSum, a);
		}
		for (int i = 0; i < size(); i++)
		{
			Value a = get(i);
			put(i, Field.divide(a, rSum));
		}
	}
	/*! Normalize each column
	 * this := cpf divide / sum(col))
	 */
	public void normalizeByDomain()
	{
		if(isSubClass())
		{
			System.out.println("lazy not done (normdom)"); return;
		}
		int dN = _DomainProduct[0].getDomain().getOrder();
		for (int k = 0; k < size() / dN; k++)
		{
			Value rSum = new ValueZero();
			for (int i = 0; i < dN; i++)
			{
				Value a = get(i * size() / dN + k);
				rSum = Field.add(rSum, a);
			}
			for (int i = 0; i < dN; i++)
			{
				Value a = get(i * size() / dN + k);
				put(i * size() / dN + k, Field.divide(a, rSum));
			}
		}
	}
	/*! Return the compatible B such that A o B works where o = * or /
	 * \param[in] A the unchange
	 * \param[in] B the CPF to check for compatibility
	 * \return a new CPF that is compatible with operators on A
	 */
	public static CPF getCompatible(CPF A, CPF B)
	{
		if(A.isSubClass() || B.isSubClass())
		{
			System.out.println("lazy not done getcomp"); return null;
		}
		BeliefNode[] X = A.getDomainProduct();
		BeliefNode[] Y = B.getDomainProduct();
		if (A.size() != B.size())
		{
			CPF R = B.expand(X);
			return R;
		}
		else
		{
			if(X.length == Y.length)
			{
				for (int i = 0; i < X.length; i++)
				{
					if (X[i].getName() != Y[i].getName())
					{
						CPF R2 = B.expand(X);
						return R2;
					}
				}
			}
			else
			{
				return B.expand(X);
			}
			return B;
		}
	}
	/*! Convert the entries in the table from double to rational 
	 * \todo: make visitor? hack better?
	 */
	public void convertDouble2Rational()
	{
		for (int i = 0; i < size(); i++)
		{
			Value mV = get(i);
			if (mV instanceof ValueDouble)
			{
				double vd = ((ValueDouble) mV).getValue();
				int top = (int) (vd * 10000);
				int bottom = 10000;
				put(i, new ValueRational(top, bottom));
			}
		}
	}
	/*! Convert the entries in the table from double to float 
	 * \todo: make visitor? hack better?
	 */
	public void convertDouble2Float()
	{
		for (int i = 0; i < size(); i++)
		{
			Value mV = get(i);
			if (mV instanceof ValueDouble)
			{
				double vd = ((ValueDouble) mV).getValue();
				put(i, new ValueFloat((float)vd));
			}
		}
	}
	/*! Zero based on evidence
	 * \param evNodes evNodes is subset of nodes that has evidence
	 * \todo make faster??
	 */
	public void zeroExceptForNodeEvidence(BeliefNode[] evNodes)
	{
		int[] query = realaddr2addr(0);
		int[] zero = realaddr2addr(0);
		for (int i = 0; i < zero.length; i++)
		{
			zero[i] = -1;
			for (int j = 0; j < evNodes.length; j++)
			{
				if (evNodes[j] == _DomainProduct[i])
				{
					if (evNodes[j].getEvidence() instanceof DiscreteEvidence)
					{
						DiscreteEvidence DE = (DiscreteEvidence) evNodes[j].getEvidence();
						zero[i] = DE.getDirectValue();
						//zero[i] = evNodes[j].getEvidence().getEvidenceValue();
					}
				}
			}
		}
		for (int i = 0; i < size(); i++)
		{
			boolean keep = true;
			for (int j = 0; j < zero.length; j++)
			{
				keep = keep && (zero[j] == -1 || zero[j] == query[j]);
			}
			if (!keep) put(i, ValueZero.SingletonZero);
			addOne(query);
		}
	}
	/*! Change the domain of a CPF
	 * \param cpf -	the original cpf
	 * \param bad -	the bad node
	 * \param domNew - the new domain
	 * \param map - the mapping for keeping from the old domain to the old domain
	 * \return the new CPF with the domain rearrangement/change
	 */
	public static CPF changeDomain(CPF cpf, BeliefNode bad, Domain domNew, int[] map)
	{
		Domain Old = bad.getDomain();
		bad.setDomain(domNew);
		BeliefNode[] prod = cpf.getDomainProduct();
		CPF newCPF = new CPF(prod);
		int idx = -1;
		for (int i = 0; i < prod.length; i++)
		{
			if (prod[i] == bad) idx = i;
		}
		bad.setDomain(Old);
		int[] q = cpf.realaddr2addr(0);
		for (int i = 0; i < cpf.size(); i++)
		{
			Value v = cpf.get(i);
			int dQ = q[idx];
			if (map[dQ] >= 0)
			{
				q[idx] = map[dQ];
				bad.setDomain(domNew);
				newCPF.put(q, v);
				bad.setDomain(Old);
				q[idx] = dQ;
			}
			cpf.addOne(q);
		}
		bad.setDomain(Old);
		return newCPF;
	}
	/*! Copy a CPF
	 * \return a copy of this
	 */
	public CPF copy()
	{
		CPF res = new CPF(this.getDomainProduct(),true);
		for (int i = 0; i < size(); i++)
		{
			res.put(i, _Values[i]);
		}
		return res;
	}
	/*! Copy a CPF, hard
	 * \return a copy that works on subclassed (not as fast)
	 */
	public CPF hardcopy()
	{
		CPF res = new CPF(this.getDomainProduct(),true);
		boolean done = false;
		int[] q = res.realaddr2addr(0);
		while(!done)
		{
			res.put(q, get(q));
			done = res.addOne(q);
		}
		return res;
		
	}
	/*! function that must be overrided by subclasses
	 * \return
	 */
	public boolean isSubClass()
	{
		return false;
	}
	
	/*! Since using lazy eval, check to see if there is memory to cache a CPF
	 * \return true if it can, false else wise
	 */
	public boolean canCache()
	{
		int Max = CacheManager.getInstance().getMax();
		int Cur = 1;
		for(int i = 0; i < _SizeBuffer.length; i++)
		{
			Cur *= _SizeBuffer[i];
			if(Cur >= Max) return false;
		}
		CacheManager.getInstance().grab(Cur);
		return true;
		
	}
}