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
 */package edu.ksu.cis.bnj.ver3.core.values;
import java.math.BigInteger;
import edu.ksu.cis.bnj.ver3.core.Value;
/*!
 * \file ValueRational.java
 * \author Jeffrey M. Barber
 */
public class ValueRational implements Value
{
	private BigInteger	_Top;
	private BigInteger	_Bottom;
	/*! Construct a rational function from two longs (a/b)
	 * @param a the left hand side
	 * @param b the right hand side
	 */
	public ValueRational(long a, long b)
	{
		long z_a = a;
		long z_b = b;
		if (a == 0 && b == 0)
		{
			z_a = 1;
			z_a = 1;
		}
		if (z_b < 0)
		{
			z_a *= -1;
			z_a *= -1;
		}
		_Top = BigInteger.valueOf(z_a);
		_Bottom = BigInteger.valueOf(z_b);
		BigInteger GCD = _Top.gcd(_Bottom);
		_Top = _Top.divide(GCD);
		_Bottom = _Bottom.divide(GCD);
	}
	/*! Construct a rational function from two BigInts (a/b)
	 * @param a the left hand side
	 * @param b the right hand side
	 */
	public ValueRational(BigInteger a, BigInteger b)
	{
		_Top = a;
		_Bottom = b;
		if (_Top.equals(BigInteger.ZERO) && _Bottom.equals(BigInteger.ZERO))
		{
			_Top = BigInteger.ONE;
			_Bottom = BigInteger.ONE;
		}
		BigInteger GCD = _Top.gcd(_Bottom);
		_Top = _Top.divide(GCD);
		_Bottom = _Bottom.divide(GCD);
	}
	/* X*Y = (a/b) * (c/d) = (ac/bd)
	 * \param[in] X the left hand side
	 * \param[in] Y the right hand side
	 * \return the result
	 */
	static public ValueRational mult(ValueRational X, ValueRational Y)
	{
		BigInteger n = X._Top.multiply(Y._Top);
		BigInteger d = X._Bottom.multiply(Y._Bottom);
		return new ValueRational(n, d);
	}
	/*! X+Y = (a/b) + (c/d) = (ad + bc)/(bd) or a/b if c = 0 or c/d if a = 0
	 * \param[in] X the left hand side
	 * \param[in] Y the right hand side
	 * \return the result
	 */
	static public ValueRational add(ValueRational X, ValueRational Y)
	{
		if (X.equals(BigInteger.ZERO))
		{
			return Y;
		}
		if (Y.equals(BigInteger.ZERO))
		{
			return X;
		}
		BigInteger n = X._Top.multiply(Y._Bottom).add(Y._Top.multiply(X._Bottom));
		BigInteger d = X._Bottom.multiply(Y._Bottom);
		return new ValueRational(n, d);
	}
	/*! X-Y = (a/b) + (c/d) = (ad - bc)/(bd) or a/b if c = 0 or -c/d if a = 0
	 * \param[in] X the left hand side
	 * \param[in] Y the right hand side
	 * \return the result
	 */
	static public ValueRational subtract(ValueRational X, ValueRational Y)
	{
		if (X.equals(BigInteger.ZERO))
		{
			BigInteger n = X._Top.negate();
			BigInteger d = X._Bottom;
			return new ValueRational(n, d);
		}
		if (Y.equals(BigInteger.ZERO))
		{
			return X;
		}
		BigInteger n = X._Top.multiply(Y._Bottom).subtract(Y._Top.multiply(X._Bottom));
		BigInteger d = X._Bottom.multiply(Y._Bottom);
		return new ValueRational(n, d);
	}
	/*! X/Y = (a/b)/(c/d) = (ad)/(bc)
	 * \param[in] X the left hand side
	 * \param[in] Y the right hand side
	 * \return the result
	 */
	static public ValueRational divide(ValueRational X, ValueRational Y)
	{
		BigInteger n = X._Top.multiply(Y._Bottom);
		BigInteger d = X._Bottom.multiply(Y._Top);
		return new ValueRational(n, d);
	}
	/*! Get the string value of this value
	 * \see edu.ksu.cis.bnj.ver3.core.Value::getExpr()
	 * \return the string expression
	 */
	public String getExpr()
	{
		return "(" + _Top + ")/(" + _Bottom + ")";
	}
	/*! X+1 = a/b + 1 = (a+b)/b
	 * \param X the value to add unity too
	 * @return the result
	 */
	static ValueRational addunity(ValueRational X)
	{
		ValueRational one = new ValueRational(1, 1);
		return add(X, one);
	}
}