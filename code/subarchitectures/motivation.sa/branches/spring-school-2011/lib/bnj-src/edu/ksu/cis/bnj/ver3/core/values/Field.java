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
import edu.ksu.cis.bnj.ver3.core.Value;
/*!
 * \file Field.java
 * \author Jeffrey M. Barber
 */
public class Field
{
	/*! a+b
	 * \param a left hand side
	 * \param b right hand side
	 * \return the result
	 */
	public static Value add(Value a, Value b)
	{
		if ((a instanceof ValueDouble) && (b instanceof ValueDouble))
		{
			return new ValueDouble(((ValueDouble) a).getValue() + ((ValueDouble) b).getValue());
		}
		if ((a instanceof ValueDouble) && (b instanceof ValueUnity))
		{
			return new ValueDouble(((ValueDouble) a).getValue() + 1);
		}
		if ((b instanceof ValueDouble) && (a instanceof ValueUnity))
		{
			return new ValueDouble(((ValueDouble) b).getValue() + 1);
		}

		if ((a instanceof ValueFloat) && (b instanceof ValueFloat))
		{
			return new ValueFloat(((ValueFloat) a).getValue() + ((ValueFloat) b).getValue());
		}
		if ((a instanceof ValueFloat) && (b instanceof ValueUnity))
		{
			return new ValueFloat(((ValueFloat) a).getValue() + 1);
		}
		if ((b instanceof ValueFloat) && (a instanceof ValueUnity))
		{
			return new ValueFloat(((ValueFloat) b).getValue() + 1);
		}
		
		if ((a instanceof ValueZero))
		{
			return b;
		}
		if ((b instanceof ValueZero))
		{
			return a;
		}
		if ((a instanceof ValueRational) && (b instanceof ValueRational))
		{
			return ValueRational.add((ValueRational) a, (ValueRational) b);
		}
		if ((a instanceof ValueRational) && (b instanceof ValueUnity))
		{
			return ValueRational.addunity((ValueRational) a);
		}
		if ((b instanceof ValueRational) && (a instanceof ValueUnity))
		{
			return ValueRational.addunity((ValueRational) b);
		}
		return new ExprAdd(a, b);
	}
	/*! a-b
	 * \param a left hand side
	 * \param b right hand side
	 * \return the result
	 */
	public static Value subtract(Value a, Value b)
	{
		if ((a instanceof ValueDouble) && (b instanceof ValueDouble))
		{
			return new ValueDouble(((ValueDouble) a).getValue() - ((ValueDouble) b).getValue());
		}
		if ((a instanceof ValueDouble) && (b instanceof ValueUnity))
		{
			return new ValueDouble(((ValueDouble) a).getValue() - 1);
		}
		if ((b instanceof ValueDouble) && (a instanceof ValueUnity))
		{
			return new ValueDouble(-((ValueDouble) b).getValue() + 1);
		}
		if ((a instanceof ValueFloat) && (b instanceof ValueFloat))
		{
			return new ValueFloat(((ValueFloat) a).getValue() - ((ValueFloat) b).getValue());
		}
		if ((a instanceof ValueFloat) && (b instanceof ValueFloat))
		{
			return new ValueFloat(((ValueFloat) a).getValue() - 1);
		}
		if ((b instanceof ValueFloat) && (a instanceof ValueUnity))
		{
			return new ValueFloat(-((ValueFloat) b).getValue() + 1);
		}
		
		if ((b instanceof ValueZero))
		{
			return a;
		}
		if ((a instanceof ValueRational) && (b instanceof ValueRational))
		{
			return ValueRational.subtract((ValueRational) a, (ValueRational) b);
		}
		return Field.add(a, new ExprNegate(b));
	}
	/*! a*b
	 * \param a left hand side
	 * \param b right hand side
	 * \return the result
	 */
	public static Value mult(Value a, Value b)
	{
		if ((a instanceof ValueDouble) && (b instanceof ValueDouble))
		{
			return new ValueDouble(((ValueDouble) a).getValue() * ((ValueDouble) b).getValue());
		}
		if ((a instanceof ValueFloat) && (b instanceof ValueFloat))
		{
			return new ValueFloat(((ValueFloat) a).getValue() * ((ValueFloat) b).getValue());
		}
		if ((b instanceof ValueUnity))
		{
			return a;
		}
		if ((a instanceof ValueUnity))
		{
			return b;
		}
		if ((a instanceof ValueZero))
		{
			return a;
		}
		if ((b instanceof ValueZero))
		{
			return b;
		}
		if ((a instanceof ValueRational) && (b instanceof ValueRational))
		{
			return ValueRational.mult((ValueRational) a, (ValueRational) b);
		}
		return new ExprMultiply(a, b);
	}
	/*! a/b
	 * \param a left hand side
	 * \param b right hand side
	 * \return the result
	 */
	public static Value divide(Value a, Value b)
	{
		if ((a instanceof ValueDouble) && (b instanceof ValueDouble))
		{
			if (((ValueDouble) b).getValue() != 0.0)
			{
				return new ValueDouble(((ValueDouble) a).getValue() / ((ValueDouble) b).getValue());
			}
			else
			{
				if (((ValueDouble) a).getValue() == 0.0) return a;
			}
		}
		if ((a instanceof ValueFloat) && (b instanceof ValueFloat))
		{
			if (((ValueFloat) b).getValue() != 0.0)
			{
				return new ValueFloat(((ValueFloat) a).getValue() / ((ValueFloat) b).getValue());
			}
			else
			{
				if (((ValueFloat) a).getValue() == 0.0) return a;
			}
		}
		if ((b instanceof ValueUnity))
		{
			return a;
		}
		if ((b instanceof ValueDouble) && (a instanceof ValueUnity))
		{
			if (((ValueDouble) b).getValue() != 0.0)
			{
				return new ValueDouble(1.0 / ((ValueDouble) b).getValue());
			}
			else
			{
				if (((ValueDouble) a).getValue() == 0.0) return a;
			}
		}
		if ((b instanceof ValueFloat) && (a instanceof ValueUnity))
		{
			if (((ValueFloat) b).getValue() != 0.0)
			{
				return new ValueFloat(1.0f / ((ValueFloat) b).getValue());
			}
			else
			{
				if (((ValueFloat) a).getValue() == 0.0) return a;
			}
		}
		if ((a instanceof ValueRational) && (b instanceof ValueRational))
		{
			return ValueRational.divide((ValueRational) a, (ValueRational) b);
		}
		if ((a instanceof ValueZero))
		{
			return a;
		}
		return new ExprDivide(a, b);
	}
}