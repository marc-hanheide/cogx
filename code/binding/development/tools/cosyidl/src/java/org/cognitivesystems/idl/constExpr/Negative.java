/*
 * @(#)Negative.java	1.12 03/12/19
 *
 * Copyright 2004 Sun Microsystems, Inc. All rights reserved.
 * SUN PROPRIETARY/CONFIDENTIAL. Use is subject to license terms.
 */
/*
 * COMPONENT_NAME: idl.parser
 *
 * ORIGINS: 27
 *
 * Licensed Materials - Property of IBM
 * 5639-D57 (C) COPYRIGHT International Business Machines Corp. 1997, 1999
 * RMI-IIOP v1.0
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with IBM Corp.
 *
 * @(#)Negative.java	1.12 03/12/19
 */

package org.cognitivesystems.idl.constExpr;

// NOTES:

import java.math.BigInteger;

import org.cognitivesystems.idl.Util;

public class Negative extends UnaryExpr
{
  protected Negative (Expression operand)
  {
    super ("-", operand);
  } // ctor

  public Object evaluate () throws EvaluationException
  {
    try
    {
      Number op = (Number)operand ().evaluate ();

      if (op instanceof Float || op instanceof Double)
        value (new Double (-op.doubleValue ()));
      else
      {
        // Multiply by -1
        //daz        value (new Long (-op.longValue ()));
        BigInteger tmpOp = (BigInteger)op;
        value (tmpOp.multiply (BigInteger.valueOf (-1)));
     }
    }
    catch (ClassCastException e)
    {
      String[] parameters = {Util.getMessage ("EvaluationException.neg"), operand ().value ().getClass ().getName ()};
      throw new EvaluationException (Util.getMessage ("EvaluationException.2", parameters));
    }
    return value ();
  } // evaluate
} // class Negative
