/*
 * @(#)Or.java	1.12 03/12/19
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
 * @(#)Or.java	1.12 03/12/19
 */

package org.cognitivesystems.idl.constExpr;

// NOTES:

import java.math.BigInteger;

import org.cognitivesystems.idl.Util;

public class Or extends BinaryExpr
{
  protected Or (Expression leftOperand, Expression rightOperand)
  {
    super ("|", leftOperand, rightOperand);
  } // ctor

  public Object evaluate () throws EvaluationException
  {
    try
    {
      Number l = (Number)left ().evaluate ();
      Number r = (Number)right ().evaluate ();

      if (l instanceof Float || l instanceof Double || r instanceof Float || r instanceof Double)
      {
        String[] parameters = {Util.getMessage ("EvaluationException.or"), left ().value ().getClass ().getName (), right ().value ().getClass ().getName ()};
        throw new EvaluationException (Util.getMessage ("EvaluationException.1", parameters));
      }
      else
      {
        // or
        //daz        value (new Long (l.longValue () | r.longValue ()));
        BigInteger uL = (BigInteger)toUnsigned((BigInteger)l);
        BigInteger uR = (BigInteger)toUnsigned((BigInteger)r);
        value((BigInteger)coerceToTarget(uL.or (uR)));
      }
    }
    catch (ClassCastException e)
    {
      String[] parameters = {Util.getMessage ("EvaluationException.or"), left ().value ().getClass ().getName (), right ().value ().getClass ().getName ()};
      throw new EvaluationException (Util.getMessage ("EvaluationException.1", parameters));
    }
    return value ();
  } // evaluate
} // class Or
