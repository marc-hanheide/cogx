/*
 * @(#)ValueBoxGen.java	1.12 03/12/19
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
 * @(#)ValueBoxGen.java	1.12 03/12/19
 */

package org.cognitivesystems.idl;

// NOTES:

import java.io.PrintWriter;
import java.util.Hashtable;
import org.cognitivesystems.idl.*;

public interface ValueBoxGen extends Generator
{
  void generate (Hashtable symbolTable, ValueBoxEntry entry, PrintWriter stream);
} // interface ValueBoxGen
