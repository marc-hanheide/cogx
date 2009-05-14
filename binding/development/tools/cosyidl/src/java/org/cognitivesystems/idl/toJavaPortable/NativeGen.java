/*
 * @(#)NativeGen.java	1.5 03/12/19
 *
 * Copyright 2004 Sun Microsystems, Inc. All rights reserved.
 * SUN PROPRIETARY/CONFIDENTIAL. Use is subject to license terms.
 */

package org.cognitivesystems.idl.toJavaPortable;

// NOTES:

import java.io.File;
import java.io.PrintWriter;
import java.util.Hashtable;
import java.util.Vector;

import org.cognitivesystems.idl.*;


/**
 *
 **/
public class NativeGen implements org.cognitivesystems.idl.NativeGen
{
  /**
   * Public zero-argument constructor.
   **/
  public NativeGen ()
  {
  } // ctor

  /**
   * Generate Java code for an IDL constant.  A constant is written to
   * a new class only when it is not a member of an interface; otherwise
   * it written to the interface class in which it resides.
   **/
  public void generate (Hashtable symbolTable, NativeEntry c, PrintWriter s)
  {
	// noop, do not generate anything
  } // generate

} // class NativeGen

