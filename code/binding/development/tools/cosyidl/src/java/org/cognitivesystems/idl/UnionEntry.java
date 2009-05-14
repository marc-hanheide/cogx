/*
 * @(#)UnionEntry.java	1.12 03/12/19
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
 * @(#)UnionEntry.java	1.12 03/12/19
 */

package org.cognitivesystems.idl;

// NOTES:

import java.io.PrintWriter;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

import org.cognitivesystems.idl.constExpr.Expression;


/**
 * This is the symbol table entry for unions.
 **/
public class UnionEntry extends SymtabEntry
{
  protected UnionEntry ()
  {
    super ();
  } // ctor

  protected UnionEntry (UnionEntry that)
  {
    super (that);
    if (!name ().equals (""))
    {
      module (module () + name ());
      name ("");
    }
    _branches      = (Vector)that._branches.clone ();
    _defaultBranch = that._defaultBranch;
    _contained     = that._contained;
  } // ctor

  protected UnionEntry (SymtabEntry that, IDLID clone)
  {
    super (that, clone);
    if (module ().equals (""))
      module (name ());
    else if (!name ().equals (""))
      module (module () + "/" + name ());
  } // ctor

  public Object clone ()
  {
    return new UnionEntry (this);
  } // clone

  /** Invoke the union generator.
      @param symbolTable the symbol table is a hash table whose key is
       a fully qualified type name and whose value is a SymtabEntry or
       a subclass of SymtabEntry.
      @param stream the stream to which the generator should sent its output.
      @see SymtabEntry */
  public void generate (Hashtable symbolTable, PrintWriter stream)
  {
    unionGen.generate (symbolTable, this, stream);
  } // generate

  /** Access the union generator.
      @returns an object which implements the UnionGen interface.
      @see UnionGen */
  public Generator generator ()
  {
    return unionGen;
  } // generator

  public void addBranch (UnionBranch branch)
  {
    _branches.addElement (branch);
  } // addBranch

  /** This is a vector of UnionBranch's. */
  public Vector branches ()
  {
    return _branches;
  } // branches

  /** This TypedefEntry describes the type and name for the default branch.
      Like the entries in the branches vector, only the type and name fields
      are pertinent. */
  public void defaultBranch (TypedefEntry branch)
  {
    _defaultBranch = branch;
  } // defaultBranch

  /** This TypedefEntry describes the type and name for the default branch.
      Like the entries in the branches vector, only the type and name fields
      are pertinent. */
  public TypedefEntry defaultBranch ()
  {
    return _defaultBranch;
  } // defaultBranch

  public void addContained (SymtabEntry entry)
  {
    _contained.addElement (entry);
  } // addContained

  /** This is a vector of SymtabEntry's.  It itemizes any types which
      this union contains.  For example:

      <pre>
      union A
      switch (long)
      {
        case 0: long x;
        case 1:
          Struct B
          {
            long a;
            long b;
          } y;
      }
      </pre>
      Struct B is contained within union A. */
  public Vector contained ()
  {
    return _contained;
  } // contained

  boolean has (Expression label)
  {
    Enumeration eBranches = _branches.elements ();
    while (eBranches.hasMoreElements ())
    {
      Enumeration eLabels = ((UnionBranch)eBranches.nextElement ()).labels.elements ();
      while (eLabels.hasMoreElements ())
      {
        Expression exp = (Expression)eLabels.nextElement ();
        if (exp.equals (label) || exp.value ().equals (label.value ()))
          return true;
      }
    }
    return false;
  } // has

  boolean has (TypedefEntry typedef)
  {
    Enumeration e = _branches.elements ();
    while (e.hasMoreElements ())
    {
      UnionBranch branch = (UnionBranch)e.nextElement ();
      if (!branch.typedef.equals (typedef) && branch.typedef.name ().equals (typedef.name ()))
        return true;
    }
    return false;
  } // has

  /** A vector of UnionBranch's. */
  private Vector       _branches      = new Vector ();
  private TypedefEntry _defaultBranch = null;
  private Vector       _contained     = new Vector ();

  static UnionGen unionGen;
} // class UnionEntry
