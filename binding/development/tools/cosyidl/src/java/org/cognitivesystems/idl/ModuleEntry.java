/*
 * @(#)ModuleEntry.java	1.12 03/12/19
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
 * @(#)ModuleEntry.java	1.12 03/12/19
 */

package org.cognitivesystems.idl;

// NOTES:

import java.io.PrintWriter;
import java.util.Hashtable;
import java.util.Vector;

/**
 * This is the symbol table entry for modules.
 **/
public class ModuleEntry extends SymtabEntry
{
  protected ModuleEntry ()
  {
    super ();
  }  // ctor

  protected ModuleEntry (ModuleEntry that)
  {
    super (that);
    _contained = (Vector)that._contained.clone ();
  } // ctor

  protected ModuleEntry (SymtabEntry that, IDLID clone)
  {
    super (that, clone);

    if (module ().equals (""))
      module (name ());
    else if (!name ().equals (""))
      module (module () + "/" + name ());
  } // ctor

  public Object clone ()
  {
    return new ModuleEntry (this);
  } // clone

  /** Invoke the module generator.
      @param symbolTable the symbol table is a hash table whose key is
       a fully qualified type name and whose value is a SymtabEntry or
       a subclass of SymtabEntry.
      @param stream the stream to which the generator should sent its output.
      @see SymtabEntry */
  public void generate (Hashtable symbolTable, PrintWriter stream)
  {
    moduleGen.generate (symbolTable, this, stream);
  } // generate

  /** Access the module generator.
      @returns an object which implements the ModuleGen interface.
      @see ModuleGen */
  public Generator generator ()
  {
    return moduleGen;
  } // generator

  /** alid entries in this vector are:  TypedefEntry, ExceptionEntry,
      StructEntry, UnionEntry, EnumEntry, ConstEntry, InterfaceEntry,
      ModuleEntry. */
  public void addContained (SymtabEntry entry)
  {
    _contained.addElement (entry);
  } // addContained

  /** This is a vector of SymtabEntry's.  Valid entries in this vector are:
      TypedefEntry, ExceptionEntry, StructEntry, UnionEntry, EnumEntry,
      ConstEntry, InterfaceEntry, ModuleEntry. */
  public Vector contained ()
  {
    return _contained;
  } // contained

  private Vector _contained = new Vector ();

  static ModuleGen moduleGen;
} // class ModuleEntry
