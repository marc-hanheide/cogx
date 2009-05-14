/*
 * @(#)DefaultFactory.java	1.10 03/12/19
 *
 * Copyright 2004 Sun Microsystems, Inc. All rights reserved.
 * SUN PROPRIETARY/CONFIDENTIAL. Use is subject to license terms.
 */
/*
 * COMPONENT_NAME: idl.toJava
 *
 * ORIGINS: 27
 *
 * Licensed Materials - Property of IBM
 * 5639-D57 (C) COPYRIGHT International Business Machines Corp. 1997, 1999
 * RMI-IIOP v1.0
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with IBM Corp.
 *
 * @(#)DefaultFactory.java	1.10 03/12/19
 */

package org.cognitivesystems.idl.toJavaPortable;

// NOTES:
// -D62023 klr new class

import java.io.PrintWriter;
import java.util.Vector;

import org.cognitivesystems.idl.*;


/**
 *
 **/
public class DefaultFactory implements AuxGen
{
  /**
   * Public zero-argument constructor.
   **/
  public DefaultFactory ()
  {
  } // ctor

  /**
   * Generate the default value factory class. Provides general algorithm for
   * auxiliary binding generation:
   * 1.) Initialize symbol table and symbol table entry members,
   *     common to all generators.
   * 2.) Initialize members unique to this generator.
   * 3.) Open print stream
   * 4.) Write class heading (package, prologue, source comment, class
   *     statement, open curly
   * 5.) Write class body (member data and methods)
   * 6.) Write class closing (close curly)
   * 7.) Close the print stream
   **/
  public void generate (java.util.Hashtable symbolTable, org.cognitivesystems.idl.SymtabEntry entry)
  {
    this.symbolTable = symbolTable;
    this.entry       = entry;
    init ();
    openStream ();
    if (stream == null)
      return;
    writeHeading ();
    writeBody ();
    writeClosing ();
    closeStream ();
  } // generate

  /**
   * Initialize variables unique to this generator.
   **/
  protected void init ()
  {
    factoryClass = entry.name () + "DefaultFactory";
    factoryInterface = entry.name () + "ValueFactory";
    factoryType = Util.javaName (entry);
    implType = entry.name () + "Impl"; // default implementation class
  } // init

  /**
   * @return true if entry has any factory methods declared
   **/
  protected boolean hasFactoryMethods ()
  {
    Vector init = ((ValueEntry)entry).initializers ();
    if (init != null && init.size () > 0)
      return true;
    else
      return false;
  } // hasFactoryMethods

  /**
   * Open the print stream for subsequent output.
   **/
  protected void openStream ()
  {
    stream = Util.stream (entry, "DefaultFactory.java");
  } // openStream

  /**
   * Generate the heading, including the package, imports,
   * source comment, class statement, and left curly.
   **/
  protected void writeHeading ()
  {
    Util.writePackage (stream, entry, Util.TypeFile); // REVISIT - same as interface?
    Util.writeProlog (stream, stream.name ());
    if (entry.comment () != null)
      entry.comment ().generate ("", stream);
    stream.print ("public class " + factoryClass + " implements ");
    if (hasFactoryMethods ())
	stream.print (factoryInterface);
    else
	stream.print ("org.omg.CORBA.portable.ValueFactory");
    stream.println (" {");
  } // writeHeading

  /**
   * Generate the contents of this class
   **/
  protected void writeBody ()
  {
    writeFactoryMethods ();
    stream.println ();
    writeReadValue ();
  } // writeBody

  /**
   * Generate members of this class.
   **/
  protected void writeFactoryMethods ()
  {
    Vector init = ((ValueEntry)entry).initializers ();
    if (init != null)
    {
      for (int i = 0; i < init.size (); i++)
      {
        MethodEntry element = (MethodEntry) init.elementAt (i);
        element.valueMethod (true); //tag value method if not tagged previously
        ((MethodGen24) element.generator ()).defaultFactoryMethod (symbolTable, element, stream);
      }
    }
  } // writeFactoryMethods

  /**
   * Generate default read_value
   **/
  protected void writeReadValue ()
  {
     stream.println ("  public java.io.Serializable read_value (org.omg.CORBA_2_3.portable.InputStream is)");
     stream.println ("  {");
     stream.println ("    return is.read_value(new " + implType + " ());");
     stream.println ("  }");
  } // writeReadValue

  /**
   * Generate the closing statements.
   **/
  protected void writeClosing ()
  {
    stream.println ('}');
  } // writeClosing

  /**
   * Write the stream to file by closing the print stream.
   **/
  protected void closeStream ()
  {
    stream.close ();
  } // closeStream

  protected java.util.Hashtable     symbolTable;
  protected org.cognitivesystems.idl.SymtabEntry entry;
  protected GenFileStream           stream;

  // Unique to this generator
  protected String factoryClass;
  protected String factoryInterface;
  protected String factoryType;
  protected String implType;
} // class Holder
