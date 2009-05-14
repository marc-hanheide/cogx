/*
 * @(#)ValueBoxGen.java	1.12 03/12/19
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
 * @(#)ValueBoxGen.java	1.12 03/12/19
 */

package org.cognitivesystems.idl.toJavaPortable;

// NOTES:
// -D60929   <klr> Update for RTF2.4 changes
// -D61056   <klr> Use Util.helperName
// -D62023   <klr> Fix generation botch in helper.read for boxed bounded strings

import java.io.File;
import java.io.PrintWriter;
import java.util.Hashtable;
import java.util.Enumeration;
import java.util.Vector;

import org.cognitivesystems.idl.*;


/**
 *
 **/
public class ValueBoxGen implements org.cognitivesystems.idl.ValueBoxGen, JavaGenerator
{
  /**
   * Public zero-argument constructor.
   **/
  public ValueBoxGen ()
  {
  } // ctor

  /**
   *
   **/
  public void generate (Hashtable symbolTable, ValueBoxEntry v, PrintWriter str)
  {
    this.symbolTable = symbolTable;
    this.v = v;

    TypedefEntry member = ((InterfaceState) v.state ().elementAt (0)).entry;
    SymtabEntry mType = member.type ();
    // if it's primitive type, generate a java class
    if (mType instanceof PrimitiveEntry)
    {
      openStream ();
      if (stream == null)
        return;
      writeHeading ();
      writeBody ();
      writeClosing ();
      closeStream ();
    }
    else
    {
      // If a constructed type is nested in the value box,
      // value v struct s {...};
      // the bindings for the nested type must be handled here
      Enumeration e = v.contained ().elements ();
      while (e.hasMoreElements ())
      {
        SymtabEntry contained = (SymtabEntry) e.nextElement ();

        // in case of value box w/ nested enum, ex: value v enum e {e0, e1,...};
        // the SymtabEntry for the enum and labels are contained in the vector.
        // Must check the type to ignore the SymtabEntry for labels.
        if (contained.type () != null)
          contained.type ().generate (symbolTable, stream);
      }
    }
    generateHelper ();
    generateHolder ();
  } // generate

  /**
   *
   **/
  protected void openStream ()
  {
    stream = Util.stream (v, ".java");
  } // openStream

  /**
   *
   **/
  protected void generateHelper ()
  {
    ((Factories)Compile.compiler.factories ()).helper ().generate (symbolTable, v);
  } // generateHelper

  /**
   *
   **/
  protected void generateHolder ()
  {
    ((Factories)Compile.compiler.factories ()).holder ().generate (symbolTable, v);
  } // generateHolder

  /**
   *
   **/
  protected void writeHeading ()
  {
    Util.writePackage (stream, v);
    Util.writeProlog (stream, ((GenFileStream)stream).name ());
    if (v.comment () != null)
      v.comment ().generate ("", stream);
//  stream.println ("public class " + v.name () + " implements org.omg.CORBA.portable.ValueBase, org.omg.CORBA.portable.Streamable");
    stream.println ("public class " + v.name () + " implements org.omg.CORBA.portable.ValueBase"); // <d60929>
    stream.println ("{");
  } // writeHeading

  /**
   *
   **/
  protected void writeBody ()
  {
    InterfaceState member = (InterfaceState) v.state ().elementAt (0);
    SymtabEntry entry = (SymtabEntry) member.entry;
    Util.fillInfo (entry);
    if (entry.comment () != null)
      entry.comment ().generate (" ", stream);
    stream.println ("  public " +  Util.javaName (entry) + " value;");
    stream.println ("  public " +  v.name () + " (" + Util.javaName (entry) + " initial)");
    stream.println ("  {");
    stream.println ("    value = initial;");
    stream.println ("  }");
    stream.println ();
    writeTruncatable (); // <d60929>
//  writeStreamableMethods ();
  } // writeBody

  /**
   *
   **/
  protected void writeTruncatable () // <d60929>
  {
   // Per Simon, 4/6/98, emit _truncatable_ids()
      stream.println ("  public String[] _truncatable_ids() {");
      stream.println ("      return " + Util.helperName(v, true) + ".get_instance().get_truncatable_base_ids();"); // <d61056>
      stream.println ("  }");
      stream.println ();
  } // writeTruncatable

  /**
   *
   **/
  protected void writeClosing ()
  {
    stream.println ("} // class " + v.name ());
  } // writeClosing

  /**
   *
   **/
  protected void closeStream ()
  {
    stream.close ();
  } // closeStream

  /**
   *
   **/
  protected void writeStreamableMethods ()
  {
    stream.println ("  public void _read (org.omg.CORBA.portable.InputStream istream)");
    stream.println ("  {");
    streamableRead ("this", v, stream);
    stream.println ("  }");
    stream.println ();
    stream.println ("  public void _write (org.omg.CORBA.portable.OutputStream ostream)");
    stream.println ("  {");
    write (0, "    ", "this", v, stream);
    stream.println ("  }");
    stream.println ();
    stream.println ("  public org.omg.CORBA.TypeCode _type ()");
    stream.println ("  {");
    stream.println ("    return " + Util.helperName (v, false) + ".type ();"); // <d61056>
    stream.println ("  }");
  } // writeStreamableMethods

  ///////////////
  // From JavaGenerator

  public int helperType (int index, String indent, TCOffsets tcoffsets, String name, SymtabEntry entry, PrintWriter stream)
  {
    ValueEntry vt = (ValueEntry) entry;
    TypedefEntry member = (TypedefEntry) ((InterfaceState) (vt.state ()).elementAt (0)).entry;
    SymtabEntry mType = Util.typeOf (member);
    index = ((JavaGenerator)mType.generator ()).type (index, indent, tcoffsets, name, mType, stream);
    stream.println (indent + name + " = org.omg.CORBA.ORB.init ().create_value_box_tc ("
      + "_id, "
      + '"' + entry.name () + "\", "
      + name
      + ");");
    return index;
  } // helperType

  public int type (int index, String indent, TCOffsets tcoffsets, String name, SymtabEntry entry, PrintWriter stream) {
    stream.println (indent + name + " = " + Util.helperName (entry, true) + ".type ();"); // <d61056>
    return index;
  } // type

  public int read (int index, String indent, String name, SymtabEntry entry, PrintWriter stream)
  {
    return index;
  } // read

  public void helperRead (String entryName, SymtabEntry entry, PrintWriter stream)
  {
  // <d59418 - KLR> per Simon, make "static" read call istream.read_value.
  //		    put real marshalling code in read_value.
    stream.println ("    return (" + entryName +") ((org.omg.CORBA_2_3.portable.InputStream) istream).read_value (get_instance());"); // <d60929>
    stream.println ("  }");
    stream.println ();

    // done with "read", now do "read_value with real marshalling code.

    stream.println ("  public java.io.Serializable read_value (org.omg.CORBA.portable.InputStream istream)"); // <d60929>
    stream.println ("  {");
    // end of <d59418> changes

    String indent = "    ";
    Vector vMembers = ((ValueBoxEntry) entry).state ();
    TypedefEntry member = ((InterfaceState) vMembers.elementAt (0)).entry;
    SymtabEntry mType = member.type ();
    if (mType instanceof PrimitiveEntry ||
        mType instanceof SequenceEntry ||
        mType instanceof TypedefEntry ||
        mType instanceof StringEntry ||
        !member.arrayInfo ().isEmpty ()) {
      stream.println (indent + Util.javaName (mType) + " tmp;"); // <d62023>
      ((JavaGenerator)member.generator ()).read (0, indent, "tmp", member, stream);
    }
    else if (mType instanceof ValueEntry || mType instanceof ValueBoxEntry)
      stream.println (indent + Util.javaQualifiedName (mType) + " tmp = (" +
                      Util.javaQualifiedName (mType) + ") ((org.omg.CORBA_2_3.portable.InputStream)istream).read_value (" + Util.helperName (mType, true) + ".get_instance ());"); // <d60929> // <d61056>
    else
      stream.println (indent + Util.javaName (mType) + " tmp = " +
                      Util.helperName ( mType, true ) + ".read (istream);"); // <d61056>
    if (mType instanceof PrimitiveEntry)
      stream.println (indent + "return new " + entryName + " (tmp);");
    else
      stream.println (indent + "return tmp;");
  } // helperRead

  public void helperWrite (SymtabEntry entry, PrintWriter stream)
  {
    // <d59418 - KLR> per Simon, make "static" write call istream.write_value.
    //		    put real marshalling code in write_value.
    stream.println ("    ((org.omg.CORBA_2_3.portable.OutputStream) ostream).write_value (value, get_instance());"); // <d60929>
    stream.println ("  }");
    stream.println ();

    // done with "write", now do "write_value with real marshalling code.

    stream.println ("  public void write_value (org.omg.CORBA.portable.OutputStream ostream, java.io.Serializable obj)"); // <d60929>
    stream.println ("  {");

    String entryName = Util.javaName(entry);
    stream.println ("    " + entryName + " value  = (" + entryName + ") obj;");
    write (0, "    ", "value", entry, stream);
  } // helperWrite

  public int write (int index, String indent, String name, SymtabEntry entry, PrintWriter stream)
  {
    Vector vMembers = ( (ValueEntry) entry ).state ();
    TypedefEntry member = ((InterfaceState) vMembers.elementAt (0)).entry;
    SymtabEntry mType = member.type ();

    if (mType instanceof PrimitiveEntry || !member.arrayInfo ().isEmpty ())
      index = ((JavaGenerator)member.generator ()).write (index, indent, name + ".value", member, stream);
    else if (mType instanceof SequenceEntry || mType instanceof StringEntry || mType instanceof TypedefEntry || !member.arrayInfo ().isEmpty ())
      index = ((JavaGenerator)member.generator ()).write (index, indent, name, member, stream);
    else if (mType instanceof ValueEntry || mType instanceof ValueBoxEntry)
      stream.println (indent
                      + "((org.omg.CORBA_2_3.portable.OutputStream)ostream).write_value ((java.io.Serializable) value, " // <d60929>
                      +  Util.helperName (mType, true)  // <d61056>
		      + ".get_instance ());"); // <d61056>
    else
      stream.println (indent + Util.helperName (mType, true) + ".write (ostream, " + name + ");"); // <d61056>
    return index;
  } // write

  protected void writeAbstract ()
  {
  } // writeAbstract

  protected void streamableRead (String entryName, SymtabEntry entry, PrintWriter stream)
  {
    Vector vMembers = ( (ValueBoxEntry) entry ).state ();
    TypedefEntry member = ((InterfaceState) vMembers.elementAt (0)).entry;
    SymtabEntry mType = member.type ();
    if (mType instanceof PrimitiveEntry || mType instanceof SequenceEntry || mType instanceof TypedefEntry ||
        mType instanceof StringEntry || !member.arrayInfo ().isEmpty ())
    {
      SymtabEntry mEntry = (SymtabEntry) ((InterfaceState) vMembers.elementAt (0)).entry;
      ((JavaGenerator)member.generator ()).read (0, "    ", entryName + ".value", member, stream);
    }
    else if (mType instanceof ValueEntry || mType instanceof ValueBoxEntry)
      stream.println ("    " + entryName + ".value = (" + Util.javaQualifiedName (mType) + ") ((org.omg.CORBA_2_3.portable.InputStream)istream).read_value (" + Util.helperName(mType, true) + ".get_instance ());"); // <d60929> // <d61056>
    else
      stream.println ("    " + entryName + ".value = " + Util.helperName (mType, true) + ".read (istream);"); // <d61056>
  } // streamableRead

  protected Hashtable  symbolTable = null;
  protected ValueBoxEntry v = null;
  protected PrintWriter stream = null;
} // class ValueBoxGen
