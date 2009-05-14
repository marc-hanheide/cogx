/*
 * @(#)GenFactory.java	1.13 04/04/07
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
 * @(#)GenFactory.java	1.13 04/04/07
 */

package org.cognitivesystems.idl.toJavaPortable;

// NOTES:

/**
 *
 **/
public class GenFactory implements org.cognitivesystems.idl.GenFactory
{

  public org.cognitivesystems.idl.AttributeGen createAttributeGen ()
  {
    if (Util.corbaLevel (2.4f, 99.0f)) // <d60023>
      return new AttributeGen24 ();
    else
      return new AttributeGen ();
  } // createAttributeGen

  public org.cognitivesystems.idl.ConstGen createConstGen ()
  {
    return new ConstGen ();
  } // createConstGen

  public org.cognitivesystems.idl.NativeGen createNativeGen ()
  {
    return new NativeGen ();
  } // createNativeGen

  public org.cognitivesystems.idl.EnumGen createEnumGen ()
  {
    return new EnumGen ();
  } // createEnumGen

  public org.cognitivesystems.idl.ExceptionGen createExceptionGen ()
  {
    return new ExceptionGen ();
  } // createExceptionGen

  public org.cognitivesystems.idl.ForwardGen createForwardGen ()
  {
    return null;
  } // createForwardGen

  public org.cognitivesystems.idl.ForwardValueGen createForwardValueGen ()
  {
    return null;
  } // createForwardValueGen

  public org.cognitivesystems.idl.IncludeGen createIncludeGen ()
  {
    return null;
  } // createIncludeGen

  public org.cognitivesystems.idl.InterfaceGen createInterfaceGen ()
  {
    return new InterfaceGen ();
  } // createInterfaceGen

  public org.cognitivesystems.idl.ValueGen createValueGen ()
  {
    if (Util.corbaLevel (2.4f, 99.0f)) // <d60023>
      return new ValueGen24 ();
    else
      return new ValueGen ();
  } // createValueGen

  public org.cognitivesystems.idl.ValueBoxGen createValueBoxGen ()
  {
    if (Util.corbaLevel (2.4f, 99.0f)) // <d60023>
      return new ValueBoxGen24 ();
    else
      return new ValueBoxGen ();
  } // createValueBoxGen

  public org.cognitivesystems.idl.MethodGen createMethodGen ()
  {
    if (Util.corbaLevel (2.4f, 99.0f)) // <d60023>
      return new MethodGen24 ();
    else
      return new MethodGen ();
  } // createMethodGen

  public org.cognitivesystems.idl.ModuleGen createModuleGen ()
  {
    return new ModuleGen ();
  } // createModuleGen

  public org.cognitivesystems.idl.ParameterGen createParameterGen ()
  {
    return null;
  } // createParameterGen

  public org.cognitivesystems.idl.PragmaGen createPragmaGen ()
  {
    return null;
  } // createPragmaGen

  public org.cognitivesystems.idl.PrimitiveGen createPrimitiveGen ()
  {
    return new PrimitiveGen ();
  } // createPrimitiveGen

  public org.cognitivesystems.idl.SequenceGen createSequenceGen ()
  {
    return new SequenceGen ();
  } // createSequenceGen

  public org.cognitivesystems.idl.StringGen createStringGen ()
  {
    return new StringGen ();
  } // createSequenceGen

  public org.cognitivesystems.idl.StructGen createStructGen ()
  {
    return new StructGen ();
  } // createStructGen

  public org.cognitivesystems.idl.TypedefGen createTypedefGen ()
  {
    return new TypedefGen ();
  } // createTypedefGen

  public org.cognitivesystems.idl.UnionGen createUnionGen ()
  {
    return new UnionGen ();
  } // createUnionGen
} // class GenFactory
