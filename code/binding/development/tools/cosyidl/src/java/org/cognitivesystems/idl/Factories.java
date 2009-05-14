/*
 * @(#)Factories.java	1.12 03/12/19
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
 * @(#)Factories.java	1.12 03/12/19
 */

package org.cognitivesystems.idl;

// NOTES:

/**
 * If the framework is being extended, this class must be extended.
 * At very least, the genFactory method must be overridden to return
 * the code generator extensions.  The remaining methods may be overridden
 * if necessary:
 * <dl>
 * <dt>symtabFactory
 * <dd>If you wish to extend the symbol table entries, this method must return the factory which constructs those extensions.  If you only want to extend a few of the symbol table entries, it may be useful to extend org.cognitivesystems.idl.DefaultSymtabFactory and only override the pertinent methods.
 * <dt>exprFactory
 * <dd>If you wish to extend the expression classes, this method must return the factory which constructs those extensions.  If you only want to extend a few of the expression classes, it may be useful to extend org.cognitivesystems.idl.constExpr.DefaultSymtabFactory and only override the pertinent methods.
 * <dt>arguments
 * <dd>If you wish to add additional arguments to the base set of arguments, extend org.cognitivesystems.idl.Arguments and override this method to return that class.
 * <dt>languageKeywords
 * <dd>If the language you are generating code in has keywords other than IDL keywords, these keywords should be returned by this method.  The framework will prepend any IDL identifiers it encounters which are in this list with an underscore (`_') to avoid compilation errors.  For instance, `catch' is a Java keyword.  If the generators are emitting Java code for the following IDL, emitting `catch' as is will cause compile errors, so it is changed to `_catch':
 * <br>
 * IDL:
 * <br>
 * const long catch = 22;
 * <br>
 * Possible generated code:
 * <br>
 * public static final int _catch = 22;
 * </dl>
 **/
public class Factories
{
  /** Return the implementation of the GenFactory interface.  If this
      returns null, then the compiler cannot generate anything. */
  public GenFactory genFactory ()
  {
    return null;
  } // genFactory

  /** Return the implementation of the SymtabFactory interface.  If this
      returns null, the default symbol table entries will be used. */
  public SymtabFactory symtabFactory ()
  {
    return new DefaultSymtabFactory ();
  } // symtabFactory

  /** Return the implementation of the ExprFactory interface.  If this
      returns null, the default expressions will be used. */
  public org.cognitivesystems.idl.constExpr.ExprFactory exprFactory ()
  {
    return new org.cognitivesystems.idl.constExpr.DefaultExprFactory ();
  } // exprFactory

  /** Return a subclass of the Arguments class.  If this returns null,
      the default will be used. */
  public Arguments arguments ()
  {
    return new Arguments ();
  } // arguments

  /** Return the list of keywords in the generated language.
      Note that these keywords may contain the following wildcards:
      <dl>
      <dt>`*'
      <dd>matches zero or more characters
      <dt>`+'
      <dd>matches one or more characters
      <dt>`.'
      <dd>matches any single character
      </dl> */
  public String[] languageKeywords ()
  {
    return null;
  } // languageKeywords
} // interface Factories
