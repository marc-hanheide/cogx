
/* A Bison parser, made by GNU Bison 2.4.1.  */

/* Skeleton implementation for Bison LALR(1) parsers in Java
   
      Copyright (C) 2007, 2008 Free Software Foundation, Inc.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.
   
   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

package de.dfki.lt.tr.dialogue.cplan;
/* First part of user declarations.  */

/* "%code imports" blocks.  */

/* Line 33 of lalr1.java  */
/* Line 3 of "RuleParser.y"  */

import java.io.Reader;
import java.util.List;
import java.util.LinkedList;

import de.dfki.lt.tr.dialogue.cplan.Path;
import de.dfki.lt.tr.dialogue.cplan.matches.*;
import de.dfki.lt.tr.dialogue.cplan.actions.*;



/* Line 33 of lalr1.java  */
/* Line 54 of "RuleParser.java"  */

/**
 * A Bison parser, automatically generated from <tt>RuleParser.y</tt>.
 *
 * @author LALR (1) parser skeleton written by Paolo Bonzini.
 */
public class RuleParser
{
    /** Version number for the Bison executable that generated this parser.  */
  public static final String bisonVersion = "2.4.1";

  /** Name of the skeleton that generated this parser.  */
  public static final String bisonSkeleton = "lalr1.java";


  /** True if verbose error messages are enabled.  */
  public boolean errorVerbose = false;



  /** Token returned by the scanner to signal the end of its input.  */
  public static final int EOF = 0;

/* Tokens.  */
  /** Token number, to be returned by the scanner.  */
  public static final int ID = 258;
  /** Token number, to be returned by the scanner.  */
  public static final int VAR = 259;
  /** Token number, to be returned by the scanner.  */
  public static final int GVAR = 260;
  /** Token number, to be returned by the scanner.  */
  public static final int COMPARISON = 261;
  /** Token number, to be returned by the scanner.  */
  public static final int ARROW = 262;
  /** Token number, to be returned by the scanner.  */
  public static final int STRING = 263;



  

  /**
   * Communication interface between the scanner and the Bison-generated
   * parser <tt>RuleParser</tt>.
   */
  public interface Lexer {
    

    /**
     * Method to retrieve the semantic value of the last scanned token.
     * @return the semantic value of the last scanned token.  */
    Object getLVal ();

    /**
     * Entry point for the scanner.  Returns the token identifier corresponding
     * to the next token and prepares to return the semantic value
     * of the token. 
     * @return the token identifier corresponding to the next token. */
    int yylex () throws java.io.IOException;

    /**
     * Entry point for error reporting.  Emits an error
     * in a user-defined way.
     *
     * 
     * @param s The string for the error message.  */
     void yyerror (String s);
  }

  /** The object doing lexical analysis for us.  */
  private Lexer yylexer;
  
  



  /**
   * Instantiates the Bison-generated parser.
   * @param yylexer The scanner that will supply tokens to the parser.
   */
  public RuleParser (Lexer yylexer) {
    this.yylexer = yylexer;
    
  }

  private java.io.PrintStream yyDebugStream = System.err;

  /**
   * Return the <tt>PrintStream</tt> on which the debugging output is
   * printed.
   */
  public final java.io.PrintStream getDebugStream () { return yyDebugStream; }

  /**
   * Set the <tt>PrintStream</tt> on which the debug output is printed.
   * @param s The stream that is used for debugging output.
   */
  public final void setDebugStream(java.io.PrintStream s) { yyDebugStream = s; }

  private int yydebug = 0;

  /**
   * Answer the verbosity of the debugging output; 0 means that all kinds of
   * output from the parser are suppressed.
   */
  public final int getDebugLevel() { return yydebug; }

  /**
   * Set the verbosity of the debugging output; 0 means that all kinds of
   * output from the parser are suppressed.
   * @param level The verbosity level for debugging output.
   */
  public final void setDebugLevel(int level) { yydebug = level; }

  private final int yylex () throws java.io.IOException {
    return yylexer.yylex ();
  }
  protected final void yyerror (String s) {
    yylexer.yyerror (s);
  }

  

  protected final void yycdebug (String s) {
    if (yydebug > 0)
      yyDebugStream.println (s);
  }

  private final class YYStack {
    private int[] stateStack = new int[16];
    
    private Object[] valueStack = new Object[16];

    public int size = 16;
    public int height = -1;
    
    public final void push (int state, Object value    	   	      	    ) {
      height++;
      if (size == height) 
        {
	  int[] newStateStack = new int[size * 2];
	  System.arraycopy (stateStack, 0, newStateStack, 0, height);
	  stateStack = newStateStack;
	  
	  
	  Object[] newValueStack = new Object[size * 2];
	  System.arraycopy (valueStack, 0, newValueStack, 0, height);
	  valueStack = newValueStack;

	  size *= 2;
	}

      stateStack[height] = state;
      
      valueStack[height] = value;
    }

    public final void pop () {
      height--;
    }

    public final void pop (int num) {
      // Avoid memory leaks... garbage collection is a white lie!
      if (num > 0) {
	java.util.Arrays.fill (valueStack, height - num + 1, height, null);
        
      }
      height -= num;
    }

    public final int stateAt (int i) {
      return stateStack[height - i];
    }

    public final Object valueAt (int i) {
      return valueStack[height - i];
    }

    // Print the state stack on the debug stream.
    public void print (java.io.PrintStream out)
    {
      out.print ("Stack now");
      
      for (int i = 0; i < height; i++)
        {
	  out.print (' ');
	  out.print (stateStack[i]);
        }
      out.println ();
    }
  }

  /**
   * Returned by a Bison action in order to stop the parsing process and
   * return success (<tt>true</tt>).  */
  public static final int YYACCEPT = 0;

  /**
   * Returned by a Bison action in order to stop the parsing process and
   * return failure (<tt>false</tt>).  */
  public static final int YYABORT = 1;

  /**
   * Returned by a Bison action in order to start error recovery without
   * printing an error message.  */
  public static final int YYERROR = 2;

  /**
   * Returned by a Bison action in order to print an error message and start
   * error recovery.  */
  public static final int YYFAIL = 3;

  private static final int YYNEWSTATE = 4;
  private static final int YYDEFAULT = 5;
  private static final int YYREDUCE = 6;
  private static final int YYERRLAB1 = 7;
  private static final int YYRETURN = 8;

  private int yyerrstatus_ = 0;

  /**
   * Return whether error recovery is being done.  In this state, the parser
   * reads token until it reaches a known state, and then restarts normal
   * operation.  */
  public final boolean recovering ()
  {
    return yyerrstatus_ == 0;
  }

  private int yyaction (int yyn, YYStack yystack, int yylen) 
  {
    Object yyval;
    

    /* If YYLEN is nonzero, implement the default value of the action:
       `$$ = $1'.  Otherwise, use the top of the stack.
    
       Otherwise, the following line sets YYVAL to garbage.
       This behavior is undocumented and Bison
       users should not rely upon it.  */
    if (yylen > 0)
      yyval = yystack.valueAt (yylen - 1);
    else
      yyval = yystack.valueAt (0);
    
    yy_reduce_print (yyn, yystack);

    switch (yyn)
      {
	  case 2:
  if (yyn == 2)
    
/* Line 353 of lalr1.java  */
/* Line 85 of "RuleParser.y"  */
    { if ((( Rule )(yystack.valueAt (3-(1)))) != null) _ruleStore.add(0, (( Rule )(yystack.valueAt (3-(1)))));  };
  break;
    

  case 3:
  if (yyn == 3)
    
/* Line 353 of lalr1.java  */
/* Line 86 of "RuleParser.y"  */
    { if ((( Rule )(yystack.valueAt (2-(1)))) != null) _ruleStore.add((( Rule )(yystack.valueAt (2-(1))))); };
  break;
    

  case 4:
  if (yyn == 4)
    
/* Line 353 of lalr1.java  */
/* Line 89 of "RuleParser.y"  */
    { yyval = new Rule((List<VarMatch>)(( List )(yystack.valueAt (3-(1)))), (( List )(yystack.valueAt (3-(3))))); };
  break;
    

  case 5:
  if (yyn == 5)
    
/* Line 353 of lalr1.java  */
/* Line 90 of "RuleParser.y"  */
    { yyval = null; };
  break;
    

  case 6:
  if (yyn == 6)
    
/* Line 353 of lalr1.java  */
/* Line 93 of "RuleParser.y"  */
    { (( List )(yystack.valueAt (2-(2)))).add(0, new VarMatch(null, (( Match )(yystack.valueAt (2-(1)))))); yyval = (( List )(yystack.valueAt (2-(2)))); };
  break;
    

  case 7:
  if (yyn == 7)
    
/* Line 353 of lalr1.java  */
/* Line 96 of "RuleParser.y"  */
    { (( List )(yystack.valueAt (3-(3)))).add(0, (( VarMatch )(yystack.valueAt (3-(2))))); yyval = (( List )(yystack.valueAt (3-(3)))); };
  break;
    

  case 8:
  if (yyn == 8)
    
/* Line 353 of lalr1.java  */
/* Line 97 of "RuleParser.y"  */
    { yyval = new LinkedList<VarMatch>(); };
  break;
    

  case 9:
  if (yyn == 9)
    
/* Line 353 of lalr1.java  */
/* Line 99 of "RuleParser.y"  */
    { yyval = new VarMatch(new GlobalVar((( String )(yystack.valueAt (3-(1))))), (( Match )(yystack.valueAt (3-(3))))); };
  break;
    

  case 10:
  if (yyn == 10)
    
/* Line 353 of lalr1.java  */
/* Line 100 of "RuleParser.y"  */
    { yyval = new VarMatch((( MatchLVal )(yystack.valueAt (3-(1)))), (( Match )(yystack.valueAt (3-(3))))); };
  break;
    

  case 11:
  if (yyn == 11)
    
/* Line 353 of lalr1.java  */
/* Line 103 of "RuleParser.y"  */
    { yyval = new Conjunction((( Match )(yystack.valueAt (3-(1)))), (( Match )(yystack.valueAt (3-(3))))); };
  break;
    

  case 12:
  if (yyn == 12)
    
/* Line 353 of lalr1.java  */
/* Line 104 of "RuleParser.y"  */
    { yyval = new Disjunction((( Match )(yystack.valueAt (3-(1)))), (( Match )(yystack.valueAt (3-(3))))); };
  break;
    

  case 14:
  if (yyn == 14)
    
/* Line 353 of lalr1.java  */
/* Line 109 of "RuleParser.y"  */
    { yyval = new FeatVal((( Match )(yystack.valueAt (4-(2)))), (( Match )(yystack.valueAt (4-(4))))); };
  break;
    

  case 15:
  if (yyn == 15)
    
/* Line 353 of lalr1.java  */
/* Line 110 of "RuleParser.y"  */
    { yyval = new FeatVal((( Match )(yystack.valueAt (3-(2)))), null); };
  break;
    

  case 16:
  if (yyn == 16)
    
/* Line 353 of lalr1.java  */
/* Line 111 of "RuleParser.y"  */
    { yyval = (( Match )(yystack.valueAt (1-(1)))); };
  break;
    

  case 17:
  if (yyn == 17)
    
/* Line 353 of lalr1.java  */
/* Line 114 of "RuleParser.y"  */
    { yyval = (( Match )(yystack.valueAt (1-(1)))); };
  break;
    

  case 18:
  if (yyn == 18)
    
/* Line 353 of lalr1.java  */
/* Line 115 of "RuleParser.y"  */
    { yyval = new Conjunction((( Match )(yystack.valueAt (2-(1)))),
                                   new FeatVal(DagNode.TYPE_FEAT_ID, (( Match )(yystack.valueAt (2-(2)))))); };
  break;
    

  case 19:
  if (yyn == 19)
    
/* Line 353 of lalr1.java  */
/* Line 117 of "RuleParser.y"  */
    { yyval = new FeatVal(DagNode.TYPE_FEAT_ID, (( Match )(yystack.valueAt (2-(2))))); };
  break;
    

  case 20:
  if (yyn == 20)
    
/* Line 353 of lalr1.java  */
/* Line 118 of "RuleParser.y"  */
    { yyval = new FeatVal(DagNode.PROP_FEAT_ID, (( Match )(yystack.valueAt (1-(1))))); };
  break;
    

  case 21:
  if (yyn == 21)
    
/* Line 353 of lalr1.java  */
/* Line 119 of "RuleParser.y"  */
    { yyval = new Negation((( Match )(yystack.valueAt (2-(2))))); };
  break;
    

  case 22:
  if (yyn == 22)
    
/* Line 353 of lalr1.java  */
/* Line 120 of "RuleParser.y"  */
    { yyval = (( Match )(yystack.valueAt (3-(2)))); };
  break;
    

  case 23:
  if (yyn == 23)
    
/* Line 353 of lalr1.java  */
/* Line 126 of "RuleParser.y"  */
    { yyval = new FeatVal(DagNode.ID_FEAT_ID, new Atom((( String )(yystack.valueAt (2-(1)))))); };
  break;
    

  case 24:
  if (yyn == 24)
    
/* Line 353 of lalr1.java  */
/* Line 127 of "RuleParser.y"  */
    { yyval = new LocalVar((( String )(yystack.valueAt (2-(1)))));  };
  break;
    

  case 25:
  if (yyn == 25)
    
/* Line 353 of lalr1.java  */
/* Line 128 of "RuleParser.y"  */
    { yyval = new GlobalVar((( String )(yystack.valueAt (2-(1))))); };
  break;
    

  case 26:
  if (yyn == 26)
    
/* Line 353 of lalr1.java  */
/* Line 131 of "RuleParser.y"  */
    { yyval = new LocalVar((( String )(yystack.valueAt (1-(1))))); };
  break;
    

  case 27:
  if (yyn == 27)
    
/* Line 353 of lalr1.java  */
/* Line 132 of "RuleParser.y"  */
    { yyval = new Atom((( String )(yystack.valueAt (1-(1))))); };
  break;
    

  case 28:
  if (yyn == 28)
    
/* Line 353 of lalr1.java  */
/* Line 134 of "RuleParser.y"  */
    { yyval = getNewFunCall((( String )(yystack.valueAt (4-(1)))), (( List )(yystack.valueAt (4-(3)))));
                             if (yyval == null) return YYERROR ;
                           };
  break;
    

  case 29:
  if (yyn == 29)
    
/* Line 353 of lalr1.java  */
/* Line 137 of "RuleParser.y"  */
    { yyval = getNewFunCall((( String )(yystack.valueAt (3-(1)))), null);
                             if (yyval == null) return YYERROR ;
                           };
  break;
    

  case 30:
  if (yyn == 30)
    
/* Line 353 of lalr1.java  */
/* Line 159 of "RuleParser.y"  */
    {
            List<Action> result  = new LinkedList<Action>();
            result.add((( Action )(yystack.valueAt (1-(1)))));
            yyval = result;
          };
  break;
    

  case 31:
  if (yyn == 31)
    
/* Line 353 of lalr1.java  */
/* Line 164 of "RuleParser.y"  */
    { (( List )(yystack.valueAt (3-(3)))).add(0, (( Action )(yystack.valueAt (3-(1))))); yyval = (( List )(yystack.valueAt (3-(3)))); };
  break;
    

  case 32:
  if (yyn == 32)
    
/* Line 353 of lalr1.java  */
/* Line 168 of "RuleParser.y"  */
    {
         DagNode rval = (((( DagNode )(yystack.valueAt (4-(4)))) != null) ? (( DagNode )(yystack.valueAt (4-(4)))).copyResult(false) : null);
         DagNode.invalidate();
         yyval = new Assignment((( VarDagNode )(yystack.valueAt (4-(1)))), (( Path )(yystack.valueAt (4-(2)))), rval);
       };
  break;
    

  case 33:
  if (yyn == 33)
    
/* Line 353 of lalr1.java  */
/* Line 174 of "RuleParser.y"  */
    {
         DagNode rval = (((( DagNode )(yystack.valueAt (4-(4)))) != null) ? (( DagNode )(yystack.valueAt (4-(4)))).copyResult(false) : null);
         DagNode.invalidate();
         yyval = new Addition((( VarDagNode )(yystack.valueAt (4-(1)))), (( Path )(yystack.valueAt (4-(2)))), rval);
       };
  break;
    

  case 34:
  if (yyn == 34)
    
/* Line 353 of lalr1.java  */
/* Line 182 of "RuleParser.y"  */
    { yyval = new Deletion((( VarDagNode )(yystack.valueAt (6-(1)))), (( Path )(yystack.valueAt (6-(2)))), new DagNode((( String )(yystack.valueAt (6-(5)))), new DagNode())); };
  break;
    

  case 35:
  if (yyn == 35)
    
/* Line 353 of lalr1.java  */
/* Line 185 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.LOCAL); };
  break;
    

  case 36:
  if (yyn == 36)
    
/* Line 353 of lalr1.java  */
/* Line 186 of "RuleParser.y"  */
    { yyval = new VarDagNode("#", Bindings.LOCAL); };
  break;
    

  case 37:
  if (yyn == 37)
    
/* Line 353 of lalr1.java  */
/* Line 187 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.GLOBAL); };
  break;
    

  case 38:
  if (yyn == 38)
    
/* Line 353 of lalr1.java  */
/* Line 191 of "RuleParser.y"  */
    { yyval = (( Path )(yystack.valueAt (4-(4)))).addToFront((( String )(yystack.valueAt (4-(2))))); };
  break;
    

  case 39:
  if (yyn == 39)
    
/* Line 353 of lalr1.java  */
/* Line 192 of "RuleParser.y"  */
    { yyval = new Path(); };
  break;
    

  case 40:
  if (yyn == 40)
    
/* Line 353 of lalr1.java  */
/* Line 195 of "RuleParser.y"  */
    { (( DagNode )(yystack.valueAt (3-(1)))).add((( DagNode )(yystack.valueAt (3-(3))))); (( DagNode )(yystack.valueAt (3-(1)))).setNominal(); yyval = (( DagNode )(yystack.valueAt (3-(1)))); };
  break;
    

  case 41:
  if (yyn == 41)
    
/* Line 353 of lalr1.java  */
/* Line 196 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (1-(1)))); };
  break;
    

  case 42:
  if (yyn == 42)
    
/* Line 353 of lalr1.java  */
/* Line 199 of "RuleParser.y"  */
    { yyval = new DagNode((( String )(yystack.valueAt (4-(2)))), (( DagNode )(yystack.valueAt (4-(4))))).setNominal(); };
  break;
    

  case 43:
  if (yyn == 43)
    
/* Line 353 of lalr1.java  */
/* Line 200 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (1-(1)))); };
  break;
    

  case 44:
  if (yyn == 44)
    
/* Line 353 of lalr1.java  */
/* Line 203 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (1-(1)))); };
  break;
    

  case 45:
  if (yyn == 45)
    
/* Line 353 of lalr1.java  */
/* Line 204 of "RuleParser.y"  */
    { (( DagNode )(yystack.valueAt (2-(1)))).add(new DagNode(DagNode.TYPE_FEAT_ID, (( DagNode )(yystack.valueAt (2-(2))))));
                            yyval = (( DagNode )(yystack.valueAt (2-(1)))); };
  break;
    

  case 46:
  if (yyn == 46)
    
/* Line 353 of lalr1.java  */
/* Line 206 of "RuleParser.y"  */
    { yyval = new DagNode(DagNode.TYPE_FEAT_ID, (( DagNode )(yystack.valueAt (2-(2)))))
                                    .setNominal();
                          };
  break;
    

  case 47:
  if (yyn == 47)
    
/* Line 353 of lalr1.java  */
/* Line 209 of "RuleParser.y"  */
    { yyval = new DagNode(DagNode.PROP_FEAT_ID, (( DagNode )(yystack.valueAt (1-(1))))); };
  break;
    

  case 48:
  if (yyn == 48)
    
/* Line 353 of lalr1.java  */
/* Line 210 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (3-(2)))).setNominal(); };
  break;
    

  case 49:
  if (yyn == 49)
    
/* Line 353 of lalr1.java  */
/* Line 211 of "RuleParser.y"  */
    { yyval = getNewFunCallDagNode((( String )(yystack.valueAt (4-(1)))), (( List )(yystack.valueAt (4-(3)))));
                            if (yyval == null) return YYERROR ;
                          };
  break;
    

  case 50:
  if (yyn == 50)
    
/* Line 353 of lalr1.java  */
/* Line 214 of "RuleParser.y"  */
    { yyval = getNewFunCallDagNode((( String )(yystack.valueAt (3-(1)))), null);
                            if (yyval == null) return YYERROR ;
                          };
  break;
    

  case 51:
  if (yyn == 51)
    
/* Line 353 of lalr1.java  */
/* Line 219 of "RuleParser.y"  */
    {
             yyval = new DagNode(DagNode.ID_FEAT_ID, (( DagNode )(yystack.valueAt (2-(1))))).setNominal();
         };
  break;
    

  case 52:
  if (yyn == 52)
    
/* Line 353 of lalr1.java  */
/* Line 224 of "RuleParser.y"  */
    { (( List )(yystack.valueAt (3-(3)))).add(0, (( DagNode )(yystack.valueAt (3-(1))))); yyval = (( List )(yystack.valueAt (3-(3)))); };
  break;
    

  case 53:
  if (yyn == 53)
    
/* Line 353 of lalr1.java  */
/* Line 225 of "RuleParser.y"  */
    { List<DagNode> result = new LinkedList<DagNode>();
                           result.add((( DagNode )(yystack.valueAt (1-(1)))));
                           yyval = result;
                         };
  break;
    

  case 54:
  if (yyn == 54)
    
/* Line 353 of lalr1.java  */
/* Line 231 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (1-(1)))); };
  break;
    

  case 55:
  if (yyn == 55)
    
/* Line 353 of lalr1.java  */
/* Line 232 of "RuleParser.y"  */
    { yyval = new DagNode((( String )(yystack.valueAt (1-(1))))); };
  break;
    

  case 56:
  if (yyn == 56)
    
/* Line 353 of lalr1.java  */
/* Line 233 of "RuleParser.y"  */
    { yyval = new VarDagNode("#", Bindings.LOCAL); };
  break;
    

  case 57:
  if (yyn == 57)
    
/* Line 353 of lalr1.java  */
/* Line 236 of "RuleParser.y"  */
    { yyval = new DagNode((( String )(yystack.valueAt (1-(1))))); };
  break;
    

  case 58:
  if (yyn == 58)
    
/* Line 353 of lalr1.java  */
/* Line 237 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.LOCAL); };
  break;
    

  case 59:
  if (yyn == 59)
    
/* Line 353 of lalr1.java  */
/* Line 238 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.GLOBAL); };
  break;
    



/* Line 353 of lalr1.java  */
/* Line 849 of "RuleParser.java"  */
	default: break;
      }

    yy_symbol_print ("-> $$ =", yyr1_[yyn], yyval);

    yystack.pop (yylen);
    yylen = 0;

    /* Shift the result of the reduction.  */
    yyn = yyr1_[yyn];
    int yystate = yypgoto_[yyn - yyntokens_] + yystack.stateAt (0);
    if (0 <= yystate && yystate <= yylast_
	&& yycheck_[yystate] == yystack.stateAt (0))
      yystate = yytable_[yystate];
    else
      yystate = yydefgoto_[yyn - yyntokens_];

    yystack.push (yystate, yyval);
    return YYNEWSTATE;
  }

  /* Return YYSTR after stripping away unnecessary quotes and
     backslashes, so that it's suitable for yyerror.  The heuristic is
     that double-quoting is unnecessary unless the string contains an
     apostrophe, a comma, or backslash (other than backslash-backslash).
     YYSTR is taken from yytname.  */
  private final String yytnamerr_ (String yystr)
  {
    if (yystr.charAt (0) == '"')
      {
        StringBuffer yyr = new StringBuffer ();
        strip_quotes: for (int i = 1; i < yystr.length (); i++)
          switch (yystr.charAt (i))
            {
            case '\'':
            case ',':
              break strip_quotes;

            case '\\':
	      if (yystr.charAt(++i) != '\\')
                break strip_quotes;
              /* Fall through.  */
            default:
              yyr.append (yystr.charAt (i));
              break;

            case '"':
              return yyr.toString ();
            }
      }
    else if (yystr.equals ("$end"))
      return "end of input";

    return yystr;
  }

  /*--------------------------------.
  | Print this symbol on YYOUTPUT.  |
  `--------------------------------*/

  private void yy_symbol_print (String s, int yytype,
			         Object yyvaluep				 )
  {
    if (yydebug > 0)
    yycdebug (s + (yytype < yyntokens_ ? " token " : " nterm ")
	      + yytname_[yytype] + " ("
	      + (yyvaluep == null ? "(null)" : yyvaluep.toString ()) + ")");
  }

  /**
   * Parse input from the scanner that was specified at object construction
   * time.  Return whether the end of the input was reached successfully.
   *
   * @return <tt>true</tt> if the parsing succeeds.  Note that this does not
   *          imply that there were no syntax errors.
   */
  public boolean parse () throws java.io.IOException
  {
    /// Lookahead and lookahead in internal form.
    int yychar = yyempty_;
    int yytoken = 0;

    /* State.  */
    int yyn = 0;
    int yylen = 0;
    int yystate = 0;

    YYStack yystack = new YYStack ();

    /* Error handling.  */
    int yynerrs_ = 0;
    

    /// Semantic value of the lookahead.
    Object yylval = null;

    int yyresult;

    yycdebug ("Starting parse\n");
    yyerrstatus_ = 0;


    /* Initialize the stack.  */
    yystack.push (yystate, yylval);

    int label = YYNEWSTATE;
    for (;;)
      switch (label)
      {
        /* New state.  Unlike in the C/C++ skeletons, the state is already
	   pushed when we come here.  */
      case YYNEWSTATE:
        yycdebug ("Entering state " + yystate + "\n");
        if (yydebug > 0)
          yystack.print (yyDebugStream);
    
        /* Accept?  */
        if (yystate == yyfinal_)
          return true;
    
        /* Take a decision.  First try without lookahead.  */
        yyn = yypact_[yystate];
        if (yyn == yypact_ninf_)
          {
            label = YYDEFAULT;
	    break;
          }
    
        /* Read a lookahead token.  */
        if (yychar == yyempty_)
          {
	    yycdebug ("Reading a token: ");
	    yychar = yylex ();
            
            yylval = yylexer.getLVal ();
          }
    
        /* Convert token to internal form.  */
        if (yychar <= EOF)
          {
	    yychar = yytoken = EOF;
	    yycdebug ("Now at end of input.\n");
          }
        else
          {
	    yytoken = yytranslate_ (yychar);
	    yy_symbol_print ("Next token is", yytoken,
	    		     yylval);
          }
    
        /* If the proper action on seeing token YYTOKEN is to reduce or to
           detect an error, take that action.  */
        yyn += yytoken;
        if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yytoken)
          label = YYDEFAULT;
    
        /* <= 0 means reduce or error.  */
        else if ((yyn = yytable_[yyn]) <= 0)
          {
	    if (yyn == 0 || yyn == yytable_ninf_)
	      label = YYFAIL;
	    else
	      {
	        yyn = -yyn;
	        label = YYREDUCE;
	      }
          }
    
        else
          {
            /* Shift the lookahead token.  */
	    yy_symbol_print ("Shifting", yytoken,
	    		     yylval);
    
            /* Discard the token being shifted.  */
            yychar = yyempty_;
    
            /* Count tokens shifted since error; after three, turn off error
               status.  */
            if (yyerrstatus_ > 0)
              --yyerrstatus_;
    
            yystate = yyn;
            yystack.push (yystate, yylval);
            label = YYNEWSTATE;
          }
        break;
    
      /*-----------------------------------------------------------.
      | yydefault -- do the default action for the current state.  |
      `-----------------------------------------------------------*/
      case YYDEFAULT:
        yyn = yydefact_[yystate];
        if (yyn == 0)
          label = YYFAIL;
        else
          label = YYREDUCE;
        break;
    
      /*-----------------------------.
      | yyreduce -- Do a reduction.  |
      `-----------------------------*/
      case YYREDUCE:
        yylen = yyr2_[yyn];
        label = yyaction (yyn, yystack, yylen);
	yystate = yystack.stateAt (0);
        break;
    
      /*------------------------------------.
      | yyerrlab -- here on detecting error |
      `------------------------------------*/
      case YYFAIL:
        /* If not already recovering from an error, report this error.  */
        if (yyerrstatus_ == 0)
          {
	    ++yynerrs_;
	    yyerror (yysyntax_error (yystate, yytoken));
          }
    
        
        if (yyerrstatus_ == 3)
          {
	    /* If just tried and failed to reuse lookahead token after an
	     error, discard it.  */
    
	    if (yychar <= EOF)
	      {
	      /* Return failure if at end of input.  */
	      if (yychar == EOF)
	        return false;
	      }
	    else
	      yychar = yyempty_;
          }
    
        /* Else will try to reuse lookahead token after shifting the error
           token.  */
        label = YYERRLAB1;
        break;
    
      /*---------------------------------------------------.
      | errorlab -- error raised explicitly by YYERROR.  |
      `---------------------------------------------------*/
      case YYERROR:
    
        
        /* Do not reclaim the symbols of the rule which action triggered
           this YYERROR.  */
        yystack.pop (yylen);
        yylen = 0;
        yystate = yystack.stateAt (0);
        label = YYERRLAB1;
        break;
    
      /*-------------------------------------------------------------.
      | yyerrlab1 -- common code for both syntax error and YYERROR.  |
      `-------------------------------------------------------------*/
      case YYERRLAB1:
        yyerrstatus_ = 3;	/* Each real token shifted decrements this.  */
    
        for (;;)
          {
	    yyn = yypact_[yystate];
	    if (yyn != yypact_ninf_)
	      {
	        yyn += yyterror_;
	        if (0 <= yyn && yyn <= yylast_ && yycheck_[yyn] == yyterror_)
	          {
	            yyn = yytable_[yyn];
	            if (0 < yyn)
		      break;
	          }
	      }
    
	    /* Pop the current state because it cannot handle the error token.  */
	    if (yystack.height == 1)
	      return false;
    
	    
	    yystack.pop ();
	    yystate = yystack.stateAt (0);
	    if (yydebug > 0)
	      yystack.print (yyDebugStream);
          }
    
	

        /* Shift the error token.  */
        yy_symbol_print ("Shifting", yystos_[yyn],
			 yylval);
    
        yystate = yyn;
	yystack.push (yyn, yylval);
        label = YYNEWSTATE;
        break;
    
        /* Accept.  */
      case YYACCEPT:
        return true;
    
        /* Abort.  */
      case YYABORT:
        return false;
      }
  }

  // Generate an error message.
  private String yysyntax_error (int yystate, int tok)
  {
    if (errorVerbose)
      {
        int yyn = yypact_[yystate];
        if (yypact_ninf_ < yyn && yyn <= yylast_)
          {
	    StringBuffer res;

	    /* Start YYX at -YYN if negative to avoid negative indexes in
	       YYCHECK.  */
	    int yyxbegin = yyn < 0 ? -yyn : 0;

	    /* Stay within bounds of both yycheck and yytname.  */
	    int yychecklim = yylast_ - yyn + 1;
	    int yyxend = yychecklim < yyntokens_ ? yychecklim : yyntokens_;
	    int count = 0;
	    for (int x = yyxbegin; x < yyxend; ++x)
	      if (yycheck_[x + yyn] == x && x != yyterror_)
	        ++count;

	    // FIXME: This method of building the message is not compatible
	    // with internationalization.
	    res = new StringBuffer ("syntax error, unexpected ");
	    res.append (yytnamerr_ (yytname_[tok]));
	    if (count < 5)
	      {
	        count = 0;
	        for (int x = yyxbegin; x < yyxend; ++x)
	          if (yycheck_[x + yyn] == x && x != yyterror_)
		    {
		      res.append (count++ == 0 ? ", expecting " : " or ");
		      res.append (yytnamerr_ (yytname_[x]));
		    }
	      }
	    return res.toString ();
          }
      }

    return "syntax error";
  }


  /* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
     STATE-NUM.  */
  private static final byte yypact_ninf_ = -61;
  private static final byte yypact_[] =
  {
        55,   -61,   -12,     4,    18,    11,    11,    60,    60,     7,
      38,    45,    51,    33,   -61,    11,   -61,   -61,   -61,   -61,
     -61,   -61,    48,   -61,   -61,    49,   -61,     5,    37,    24,
     -61,    60,    60,   -61,    60,   -61,   -61,   -61,   -61,   -61,
     -61,    59,    61,    69,    82,    51,    84,   -61,   -61,   -61,
      37,    93,    78,     8,    60,   -61,    60,   -61,    73,    75,
      85,    75,   -61,   -61,   -61,   -61,   -61,   -61,    81,    90,
     -61,   -61,   -61,    61,    86,    98,    80,    75,   -61,    91,
     -61,    80,    89,   102,   -61,   -61,    46,   -61,    35,    92,
     -61,    94,    75,   -61,   -61,    95,   -61,   -61,    96,    75,
     -61,   -61,   -61,   -61,   -61
  };

  /* YYDEFACT[S] -- default rule to reduce with in state S when YYTABLE
     doesn't specify something else to do.  Zero means the default is an
     error.  */
  private static final byte yydefact_[] =
  {
         0,     5,    27,    26,     0,     0,     0,     0,     0,     0,
       0,     0,     8,    13,    16,    17,    20,    23,    24,    25,
      27,    26,     0,    19,    21,     0,     1,     0,     0,     0,
       6,     0,     0,    18,    15,    22,     2,    35,    37,    36,
       4,    30,    39,     0,     0,     8,     0,    11,    12,    14,
       0,     0,     0,     0,     0,     7,     0,    31,     0,     0,
       0,     0,    57,    58,    59,    55,    29,    56,     0,    53,
      54,     9,    10,    39,    57,     0,     0,     0,    33,    41,
      43,    44,    47,     0,    32,    28,     0,    38,     0,     0,
      46,     0,     0,    45,    51,     0,    52,    50,     0,     0,
      48,    40,    34,    49,    42
  };

  /* YYPGOTO[NTERM-NUM].  */
  private static final byte yypgoto_[] =
  {
       -61,    83,   -61,   -61,    62,   -61,    -8,    -3,   -61,   -61,
      76,   -61,    58,   -61,   -61,    40,   -60,    12,   -61,   -61,
     -52,   -61,   -51
  };

  /* YYDEFGOTO[NTERM-NUM].  */
  private static final byte
  yydefgoto_[] =
  {
        -1,     9,    10,    11,    30,    45,    12,    13,    14,    15,
      16,    46,    40,    41,    42,    52,    78,    79,    80,    81,
      68,    69,    82
  };

  /* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
     positive, shift that token.  If negative, reduce the rule which
     number is the opposite.  If zero, do what YYDEFACT says.  */
  private static final byte yytable_ninf_ = -4;
  private static final byte
  yytable_[] =
  {
        25,    84,    70,    17,    24,    -3,     1,    26,     2,     3,
       4,    62,    63,    64,    20,    21,    65,    91,     5,    18,
       6,     7,     8,    47,    48,    90,    66,    43,    67,    44,
      93,    49,   101,    19,    96,    70,    98,    70,    62,    63,
      64,    37,    38,    65,    31,    32,    71,    27,    72,    62,
      63,    64,    28,    97,    65,    67,     1,    39,     2,     3,
       4,    29,    34,     2,     3,     4,    67,    35,     5,    50,
       6,     7,     8,     5,    51,     6,     7,     8,    74,    63,
      64,    22,    23,    62,    63,    64,    53,    73,    75,    59,
      76,    33,    77,    54,    60,    56,    58,    61,    83,    85,
      86,    89,    92,    88,    94,    95,    99,    55,    57,   102,
      36,   104,   100,    87,   103
  };

  /* YYCHECK.  */
  private static final byte
  yycheck_[] =
  {
         8,    61,    53,    15,     7,     0,     1,     0,     3,     4,
       5,     3,     4,     5,     3,     4,     8,    77,    13,    15,
      15,    16,    17,    31,    32,    76,    18,     3,    20,     5,
      81,    34,    92,    15,    86,    86,    88,    88,     3,     4,
       5,     4,     5,     8,    11,    12,    54,     9,    56,     3,
       4,     5,     7,    18,     8,    20,     1,    20,     3,     4,
       5,    10,    14,     3,     4,     5,    20,    18,    13,    10,
      15,    16,    17,    13,    13,    15,    16,    17,     3,     4,
       5,     5,     6,     3,     4,     5,    17,    14,    13,    11,
      15,    15,    17,    11,    16,    11,     3,    19,    13,    18,
      10,     3,    11,    17,    15,     3,    14,    45,    50,    14,
      27,    99,    18,    73,    18
  };

  /* STOS_[STATE-NUM] -- The (internal number of the) accessing
     symbol of state STATE-NUM.  */
  private static final byte
  yystos_[] =
  {
         0,     1,     3,     4,     5,    13,    15,    16,    17,    22,
      23,    24,    27,    28,    29,    30,    31,    15,    15,    15,
       3,     4,    31,    31,    28,    27,     0,     9,     7,    10,
      25,    11,    12,    31,    14,    18,    22,     4,     5,    20,
      33,    34,    35,     3,     5,    26,    32,    27,    27,    28,
      10,    13,    36,    17,    11,    25,    11,    33,     3,    11,
      16,    19,     3,     4,     5,     8,    18,    20,    41,    42,
      43,    27,    27,    14,     3,    13,    15,    17,    37,    38,
      39,    40,    43,    13,    37,    18,    10,    36,    17,     3,
      43,    37,    11,    43,    15,     3,    41,    18,    41,    14,
      18,    37,    14,    18,    38
  };

  /* TOKEN_NUMBER_[YYLEX-NUM] -- Internal symbol number corresponding
     to YYLEX-NUM.  */
  private static final short
  yytoken_number_[] =
  {
         0,   256,   264,   258,   259,   260,   261,   262,   263,    46,
      44,    94,   124,    60,    62,    58,    33,    40,    41,    61,
      35
  };

  /* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
  private static final byte
  yyr1_[] =
  {
         0,    21,    22,    22,    23,    23,    24,    25,    25,    26,
      26,    27,    27,    27,    28,    28,    28,    29,    29,    29,
      29,    29,    29,    30,    30,    30,    31,    31,    32,    32,
      33,    33,    34,    34,    34,    35,    35,    35,    36,    36,
      37,    37,    38,    38,    39,    39,    39,    39,    39,    39,
      39,    40,    41,    41,    42,    42,    42,    43,    43,    43
  };

  /* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
  private static final byte
  yyr2_[] =
  {
         0,     2,     3,     2,     3,     1,     2,     3,     0,     3,
       3,     3,     3,     1,     4,     3,     1,     1,     2,     2,
       1,     2,     3,     2,     2,     2,     1,     1,     4,     3,
       1,     3,     4,     4,     6,     1,     1,     1,     4,     0,
       3,     1,     4,     1,     1,     2,     2,     1,     3,     4,
       3,     2,     3,     1,     1,     1,     1,     1,     1,     1
  };

  /* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
     First, the terminals, then, starting at \a yyntokens_, nonterminals.  */
  private static final String yytname_[] =
  {
    "$end", "error", "$undefined", "ID", "VAR", "GVAR", "COMPARISON",
  "ARROW", "STRING", "'.'", "','", "'^'", "'|'", "'<'", "'>'", "':'",
  "'!'", "'('", "')'", "'='", "'#'", "$accept", "rules", "rule", "matches",
  "gmatches", "gmatch", "expr", "term", "feature", "nominal", "id_lvar",
  "funcall", "actions", "action", "lval", "path", "rexpr", "rterm",
  "rfeat", "rnominal", "rargs", "rarg", "r_id_var", null
  };

  /* YYRHS -- A `-1'-separated list of the rules' RHS.  */
  private static final byte yyrhs_[] =
  {
        22,     0,    -1,    23,     9,    22,    -1,    23,     9,    -1,
      24,     7,    33,    -1,     1,    -1,    27,    25,    -1,    10,
      26,    25,    -1,    -1,     5,    11,    27,    -1,    32,    11,
      27,    -1,    28,    11,    27,    -1,    28,    12,    27,    -1,
      28,    -1,    13,    31,    14,    28,    -1,    13,    31,    14,
      -1,    29,    -1,    30,    -1,    30,    31,    -1,    15,    31,
      -1,    31,    -1,    16,    28,    -1,    17,    27,    18,    -1,
       3,    15,    -1,     4,    15,    -1,     5,    15,    -1,     4,
      -1,     3,    -1,     3,    17,    41,    18,    -1,     3,    17,
      18,    -1,    34,    -1,    34,    10,    33,    -1,    35,    36,
      19,    37,    -1,    35,    36,    11,    37,    -1,    35,    36,
      16,    13,     3,    14,    -1,     4,    -1,    20,    -1,     5,
      -1,    13,     3,    14,    36,    -1,    -1,    38,    11,    37,
      -1,    38,    -1,    13,     3,    14,    38,    -1,    39,    -1,
      40,    -1,    40,    43,    -1,    15,    43,    -1,    43,    -1,
      17,    37,    18,    -1,     3,    17,    41,    18,    -1,     3,
      17,    18,    -1,    43,    15,    -1,    42,    10,    41,    -1,
      42,    -1,    43,    -1,     8,    -1,    20,    -1,     3,    -1,
       4,    -1,     5,    -1
  };

  /* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
     YYRHS.  */
  private static final short yyprhs_[] =
  {
         0,     0,     3,     7,    10,    14,    16,    19,    23,    24,
      28,    32,    36,    40,    42,    47,    51,    53,    55,    58,
      61,    63,    66,    70,    73,    76,    79,    81,    83,    88,
      92,    94,    98,   103,   108,   115,   117,   119,   121,   126,
     127,   131,   133,   138,   140,   142,   145,   148,   150,   154,
     159,   163,   166,   170,   172,   174,   176,   178,   180,   182
  };

  /* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
  private static final short yyrline_[] =
  {
         0,    85,    85,    86,    89,    90,    93,    96,    97,    99,
     100,   103,   104,   106,   109,   110,   111,   114,   115,   117,
     118,   119,   120,   126,   127,   128,   131,   132,   134,   137,
     159,   164,   167,   173,   181,   185,   186,   187,   191,   192,
     195,   196,   199,   200,   203,   204,   206,   209,   210,   211,
     214,   219,   224,   225,   231,   232,   233,   236,   237,   238
  };

  // Report on the debug stream that the rule yyrule is going to be reduced.
  private void yy_reduce_print (int yyrule, YYStack yystack)
  {
    if (yydebug == 0)
      return;

    int yylno = yyrline_[yyrule];
    int yynrhs = yyr2_[yyrule];
    /* Print the symbols being reduced, and their result.  */
    yycdebug ("Reducing stack by rule " + (yyrule - 1)
	      + " (line " + yylno + "), ");

    /* The symbols being reduced.  */
    for (int yyi = 0; yyi < yynrhs; yyi++)
      yy_symbol_print ("   $" + (yyi + 1) + " =",
		       yyrhs_[yyprhs_[yyrule] + yyi],
		       ((yystack.valueAt (yynrhs-(yyi + 1)))));
  }

  /* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
  private static final byte yytranslate_table_[] =
  {
         0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    16,     2,    20,     2,     2,     2,     2,
      17,    18,     2,     2,    10,     2,     9,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,    15,     2,
      13,    19,    14,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,    11,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,    12,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     2
  };

  private static final byte yytranslate_ (int t)
  {
    if (t >= 0 && t <= yyuser_token_number_max_)
      return yytranslate_table_[t];
    else
      return yyundef_token_;
  }

  private static final int yylast_ = 114;
  private static final int yynnts_ = 23;
  private static final int yyempty_ = -2;
  private static final int yyfinal_ = 26;
  private static final int yyterror_ = 1;
  private static final int yyerrcode_ = 256;
  private static final int yyntokens_ = 21;

  private static final int yyuser_token_number_max_ = 264;
  private static final int yyundef_token_ = 2;

/* User implementation code.  */
/* Unqualified %code blocks.  */

/* Line 875 of lalr1.java  */
/* Line 21 of "RuleParser.y"  */

  private List<Rule> _ruleStore;

  public List<Rule> getRules() {
    return _ruleStore;
  }

  public void reset() {
    _ruleStore = new LinkedList<Rule>();
  }

  public void reset(String inputDescription, Reader input) {
    reset();
    ((de.dfki.lt.tr.dialogue.cplan.Lexer)this.yylexer)
      .setInputReader(inputDescription, input);
  }

  private FunCall getNewFunCall(String name, List args) {
    try {
      return new FunCall(name, args);
    }
    catch (NoSuchMethodException ex) {
      yyerror("No such Function registered: " + ex.getMessage());
    }
    return null;
  }

  private FunCallDagNode getNewFunCallDagNode(String name, List args) {
    try {
      return new FunCallDagNode(name, args);
    }
    catch (NoSuchMethodException ex) {
      yyerror("No such Function registered: " + ex.getMessage());
    }
    return null;
  }



/* Line 875 of lalr1.java  */
/* Line 1518 of "RuleParser.java"  */

}


