
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
/* Line 64 of "RuleParser.y"  */
    { if ((( Rule )(yystack.valueAt (3-(1)))) != null) _ruleStore.add(0, (( Rule )(yystack.valueAt (3-(1)))));  };
  break;
    

  case 3:
  if (yyn == 3)
    
/* Line 353 of lalr1.java  */
/* Line 65 of "RuleParser.y"  */
    { if ((( Rule )(yystack.valueAt (2-(1)))) != null) _ruleStore.add((( Rule )(yystack.valueAt (2-(1))))); };
  break;
    

  case 4:
  if (yyn == 4)
    
/* Line 353 of lalr1.java  */
/* Line 68 of "RuleParser.y"  */
    { yyval = new Rule((List<VarMatch>)(( List )(yystack.valueAt (3-(1)))), (( List )(yystack.valueAt (3-(3))))); };
  break;
    

  case 5:
  if (yyn == 5)
    
/* Line 353 of lalr1.java  */
/* Line 69 of "RuleParser.y"  */
    { yyval = null; };
  break;
    

  case 6:
  if (yyn == 6)
    
/* Line 353 of lalr1.java  */
/* Line 72 of "RuleParser.y"  */
    { (( List )(yystack.valueAt (2-(2)))).add(0, new VarMatch(null, (( Match )(yystack.valueAt (2-(1)))))); yyval = (( List )(yystack.valueAt (2-(2)))); };
  break;
    

  case 7:
  if (yyn == 7)
    
/* Line 353 of lalr1.java  */
/* Line 75 of "RuleParser.y"  */
    { (( List )(yystack.valueAt (5-(5)))).add(0, new VarMatch((( String )(yystack.valueAt (5-(2)))), (( Match )(yystack.valueAt (5-(4))))));
                                        yyval = (( List )(yystack.valueAt (5-(5))));
                                      };
  break;
    

  case 8:
  if (yyn == 8)
    
/* Line 353 of lalr1.java  */
/* Line 78 of "RuleParser.y"  */
    { yyval = new LinkedList<VarMatch>(); };
  break;
    

  case 9:
  if (yyn == 9)
    
/* Line 353 of lalr1.java  */
/* Line 81 of "RuleParser.y"  */
    { yyval = new Conjunction((( Match )(yystack.valueAt (3-(1)))), (( Match )(yystack.valueAt (3-(3))))); };
  break;
    

  case 10:
  if (yyn == 10)
    
/* Line 353 of lalr1.java  */
/* Line 82 of "RuleParser.y"  */
    { yyval = new Disjunction((( Match )(yystack.valueAt (3-(1)))), (( Match )(yystack.valueAt (3-(3))))); };
  break;
    

  case 12:
  if (yyn == 12)
    
/* Line 353 of lalr1.java  */
/* Line 87 of "RuleParser.y"  */
    { yyval = new FeatVal((( Match )(yystack.valueAt (4-(2)))), (( Match )(yystack.valueAt (4-(4))))); };
  break;
    

  case 13:
  if (yyn == 13)
    
/* Line 353 of lalr1.java  */
/* Line 88 of "RuleParser.y"  */
    { yyval = new FeatVal((( Match )(yystack.valueAt (3-(2)))), null); };
  break;
    

  case 14:
  if (yyn == 14)
    
/* Line 353 of lalr1.java  */
/* Line 89 of "RuleParser.y"  */
    { yyval = (( Match )(yystack.valueAt (1-(1)))); };
  break;
    

  case 15:
  if (yyn == 15)
    
/* Line 353 of lalr1.java  */
/* Line 92 of "RuleParser.y"  */
    { yyval = (( Match )(yystack.valueAt (1-(1)))); };
  break;
    

  case 16:
  if (yyn == 16)
    
/* Line 353 of lalr1.java  */
/* Line 93 of "RuleParser.y"  */
    { yyval = new Conjunction((( Match )(yystack.valueAt (2-(1)))),
                                   new FeatVal(DagNode.TYPE_FEAT_ID, (( Match )(yystack.valueAt (2-(2)))))); };
  break;
    

  case 17:
  if (yyn == 17)
    
/* Line 353 of lalr1.java  */
/* Line 95 of "RuleParser.y"  */
    { yyval = new FeatVal(DagNode.TYPE_FEAT_ID, (( Match )(yystack.valueAt (2-(2))))); };
  break;
    

  case 18:
  if (yyn == 18)
    
/* Line 353 of lalr1.java  */
/* Line 97 of "RuleParser.y"  */
    { yyval = new FeatVal(DagNode.PROP_FEAT_ID, (( Match )(yystack.valueAt (1-(1))))); };
  break;
    

  case 19:
  if (yyn == 19)
    
/* Line 353 of lalr1.java  */
/* Line 98 of "RuleParser.y"  */
    { yyval = new Negation((( Match )(yystack.valueAt (2-(2))))); };
  break;
    

  case 20:
  if (yyn == 20)
    
/* Line 353 of lalr1.java  */
/* Line 99 of "RuleParser.y"  */
    { yyval = (( Match )(yystack.valueAt (3-(2)))); };
  break;
    

  case 21:
  if (yyn == 21)
    
/* Line 353 of lalr1.java  */
/* Line 100 of "RuleParser.y"  */
    { yyval = new FunCall((( String )(yystack.valueAt (4-(1)))), (( List )(yystack.valueAt (4-(3))))); };
  break;
    

  case 22:
  if (yyn == 22)
    
/* Line 353 of lalr1.java  */
/* Line 103 of "RuleParser.y"  */
    { yyval = new FeatVal(DagNode.ID_FEAT_ID, new Atom((( String )(yystack.valueAt (2-(1)))))); };
  break;
    

  case 23:
  if (yyn == 23)
    
/* Line 353 of lalr1.java  */
/* Line 104 of "RuleParser.y"  */
    { yyval = new LocalVar((( String )(yystack.valueAt (2-(1)))));  };
  break;
    

  case 24:
  if (yyn == 24)
    
/* Line 353 of lalr1.java  */
/* Line 108 of "RuleParser.y"  */
    { yyval = new GlobalVar((( String )(yystack.valueAt (2-(1))))); };
  break;
    

  case 25:
  if (yyn == 25)
    
/* Line 353 of lalr1.java  */
/* Line 111 of "RuleParser.y"  */
    { yyval = new LocalVar((( String )(yystack.valueAt (1-(1))))); };
  break;
    

  case 26:
  if (yyn == 26)
    
/* Line 353 of lalr1.java  */
/* Line 112 of "RuleParser.y"  */
    { yyval = new Atom((( String )(yystack.valueAt (1-(1))))); };
  break;
    

  case 27:
  if (yyn == 27)
    
/* Line 353 of lalr1.java  */
/* Line 114 of "RuleParser.y"  */
    { yyval = (( List )(yystack.valueAt (3-(3)))).add((( Match )(yystack.valueAt (3-(1))))); };
  break;
    

  case 28:
  if (yyn == 28)
    
/* Line 353 of lalr1.java  */
/* Line 115 of "RuleParser.y"  */
    { yyval = (( List )(yystack.valueAt (3-(3)))).add((( String )(yystack.valueAt (3-(1))))); };
  break;
    

  case 29:
  if (yyn == 29)
    
/* Line 353 of lalr1.java  */
/* Line 118 of "RuleParser.y"  */
    { yyval = new LinkedList(); };
  break;
    

  case 30:
  if (yyn == 30)
    
/* Line 353 of lalr1.java  */
/* Line 127 of "RuleParser.y"  */
    {
            List<Action> result  = new LinkedList<Action>();
            result.add((( Action )(yystack.valueAt (1-(1)))));
            yyval = result;
          };
  break;
    

  case 31:
  if (yyn == 31)
    
/* Line 353 of lalr1.java  */
/* Line 132 of "RuleParser.y"  */
    { (( List )(yystack.valueAt (3-(3)))).add(0, (( Action )(yystack.valueAt (3-(1))))); yyval = (( List )(yystack.valueAt (3-(3)))); };
  break;
    

  case 32:
  if (yyn == 32)
    
/* Line 353 of lalr1.java  */
/* Line 136 of "RuleParser.y"  */
    {
         DagNode rval = (( DagNode )(yystack.valueAt (4-(4)))).copyResult(false);
         DagNode.invalidate();
         yyval = new Assignment((( VarDagNode )(yystack.valueAt (4-(1)))), (( Path )(yystack.valueAt (4-(2)))), rval);
       };
  break;
    

  case 33:
  if (yyn == 33)
    
/* Line 353 of lalr1.java  */
/* Line 142 of "RuleParser.y"  */
    {
         DagNode rval = (( DagNode )(yystack.valueAt (4-(4)))).copyResult(false);
         DagNode.invalidate();
         yyval = new Addition((( VarDagNode )(yystack.valueAt (4-(1)))), (( Path )(yystack.valueAt (4-(2)))), rval);
       };
  break;
    

  case 34:
  if (yyn == 34)
    
/* Line 353 of lalr1.java  */
/* Line 150 of "RuleParser.y"  */
    { yyval = new Deletion((( VarDagNode )(yystack.valueAt (6-(1)))), (( Path )(yystack.valueAt (6-(2)))), new DagNode((( String )(yystack.valueAt (6-(5)))), new DagNode())); };
  break;
    

  case 35:
  if (yyn == 35)
    
/* Line 353 of lalr1.java  */
/* Line 153 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.LOCAL); };
  break;
    

  case 36:
  if (yyn == 36)
    
/* Line 353 of lalr1.java  */
/* Line 154 of "RuleParser.y"  */
    { yyval = new VarDagNode(null, Bindings.LOCAL); };
  break;
    

  case 37:
  if (yyn == 37)
    
/* Line 353 of lalr1.java  */
/* Line 155 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.GLOBAL); };
  break;
    

  case 38:
  if (yyn == 38)
    
/* Line 353 of lalr1.java  */
/* Line 159 of "RuleParser.y"  */
    { yyval = (( Path )(yystack.valueAt (4-(4)))).addToFront((( String )(yystack.valueAt (4-(2))))); };
  break;
    

  case 39:
  if (yyn == 39)
    
/* Line 353 of lalr1.java  */
/* Line 160 of "RuleParser.y"  */
    { yyval = new Path(); };
  break;
    

  case 40:
  if (yyn == 40)
    
/* Line 353 of lalr1.java  */
/* Line 163 of "RuleParser.y"  */
    { (( DagNode )(yystack.valueAt (3-(1)))).add((( DagNode )(yystack.valueAt (3-(3))))); (( DagNode )(yystack.valueAt (3-(1)))).setNominal(); yyval = (( DagNode )(yystack.valueAt (3-(1)))); };
  break;
    

  case 41:
  if (yyn == 41)
    
/* Line 353 of lalr1.java  */
/* Line 164 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (1-(1)))); };
  break;
    

  case 42:
  if (yyn == 42)
    
/* Line 353 of lalr1.java  */
/* Line 167 of "RuleParser.y"  */
    { yyval = new DagNode((( String )(yystack.valueAt (4-(2)))), (( DagNode )(yystack.valueAt (4-(4))))).setNominal(); };
  break;
    

  case 43:
  if (yyn == 43)
    
/* Line 353 of lalr1.java  */
/* Line 168 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (1-(1)))); };
  break;
    

  case 44:
  if (yyn == 44)
    
/* Line 353 of lalr1.java  */
/* Line 171 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (1-(1)))); };
  break;
    

  case 45:
  if (yyn == 45)
    
/* Line 353 of lalr1.java  */
/* Line 172 of "RuleParser.y"  */
    { (( DagNode )(yystack.valueAt (2-(1)))).add(new DagNode(DagNode.TYPE_FEAT_ID, (( DagNode )(yystack.valueAt (2-(2))))));
                            yyval = (( DagNode )(yystack.valueAt (2-(1)))); };
  break;
    

  case 46:
  if (yyn == 46)
    
/* Line 353 of lalr1.java  */
/* Line 174 of "RuleParser.y"  */
    { yyval = new DagNode(DagNode.TYPE_FEAT_ID, (( DagNode )(yystack.valueAt (2-(2)))))
                                    .setNominal();
                          };
  break;
    

  case 47:
  if (yyn == 47)
    
/* Line 353 of lalr1.java  */
/* Line 177 of "RuleParser.y"  */
    { yyval = new DagNode(DagNode.PROP_FEAT_ID, (( DagNode )(yystack.valueAt (1-(1))))); };
  break;
    

  case 48:
  if (yyn == 48)
    
/* Line 353 of lalr1.java  */
/* Line 178 of "RuleParser.y"  */
    { yyval = (( DagNode )(yystack.valueAt (3-(2)))).setNominal(); };
  break;
    

  case 49:
  if (yyn == 49)
    
/* Line 353 of lalr1.java  */
/* Line 179 of "RuleParser.y"  */
    { yyval = new FunCallDagNode((( String )(yystack.valueAt (4-(1)))), (( List )(yystack.valueAt (4-(3))))); };
  break;
    

  case 50:
  if (yyn == 50)
    
/* Line 353 of lalr1.java  */
/* Line 182 of "RuleParser.y"  */
    {
             yyval = new DagNode(DagNode.ID_FEAT_ID, (( DagNode )(yystack.valueAt (2-(1))))).setNominal();
         };
  break;
    

  case 51:
  if (yyn == 51)
    
/* Line 353 of lalr1.java  */
/* Line 187 of "RuleParser.y"  */
    { yyval = new DagNode((( String )(yystack.valueAt (1-(1))))); };
  break;
    

  case 52:
  if (yyn == 52)
    
/* Line 353 of lalr1.java  */
/* Line 188 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.LOCAL); };
  break;
    

  case 53:
  if (yyn == 53)
    
/* Line 353 of lalr1.java  */
/* Line 189 of "RuleParser.y"  */
    { yyval = new VarDagNode((( String )(yystack.valueAt (1-(1)))), Bindings.GLOBAL); };
  break;
    



/* Line 353 of lalr1.java  */
/* Line 786 of "RuleParser.java"  */
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
  private static final byte yypact_ninf_ = -65;
  private static final byte yypact_[] =
  {
        35,   -65,    29,     7,    28,    27,    27,    56,    56,    41,
      38,    42,    46,    66,   -65,    27,   -65,   -65,    50,   -65,
     -65,   -65,   -65,    48,   -65,   -65,    52,   -65,    10,     1,
      59,   -65,    56,    56,   -65,    58,    69,    62,    56,   -65,
     -65,   -65,   -65,   -65,   -65,    72,    68,    74,   -65,   -65,
      50,    50,   -65,   -65,     1,    80,    26,    56,   -65,   -65,
     -65,    73,    71,    76,    71,    46,    68,    75,   -65,   -65,
      87,    13,    71,   -65,    82,   -65,    13,    79,    88,   -65,
     -65,   -65,    50,    81,   -65,   -65,    78,    71,   -65,   -65,
      83,    84,    71,   -65,   -65,   -65,   -65,   -65
  };

  /* YYDEFACT[S] -- default rule to reduce with in state S when YYTABLE
     doesn't specify something else to do.  Zero means the default is an
     error.  */
  private static final byte yydefact_[] =
  {
         0,     5,    26,    25,     0,     0,     0,     0,     0,     0,
       0,     0,     8,    11,    14,    15,    18,    22,    29,    23,
      24,    26,    25,     0,    17,    19,     0,     1,     0,     0,
       0,     6,     0,     0,    16,     0,     0,     0,    13,    20,
       2,    35,    37,    36,     4,    30,    39,     0,     9,    10,
      29,    29,    21,    12,     0,     0,     0,     0,    28,    27,
      31,     0,     0,     0,     0,     8,    39,    51,    52,    53,
       0,     0,     0,    33,    41,    43,    44,    47,     0,    32,
       7,    38,    29,     0,    51,    46,     0,     0,    45,    50,
       0,     0,     0,    48,    40,    34,    49,    42
  };

  /* YYPGOTO[NTERM-NUM].  */
  private static final byte yypgoto_[] =
  {
       -65,    70,   -65,   -65,    34,     0,    -3,   -65,   -65,    14,
     -48,    47,   -65,   -65,    37,   -63,     8,   -65,   -65,   -64
  };

  /* YYDEFGOTO[NTERM-NUM].  */
  private static final byte
  yydefgoto_[] =
  {
        -1,     9,    10,    11,    31,    36,    13,    14,    15,    16,
      37,    44,    45,    46,    56,    73,    74,    75,    76,    77
  };

  /* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
     positive, shift that token.  If negative, reduce the rule which
     number is the opposite.  If zero, do what YYDEFACT says.  */
  private static final byte yytable_ninf_ = -4;
  private static final byte
  yytable_[] =
  {
        12,    79,    58,    59,    25,    41,    42,    85,    26,    86,
      -3,     1,    88,     2,     3,     4,    84,    68,    69,    23,
      24,    43,    19,     5,    94,     6,     7,     8,    12,    34,
      21,    22,    48,    49,    91,    53,     1,    62,     2,     3,
       4,    27,    63,    20,    17,    64,    18,    28,     5,    29,
       6,     7,     8,     2,     3,     4,    30,    65,    35,     2,
       3,     4,    38,     5,    47,     6,     7,     8,    50,     5,
      39,     6,     7,     8,    67,    68,    69,    32,    33,    51,
      52,    55,    54,    61,    70,    57,    71,    66,    72,    78,
      83,    90,    82,    87,    89,    92,    93,    95,    40,    80,
      97,    60,    96,    81
  };

  /* YYCHECK.  */
  private static final byte
  yycheck_[] =
  {
         0,    64,    50,    51,     7,     4,     5,    71,     8,    72,
       0,     1,    76,     3,     4,     5,     3,     4,     5,     5,
       6,    20,    15,    13,    87,    15,    16,    17,    28,    15,
       3,     4,    32,    33,    82,    38,     1,    11,     3,     4,
       5,     0,    16,    15,    15,    19,    17,     9,    13,     7,
      15,    16,    17,     3,     4,     5,    10,    57,     8,     3,
       4,     5,    14,    13,     5,    15,    16,    17,    10,    13,
      18,    15,    16,    17,     3,     4,     5,    11,    12,    10,
      18,    13,    10,     3,    13,    11,    15,    14,    17,    13,
       3,     3,    17,    11,    15,    14,    18,    14,    28,    65,
      92,    54,    18,    66
  };

  /* STOS_[STATE-NUM] -- The (internal number of the) accessing
     symbol of state STATE-NUM.  */
  private static final byte
  yystos_[] =
  {
         0,     1,     3,     4,     5,    13,    15,    16,    17,    22,
      23,    24,    26,    27,    28,    29,    30,    15,    17,    15,
      15,     3,     4,    30,    30,    27,    26,     0,     9,     7,
      10,    25,    11,    12,    30,     8,    26,    31,    14,    18,
      22,     4,     5,    20,    32,    33,    34,     5,    26,    26,
      10,    10,    18,    27,    10,    13,    35,    11,    31,    31,
      32,     3,    11,    16,    19,    26,    14,     3,     4,     5,
      13,    15,    17,    36,    37,    38,    39,    40,    13,    36,
      25,    35,    17,     3,     3,    40,    36,    11,    40,    15,
       3,    31,    14,    18,    36,    14,    18,    37
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
      26,    26,    27,    27,    27,    28,    28,    28,    28,    28,
      28,    28,    29,    29,    29,    30,    30,    31,    31,    31,
      32,    32,    33,    33,    33,    34,    34,    34,    35,    35,
      36,    36,    37,    37,    38,    38,    38,    38,    38,    38,
      39,    40,    40,    40
  };

  /* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
  private static final byte
  yyr2_[] =
  {
         0,     2,     3,     2,     3,     1,     2,     5,     0,     3,
       3,     1,     4,     3,     1,     1,     2,     2,     1,     2,
       3,     4,     2,     2,     2,     1,     1,     3,     3,     0,
       1,     3,     4,     4,     6,     1,     1,     1,     4,     0,
       3,     1,     4,     1,     1,     2,     2,     1,     3,     4,
       2,     1,     1,     1
  };

  /* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
     First, the terminals, then, starting at \a yyntokens_, nonterminals.  */
  private static final String yytname_[] =
  {
    "$end", "error", "$undefined", "ID", "VAR", "GVAR", "COMPARISON",
  "ARROW", "STRING", "'.'", "','", "'^'", "'|'", "'<'", "'>'", "':'",
  "'!'", "'('", "')'", "'='", "'#'", "$accept", "rules", "rule", "matches",
  "gmatches", "expr", "term", "feature", "nominal", "id_lvar", "args",
  "actions", "action", "lval", "path", "rexpr", "rterm", "rfeat",
  "rnominal", "r_id_var", null
  };

  /* YYRHS -- A `-1'-separated list of the rules' RHS.  */
  private static final byte yyrhs_[] =
  {
        22,     0,    -1,    23,     9,    22,    -1,    23,     9,    -1,
      24,     7,    32,    -1,     1,    -1,    26,    25,    -1,    10,
       5,    11,    26,    25,    -1,    -1,    27,    11,    26,    -1,
      27,    12,    26,    -1,    27,    -1,    13,    30,    14,    27,
      -1,    13,    30,    14,    -1,    28,    -1,    29,    -1,    29,
      30,    -1,    15,    30,    -1,    30,    -1,    16,    27,    -1,
      17,    26,    18,    -1,     3,    17,    31,    18,    -1,     3,
      15,    -1,     4,    15,    -1,     5,    15,    -1,     4,    -1,
       3,    -1,    26,    10,    31,    -1,     8,    10,    31,    -1,
      -1,    33,    -1,    33,    10,    32,    -1,    34,    35,    19,
      36,    -1,    34,    35,    11,    36,    -1,    34,    35,    16,
      13,     3,    14,    -1,     4,    -1,    20,    -1,     5,    -1,
      13,     3,    14,    35,    -1,    -1,    37,    11,    36,    -1,
      37,    -1,    13,     3,    14,    37,    -1,    38,    -1,    39,
      -1,    39,    40,    -1,    15,    40,    -1,    40,    -1,    17,
      36,    18,    -1,     3,    17,    31,    18,    -1,    40,    15,
      -1,     3,    -1,     4,    -1,     5,    -1
  };

  /* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
     YYRHS.  */
  private static final short yyprhs_[] =
  {
         0,     0,     3,     7,    10,    14,    16,    19,    25,    26,
      30,    34,    36,    41,    45,    47,    49,    52,    55,    57,
      60,    64,    69,    72,    75,    78,    80,    82,    86,    90,
      91,    93,    97,   102,   107,   114,   116,   118,   120,   125,
     126,   130,   132,   137,   139,   141,   144,   147,   149,   153,
     158,   161,   163,   165
  };

  /* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
  private static final short yyrline_[] =
  {
         0,    64,    64,    65,    68,    69,    72,    75,    78,    81,
      82,    84,    87,    88,    89,    92,    93,    95,    97,    98,
      99,   100,   103,   104,   108,   111,   112,   114,   115,   118,
     127,   132,   135,   141,   149,   153,   154,   155,   159,   160,
     163,   164,   167,   168,   171,   172,   174,   177,   178,   179,
     182,   187,   188,   189
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

  private static final int yylast_ = 103;
  private static final int yynnts_ = 20;
  private static final int yyempty_ = -2;
  private static final int yyfinal_ = 27;
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




/* Line 875 of lalr1.java  */
/* Line 1427 of "RuleParser.java"  */

}


