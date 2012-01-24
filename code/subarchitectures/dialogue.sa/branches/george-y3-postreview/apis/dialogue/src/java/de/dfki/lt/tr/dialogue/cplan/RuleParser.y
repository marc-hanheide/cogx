/* -*- Mode: Java -*- */

%code imports {
import java.io.Reader;
import java.util.List;
import java.util.LinkedList;

import de.dfki.lt.tr.dialogue.cplan.Path;
import de.dfki.lt.tr.dialogue.cplan.util.Position;
import de.dfki.lt.tr.dialogue.cplan.matches.*;
import de.dfki.lt.tr.dialogue.cplan.actions.*;

 @SuppressWarnings({"unchecked", "fallthrough", "unused"})
}

%language "Java"

%locations

%define package "de.dfki.lt.tr.dialogue.cplan"

%define public

%define parser_class_name "RuleParser"

%code {
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

  private Rule newRule(List matches, List actions, Location loc) {
    return new Rule((List<VarMatch>) matches, (List<Action>) actions,
                    loc);
  }

  private FunCall getNewFunCall(String name, List args, Location loc) {
    try {
      return new FunCall(name, args);
    }
    catch (NoSuchMethodException ex) {
      yyerror(loc, "No such Function registered: " + ex.getMessage());
    }
    return null;
  }

  private FunCallDagNode getNewFunCallDagNode(String name, List args,
                                              Location loc) {
    try {
      return new FunCallDagNode(name, args);
    }
    catch (NoSuchMethodException ex) {
      yyerror(loc, "No such Function registered: " + ex.getMessage());
    }
    return null;
  }
}

%token < String >  ID         258
%token < String >  VAR        259
%token < String >  GVAR       260
%token < String >  COMPARISON 261
%token < String >  ARROW      262
%token < String >  STRING     263

%type < Match > expr term feature nominal id_lvar iv_expr iv_term
%type < DagNode > rexpr rterm rfeat r_id_var rnominal rarg
%type < Path > path

%type < VarDagNode > lval

%type < Action > action

%type < Rule > rule

%type < List > rules matches gmatches actions rargs

%type < VarMatch > gmatch

%type < MatchLVal > funcall

%%


rules : rule '.' rules   { if ($1 != null) _ruleStore.add(0, $1);  }
      | rule '.'         { if ($1 != null) _ruleStore.add($1); }
      ;

rule : matches ARROW actions  { $$ = newRule($1, $3, @2); }
     | error                  { $$ = null; }
     ;

matches : expr gmatches  { $2.add(0, new VarMatch(null, $1)); $$ = $2; }
        ;

gmatches : ',' gmatch gmatches { $3.add(0, $2); $$ = $3; }
         |                     { $$ = new LinkedList<VarMatch>(); }

gmatch : GVAR '^' expr         { $$ = new VarMatch(new GlobalVar($1), $3); }
       | funcall '^' expr      { $$ = new VarMatch($1, $3); }
       ;

expr : term '^' expr     { $$ = new Conjunction($1, $3); }
     | term '|' expr     { $$ = new Disjunction($1, $3); }
     // | term COMPARISON expr // general boolean expressions: later
     | term
     ;

term : '<' id_lvar '>' term     { $$ = new FeatVal($2, $4); }
     | '<' id_lvar '>'          { $$ = new FeatVal($2, null); }
     | feature                  { $$ = $1; }
     ;

feature : nominal         { $$ = $1; }
        | nominal iv_term { $$ = new Conjunction($1,
                                   new FeatVal(DagNode.TYPE_FEAT_ID, $2)); }
        | ':' iv_term     { $$ = new FeatVal(DagNode.TYPE_FEAT_ID, $2); }
        | id_lvar         { $$ = new FeatVal(DagNode.PROP_FEAT_ID, $1); }
        | '!' term        { $2.setNegated(true); $$ = $2; }
        | '(' expr ')'    { $$ = $2; }
// Not needed anymore, the matching against function return values has all the
// functionality that is possible here
//        | funcall         { $$ = $1; }
        ;

nominal : ID ':'     { $$ = new FeatVal(DagNode.ID_FEAT_ID, new Atom($1)); }
        | VAR ':'    { $$ = new LocalVar($1);  }
        | GVAR ':'   { $$ = new GlobalVar($1); }
        ;

id_lvar : VAR      { $$ = new LocalVar($1); }
        | ID       { $$ = new Atom($1); }

iv_term : id_lvar           { $$ = $1; }
        | '!' iv_term       { $2.setNegated(true); $$ = $2; }
        | '(' iv_expr ')'   { $$ = $2; }
        ;

iv_expr : iv_term               { $$ = $1; }
        | iv_term '|' iv_expr   { $$ = new Disjunction($1, $3); }
        ;

funcall : ID '(' rargs ')' { $$ = getNewFunCall($1, $3, @1);
                             if ($$ == null) return YYERROR ;
                           }
        | ID '(' ')'       { $$ = getNewFunCall($1, null, @1);
                             if ($$ == null) return YYERROR ;
                           }
        ;

/*
args : arg ',' args   { $3.add(0, $1); $$ = $3; }
     | arg            { List<Match> result = new LinkedList<Match>();
                        result.add($1);
                        $$ = result;
                      }
     ;

arg  : id_lvar        { $$ = $1; }
     | STRING         { $$ = new Atom($1); }
     ;
*/

// Now for the right hand sides of the rules:
// no negation/alternative/comparison, but global var assignment and
// replacement vs. conjunction/addition

actions : action {
            List<Action> result  = new LinkedList<Action>();
            result.add($1);
            $$ = result;
          }
        | action ',' actions  { $3.add(0, $1); $$ = $3; }
        ;

action : lval path '=' rexpr
       {
         DagNode rval = (($4 != null) ? $4.copyResult(false) : null);
         DagNode.invalidate();
         $$ = new Assignment($1, $2, rval);
       }
       | lval path '^' rexpr
       {
         DagNode rval = (($4 != null) ? $4.copyResult(false) : null);
         DagNode.invalidate();
         $$ = new Addition($1, $2, rval);
       }
// THAT DOES NOT SUFFICE! IT MIGHT BE NICE TO SPECIFY REXPRS TO DELETE E.G.
// THE TYPE AND PROP AND SOME FEATURES IN ONE SWEEP, BUT KEEP THE REST
       | lval path '!' '<' ID  '>'
       { $$ = new Deletion($1, $2, new DagNode($5, new DagNode())); }
       ;

lval : VAR       { $$ = new VarDagNode($1, Bindings.LOCAL); }
     | '#'       { $$ = new VarDagNode("#", Bindings.LOCAL); }
     | GVAR      { $$ = new VarDagNode($1, Bindings.GLOBAL); }
//   | ID ':'    { $$ = new VarDagNode($1, Bindings.ABSOLUTE); }
     ;

path : '<' ID '>' path { $$ = $4.addToFront($2); }
     |                 { $$ = new Path(); }
     ;

rexpr  : rterm '^' rexpr  { $1.add($3); $1.setNominal(); $$ = $1; }
       | rterm            { $$ = $1; }
       ;

rterm : '<' ID '>' rterm  { $$ = new DagNode($2, $4).setNominal(); }
      | rfeat             { $$ = $1; }
      ;

rfeat : rnominal          { $$ = $1; }
      | rnominal r_id_var { $1.add(new DagNode(DagNode.TYPE_FEAT_ID, $2));
                            $$ = $1; }
      | ':' r_id_var      { $$ = new DagNode(DagNode.TYPE_FEAT_ID, $2)
                                    .setNominal();
                          }
      | r_id_var          { $$ = new DagNode(DagNode.PROP_FEAT_ID, $1); }
      | '(' rexpr ')'     { $$ = $2.setNominal(); }
      // | ID '(' rargs ')'  { $$ = getNewFunCallDagNode($1, $3, @1);
      //                       if ($$ == null) return YYERROR ;
      //                     }
      // | ID '(' ')'        { $$ = getNewFunCallDagNode($1, null, @1);
      //                       if ($$ == null) return YYERROR ;
      //                     }
      ;

rnominal : r_id_var ':' {
             $$ = new DagNode(DagNode.ID_FEAT_ID, $1).setNominal();
         }
         ;

rargs : rarg ',' rargs   { $3.add(0, $1); $$ = $3; }
      | rarg             { List<DagNode> result = new LinkedList<DagNode>();
                           result.add($1);
                           $$ = result;
                         }
      ;

rarg  : r_id_var  { $$ = $1; }
      | STRING    { $$ = new DagNode($1); }
      | '#'       { $$ = new VarDagNode("#", Bindings.LOCAL); }
      ;

r_id_var : ID     { $$ = new DagNode($1); }
         | VAR    { $$ = new VarDagNode($1, Bindings.LOCAL); }
         | GVAR   { $$ = new VarDagNode($1, Bindings.GLOBAL); }
         | ID '(' rargs ')'  { $$ = getNewFunCallDagNode($1, $3, @1);
                               if ($$ == null) return YYERROR ;
                             }
         | ID '(' ')'        { $$ = getNewFunCallDagNode($1, null, @1);
                               if ($$ == null) return YYERROR ;
                             }
         ;
