/* -*- Mode: Java -*- */

%code imports {
import java.io.Reader;
import java.util.List;
import java.util.LinkedList;

import de.dfki.lt.tr.dialogue.cplan.Path;
import de.dfki.lt.tr.dialogue.cplan.matches.*;
import de.dfki.lt.tr.dialogue.cplan.actions.*;
}

%language "Java"

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

}

%token < String >  ID         258
%token < String >  VAR        259
%token < String >  GVAR       260
%token < String >  COMPARISON 261
%token < String >  ARROW      262
%token < String >  STRING     263

%type < Match > expr term feature nominal id_lvar
%type < DagNode > rexpr rterm rfeat r_id_var rnominal
%type < Path > path

%type < VarDagNode > lval

%type < Action > action

%type < Rule > rule

%type < List > rules matches gmatches actions

%type < List > args

%%


rules : rule '.' rules   { if ($1 != null) _ruleStore.add(0, $1);  }
      | rule '.'         { if ($1 != null) _ruleStore.add($1); }
      ;

rule : matches ARROW actions  { $$ = new Rule((List<VarMatch>)$1, $3); }
     | error                  { $$ = null; }
     ;

matches : expr gmatches  { $2.add(0, new VarMatch(null, $1)); $$ = $2; }
        ;

gmatches : ',' GVAR '^' expr gmatches { $5.add(0, new VarMatch($2, $4));
                                        $$ = $5;
                                      }
         |                            { $$ = new LinkedList<VarMatch>(); }
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
        | nominal id_lvar { $$ = new Conjunction($1,
                                   new FeatVal(DagNode.TYPE_FEAT_ID, $2)); }
        | ':' id_lvar     { $$ = new FeatVal(DagNode.TYPE_FEAT_ID, $2); }
        // proposition
        | id_lvar         { $$ = new FeatVal(DagNode.PROP_FEAT_ID, $1); }
        | '!' term        { $$ = new Negation($2); }
        | '(' expr ')'    { $$ = $2; }
        | ID '(' args ')' { $$ = new FunCall($1, $3); }
        ;

nominal : ID ':'     { $$ = new FeatVal(DagNode.ID_FEAT_ID, new Atom($1)); }
        | VAR ':'    { $$ = new LocalVar($1);  }
          // Create a MatchNode that will redirect all matching to this global
          // var. That means that global variables can only be used sensibly
          // at the root level of an expression
        | GVAR ':'        { $$ = new GlobalVar($1); }
        ;

id_lvar : VAR      { $$ = new LocalVar($1); }
        | ID       { $$ = new Atom($1); }

args : expr ',' args   { $$ = $3.add($1); }
     | STRING ',' args { $$ = $3.add($1); }
     //| ID ',' args     { $$ = $3.add($1); }
     // already covered by expr, but not in the expected way: FIXME
     |                 { $$ = new LinkedList(); }
     ;



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
         DagNode rval = $4.copyResult(false);
         DagNode.invalidate();
         $$ = new Assignment($1, $2, rval);
       }
       | lval path '^' rexpr
       {
         DagNode rval = $4.copyResult(false);
         DagNode.invalidate();
         $$ = new Addition($1, $2, rval);
       }
// THAT DOES NOT SUFFICE! IT MIGHT BE NICE TO SPECIFY REXPRS TO DELETE E.G.
// THE TYPE AND PROP AND SOME FEATURES IN ONE SWEEP, BUT KEEP THE REST
       | lval path '!' '<' ID  '>'
       { $$ = new Deletion($1, $2, new DagNode($5, new DagNode())); }
       ;

lval : VAR       { $$ = new VarDagNode($1, Bindings.LOCAL); }
     | '#'       { $$ = new VarDagNode(null, Bindings.LOCAL); }
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
      | ID '(' args ')'   { $$ = new FunCallDagNode($1, $3); }
      ;

rnominal : r_id_var ':' {
             $$ = new DagNode(DagNode.ID_FEAT_ID, $1).setNominal();
         }
         ;

r_id_var : ID     { $$ = new DagNode($1); }
         | VAR    { $$ = new VarDagNode($1, Bindings.LOCAL); }
         | GVAR   { $$ = new VarDagNode($1, Bindings.GLOBAL); }
         ;
