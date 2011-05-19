/* -*- Mode: Java -*- */

%code imports {
import java.io.Reader;
import java.util.HashMap;

@SuppressWarnings({"fallthrough", "unused"})
}

%language "Java"

%define package "de.dfki.lt.tr.dialogue.cplan"

%define public

%define parser_class_name "LFParser"

%code {
  private DagNode _lf;

  private HashMap<String, DagNode> _nodes = new HashMap<String, DagNode>();

  private DagNode newLF(String feature, String type) {
    return new DagNode(feature, new DagNode(type));
  }

  private DagNode newLF(String feature, DagNode value) {
    return new DagNode(feature, value);
  }

  private DagNode newLF(short feature, String type) {
    return new DagNode(feature, new DagNode(type));
  }

  private DagNode getNewLF(String id) {
    DagNode lf = _nodes.get(id);
    if (lf == null) {
      lf = newLF(DagNode.ID_FEAT_ID, id);
      _nodes.put(id, lf);
    }
    return lf;
  }

  /** unify two conjunctions */
  private DagNode unify(DagNode left, DagNode right) {
    left.add(right);
    return left;
  }

  public DagNode getResultLF() {
    return _lf.copyResult(false);
  }

  public void reset() {
    _lf = null;
    _nodes.clear();
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

%type < DagNode > lf lfconj lfterm

%type < Object > start

// %type < Object > rexpr rterm rnominal

%%

start : lf               { $$ = null; _lf = $1; }
      ;

// plain logical forms

lf : '@' ID ':' ID '(' lfconj ')' {
     $$ = unify(getNewLF($2).setNominal(),
                unify(newLF(DagNode.TYPE_FEAT_ID, $4), $6));
   }
   | '(' lfconj ')'   { $$ = $2.setNominal(); }
   ;

lfconj : lfterm '^' lfconj { $$ = unify($1, $3); }
       | lfterm            { $$ = $1; }
       ;

lfterm : '<' ID '>' '(' lfconj ')' { $$ = newLF($2, $5).setNominal(); }
       | '<' ID '>' ID  { $$ = newLF($2, newLF(DagNode.PROP_FEAT_ID, $4))
                                    .setNominal();
                        }
       | '<' ID '>' ID ':' ID
                        { $$ = newLF($2,
                                     unify(getNewLF($4).setNominal(),
                                           newLF(DagNode.TYPE_FEAT_ID, $6)))
                            .setNominal();
                        }
       | ID ':' ID      { $$ = unify(getNewLF($1).setNominal(),
                                     newLF(DagNode.TYPE_FEAT_ID, $3));
                        }
       | ID ':'         { $$ = getNewLF($1).setNominal(); }
       | ':' ID         { $$ = newLF(DagNode.TYPE_FEAT_ID, $2).setNominal(); }
       | ID             { $$ = newLF(DagNode.PROP_FEAT_ID, $1); }
       ;

