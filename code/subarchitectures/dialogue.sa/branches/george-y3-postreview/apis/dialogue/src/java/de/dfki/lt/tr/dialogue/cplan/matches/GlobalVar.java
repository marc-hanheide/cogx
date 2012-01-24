package de.dfki.lt.tr.dialogue.cplan.matches;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;

/** A syntax tree node representing the occurrence of a global variable */
public class GlobalVar extends Match implements MatchLVal {
  private String _varName;

  public GlobalVar(String varName) {
    _varName = varName;
  }

  @Override
  public String toString() {
    return (_negated ? "!" : "") + "##" + _varName;
  }

  /** This constitutes a new global match ?? Is there a scenario where it would
   *  be meaningful to check the contents of a global var against the
   *  current (maybe embedded) node AND the rest of the match expression,
   *  instead of testing only the global var against the rest? Why not?
   *  To distinguish, could we use the assignment syntax? Or a custom
   *  globalMatch function?
   */
  @Override
  public boolean match(DagEdge input, Bindings bindings) {
    DagEdge lf = bindings.getBinding(_varName, Bindings.GLOBAL);
    if (lf == null) {
      return false;
    }
    throw new UnsupportedOperationException("Not implemented yet");
    // TODO THIS IS WRONG: THE LF IN THE GLOBAL VAR MUST BE MATCHED AGAINST
    // THE REST OF THE ASSOCIATED CONJUNCT SYNTREE
    // return lf.unifiable(input);
  }

  /** Establish a binding between this global variable and the given value */
  public void bind(DagEdge value, Bindings bindings) {
    bindings.bind(_varName, value, Bindings.GLOBAL);
  }

  /** Return the binding associated with this global variable, if there is any
   */
  @Override
  public DagEdge getBinding(DagEdge input, Bindings bindings) {
    return bindings.getBinding(_varName, Bindings.GLOBAL);
  }
}
