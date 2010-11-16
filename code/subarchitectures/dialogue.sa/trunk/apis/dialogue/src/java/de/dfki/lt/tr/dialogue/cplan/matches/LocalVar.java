package de.dfki.lt.tr.dialogue.cplan.matches;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;

/** A syntax tree node representing the occurence of a local variable */
public class LocalVar extends Match implements MatchLVal {
  private String _varName;

  public LocalVar(String varName) {
    _varName = varName;
  }

  @Override
  public String toString() {
    return (_negated ? "!" : "") + "#" + _varName;
  }

  /**  */
  @Override
  public boolean match(DagEdge input, Bindings bindings) {
    DagEdge bound = bindings.getBinding(_varName, Bindings.LOCAL);
    if (bound == null) {
      bindings.bind(_varName, input, Bindings.LOCAL);
      return true;
    } else {
      throw new Error("local variable twice on left hand side: " + _varName);
    }
    // return false lf.isUnifiable(input);
  }

  /** Return the binding associated with this local variable, if there is any
   */
  @Override
  public DagEdge getBinding(DagEdge input, Bindings bindings) {
    if (_varName == null) return input;
    return bindings.getBinding(_varName, Bindings.LOCAL);
  }
}
