package de.dfki.lt.tr.dialogue.cplan.matches;


import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;

/** A syntax tree node representing the disjunction of two subnodes */
public class Disjunction extends Match {
  private Match _left, _right;

  public Disjunction(Match left, Match right) {
    _left = left;
    _right = right;
  }

  @Override
  public String toString() {
    return (_negated ? "!" : "") + "(" + _left + " | " + _right + ")";
  }

  @Override
  public boolean match(DagEdge input, Bindings bindings) {
    return (_left.matches(input,bindings) || _right.matches(input,bindings));
  }

  @Override
  void normalForm() {
    _left.normalForm();
    _right.normalForm();
  }
}
