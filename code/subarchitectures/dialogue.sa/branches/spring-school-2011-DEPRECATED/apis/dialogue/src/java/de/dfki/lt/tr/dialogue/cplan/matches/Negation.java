package de.dfki.lt.tr.dialogue.cplan.matches;


import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;

/** A syntax tree node representing the negation of the child node */

public class Negation extends Match {
  private Match _negatedChild;

  public Negation(Match child) {
    _negatedChild = child;
  }

  @Override
  public String toString() {
    return " !" + _negatedChild;
  }

  @Override
  public boolean match(DagEdge input, Bindings bindings) {
    return ! _negatedChild.match(input, bindings);
  }

  @Override
  void normalForm() {
    _negatedChild.normalForm();
  }
}
