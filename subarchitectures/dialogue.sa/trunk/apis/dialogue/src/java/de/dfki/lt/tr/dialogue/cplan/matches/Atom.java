package de.dfki.lt.tr.dialogue.cplan.matches;


import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;

/** A syntax tree node representing an atomic node */
public class Atom extends Match {
  String _value;

  public Atom(String value) {
    _value = value;
  }

  @Override
  public String toString() {
    return (_negated ? "!" : "") + _value;
  }

  @Override
  public boolean match(DagEdge input, Bindings bindings) {
    return (DagNode.getTypeId(_value) == input.getValue().getNewType());
  }
}
