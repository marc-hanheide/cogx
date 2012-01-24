package de.dfki.lt.tr.dialogue.cplan.actions;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.Path;
import de.dfki.lt.tr.dialogue.cplan.VarDagNode;

public class Addition extends Action {

  public Addition(VarDagNode lval, Path shortList, DagNode dagNode) {
    super(lval, shortList, dagNode);
  }

  @Override
  public boolean apply(DagEdge input, Bindings bindings) {
    DagEdge root = _lval.getLvalBinding(input, bindings);
    if (root == null) return false;
    root = root.getLastEdge(_subpath.iterator());
    if (root == null) return false;
    // cloning the rule is necessary because the variable nodes
    // have to be deleted destructively
    DagNode toAdd = _rval.cloneFS().expandVars(bindings);
    if (toAdd == null) return false;
    root.getValue().add(toAdd);
    return true;
  }

  @Override
  public String toString() {
    return toString(" ^ ");
  }

}
