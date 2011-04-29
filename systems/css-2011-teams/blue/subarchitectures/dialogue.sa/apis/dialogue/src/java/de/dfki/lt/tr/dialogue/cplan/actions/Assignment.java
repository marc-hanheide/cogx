package de.dfki.lt.tr.dialogue.cplan.actions;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.Path;
import de.dfki.lt.tr.dialogue.cplan.VarDagNode;

public class Assignment extends Action {

  public Assignment(VarDagNode lval, Path path, DagNode dagNode) {
    super(lval, path, dagNode);
  }

  @Override
  public boolean apply(DagEdge current, Bindings bindings) {
    DagEdge root = _lval.getLvalBinding(current, bindings);
    if (root == null) return false;
    root = root.getLastEdge(_subpath.iterator());
    if (root == null) return false;
    // cloning the rule is necessary because the variable nodes
    // have to be deleted destructively
    DagNode toAssign = _rval.cloneFS().expandVars(bindings);
    if (toAssign == null) return false;
    root.setValue(toAssign);
    return true;
  }

  @Override
  public String toString() {
    return toString(" = ");
  }

}
