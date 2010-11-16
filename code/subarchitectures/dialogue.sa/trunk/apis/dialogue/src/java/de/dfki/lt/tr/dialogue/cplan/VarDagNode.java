package de.dfki.lt.tr.dialogue.cplan;

public class VarDagNode extends SpecialDagNode {

  private String _varName;

  public VarDagNode(String string, int status) {
    super(status);
    _varName = string;
  }

  @Override
  public DagNode clone(int type) {
    return new VarDagNode(_varName, type);
  }

  /** Return the binding associated with this variable, if there is any. This
   *  method is only to be used to determine the lval of an \c Action
   */
  public DagEdge getLvalBinding(DagEdge input, Bindings bindings) {
    // varName "#" is reserved and always bound to the current node
    DagEdge current = bindings.getBinding(_varName, getType());
    if (current == null && getType() == Bindings.GLOBAL) {
      // create a new global variable
      current = new DagEdge((short)-1, new DagNode());
      bindings.bind(_varName, current, getType());
    }
    if (current == null) {
      logger.warn("local variable not bound and used as lval " + _varName);
    }
    return current;
  }

  /** if _varname is in the bindings, return the value of the binding, otherwise
   *  bind varname to the provided parent DagEdge
   */
  @Override
  protected DagNode evaluate(DagNode parent, Bindings bindings) {
    // type acts as storage for the status
    DagEdge bound = bindings.getBinding(_varName, getType());
    if (bound == null) {
      if (getType() == Bindings.LOCAL) {
        // first occurence of this var, bind the variable name
        // to the given node. This is on the application side, where new bindings
        // are only relevant in expandVars, which is why the DagEdge is not
        // important, but the node, to establish coreferences.
        bound = new DagEdge((short) -1, parent);
        bindings.bind(_varName, bound, getType());
      } else {
        logger.warn("Unknown binding during application of rule: " + this);
        return parent;
      }
    }
    // Avoid unwanted coreferences for atomic nodes
    if (bound.getValue().newEdgesAreEmpty()) {
      return bound.getValue().cloneFS();
    }
    return bound.getValue();
  }

  public DagNode getBoundNode(Bindings bindings) {
    DagEdge bound = bindings.getBinding(_varName, getType());
    assert(bound != null);
    return bound.getValue();
  }

  @Override
  public void toStringSpecial(StringBuilder sb) {
    switch (getType()) {
    case Bindings.ABSOLUTE:
      sb.append(_varName).append(':');
      break;
    case Bindings.LOCAL:
      sb.append('#');
      if (_varName != null && _varName.charAt(0) != '#') sb.append(_varName);
      break;
    case Bindings.GLOBAL:
      sb.append("##").append(_varName);
      break;
    }
  }
}
