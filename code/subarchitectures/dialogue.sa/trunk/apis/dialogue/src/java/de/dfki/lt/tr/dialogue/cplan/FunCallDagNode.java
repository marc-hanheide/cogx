package de.dfki.lt.tr.dialogue.cplan;

import java.util.Iterator;
import java.util.List;

@SuppressWarnings("unchecked")
public class FunCallDagNode extends SpecialDagNode {

  private String _name;
  private List _args;

  public FunCallDagNode(String name, List args) {
    _name = name;
    _args = args;
  }

  @Override
  public DagNode clone(int type) {
    FunCallDagNode result = new FunCallDagNode(_name, _args);
    result._typeCode = type;
    return result;
  }

  @Override
  public void toStringSpecial(StringBuilder sb) {
    sb.append(_name).append('(');
    Iterator it = _args.iterator();
    if (it.hasNext()) {
      sb.append((it.next()).toString());
    }
    while (it.hasNext()) {
      sb.append(", ").append((it.next()).toString());
    }
    sb.append(')');
  }

  @Override
  public DagNode evaluate(DagNode input, Bindings bindings) {
    // TODO Implement calling of functions, they have to return a DagNode
    return null;
  }

}
