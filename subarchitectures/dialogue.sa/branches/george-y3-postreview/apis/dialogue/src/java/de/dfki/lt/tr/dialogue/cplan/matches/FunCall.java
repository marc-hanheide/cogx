package de.dfki.lt.tr.dialogue.cplan.matches;

import java.util.Iterator;
import java.util.List;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.FunCallDagNode;
import de.dfki.lt.tr.dialogue.cplan.functions.Function;
import de.dfki.lt.tr.dialogue.cplan.functions.FunctionFactory;

/** A syntax tree node representing a function application of some custom
 *  (internal or external) function
 */
public class FunCall implements MatchLVal {

  /** The name of the function.
   *  TODO it might be good to get the function object directly and cache that
   */
  private String _name;
  private Function _fn;

  /** the list of arguments to the function call */
  @SuppressWarnings("unchecked")
  private List _args;

  /** Create the function call object, giving its name and arguments */
  @SuppressWarnings("unchecked")
  public FunCall(String name, List args) throws NoSuchMethodException {
    _name = name;
    _fn = FunctionFactory.get(_name);
    if (_fn == null) {
      throw new NoSuchMethodException(_name);
    }
    _args = args;
  }

  @SuppressWarnings("unchecked")
  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append(_name).append("(");
    if (! _args.isEmpty()) {
      sb.append(_args.get(0));
    }
    Iterator it = _args.iterator();
    it.next();
    while (it.hasNext()) {
      sb.append(' ').append(it.next());
    }
    sb.append(")");
    return sb.toString();
  }

  /* We don't support this until somebody REALLY needs it */

  /** Applying a function on the left hand side means that it has to be a
   *  function returning a boolean value, otherwise, a run time error will
   *  result and throw a ClassCastException
   *
  @Override
  public boolean match(DagEdge input, Bindings bindings) {
    Object result =
      _fn.apply(FunCallDagNode.getActualParameters(_args, null, bindings));
    if (result instanceof Boolean) {
      return (Boolean) result;
    }
    if (result instanceof DagNode) {
      // TODO match result against input: does result subsume input
      throw new UnsupportedOperationException("Subsumption of logical forms" +
                                              " not  implemented yet");
    }
    return false;
  }
  */

  /** The implementation of MatchLVal */
  @Override
  public DagEdge getBinding(DagEdge input, Bindings bindings) {
    Object result =
      _fn.apply(FunCallDagNode.getActualParameters(_args, null, bindings));
    DagNode subRes = (result instanceof DagNode)
                     ? (DagNode) result
                     : new DagNode(result.toString());
    return new DagEdge((short)-1,
        (subRes.newEdgesAreEmpty()
         ? new DagNode(DagNode.PROP_FEAT_ID, subRes)
         : subRes));

  }
}