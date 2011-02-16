package de.dfki.lt.tr.dialogue.cplanwrapper;

import java.util.ArrayList;
import java.util.List;

import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.functions.Function;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;

@SuppressWarnings("unchecked")
public abstract class LFFunction implements Function {

  List convertArgs(List dagArgs) {
    // convert all dagArgs appropriately
    List result = new ArrayList(dagArgs.size());
    for (Object arg : dagArgs) {
      Object converted = arg;
      // TODO let's hope we got all cases. Maybe if a DagNode contains only
      // a PROP, it should be (have been) converted into an atomic symbol.
      if (arg instanceof DagNode) {
        DagNode dag = (DagNode) arg;
        if (dag.newEdgesAreEmpty()) {
          converted = dag.getTypeName();
        }
        else {
          // TODO This may be problematic because the dag could be partial, not a
          // complete LF. Test if this works, but conversion fn looks good.
          converted = CPlanWrapper.dagNodeToLf((DagNode) arg);
        }
      } else if (arg instanceof String) {
        converted = arg;
      }
      result.add(converted);
    }
    return result;
  }

  protected abstract LogicalForm applyLfFunction(List args);

  @Override
  public final Object apply(List args) {
    Object result = applyLfFunction(convertArgs(args));
    if (result instanceof LogicalForm) {
      return CPlanWrapper.lfToDagNode((LogicalForm) result);
    }
    return result;
  }

}
