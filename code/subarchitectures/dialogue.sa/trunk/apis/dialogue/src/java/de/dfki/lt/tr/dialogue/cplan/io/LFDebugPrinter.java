package de.dfki.lt.tr.dialogue.cplan.io;

import gnu.trove.TObjectIntHashMap;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;

public class LFDebugPrinter implements DagPrinter {

  public void toStringRec(DagNode hereNode, boolean readable,
      StringBuilder sb, TObjectIntHashMap<DagNode> corefs) {
    if (hereNode == null) {
      sb.append("(null)");
      return;
    }

    DagNode here = hereNode.dereference();
    int corefNo = corefs.get(here);
    if (corefNo < 0) { // already printed, only coref
      sb.append(" #").append(-corefNo).append(' ');
      return;
    }

    if (corefNo > 0) {
      sb.append(" #").append(corefNo).append(' ');
      corefs.put(here, -corefNo);
    }

    sb.append('[');
    // sb.append(readable ? here.getTypeName() : here.getType());
    DagNode.EdgeIterator fvListIt = here.getTransitionalEdgeIterator();
    if (fvListIt != null) {
      while(fvListIt.hasNext()) {
        DagEdge edge = fvListIt.next();
        short feature = edge.getFeature();
        sb.append(' ');
        if (feature == DagNode.ID_FEAT_ID) {
          sb.append(edge.getValue().dereference().getTypeName()).append(':');
        } else if (feature == DagNode.TYPE_FEAT_ID) {
          sb.append(':').append(edge.getValue().dereference().getTypeName());
        } else if (feature == DagNode.PROP_FEAT_ID) {
          sb.append(edge.getValue().dereference().getTypeName());
        } else {
          sb.append(readable ? edge.getName() : edge.getFeature());
          toStringRec(edge.getValue(), readable, sb, corefs);
        }
      }
    }
    sb.append(']');
  }

  @Override
  public void maxCoref(int corefs) { }
}
