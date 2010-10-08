package de.dfki.lt.tr.dialogue.cplan.io;

import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.SpecialDagNode;

public class LFDebugPrinter extends DagPrinter {

  private StringBuilder specialEdgeString(DagEdge edge, StringBuilder sb) {
    DagNode val = edge.getValue().dereference();
    if (val instanceof SpecialDagNode) {
      ((SpecialDagNode)val).toStringSpecial(sb);
    } else {
      sb.append(val.getTypeName());
    }
    return sb;
  }

  @Override
  public void toStringRec(DagNode hereNode, boolean readable,
      StringBuilder sb) {
    if (hereNode == null) {
      sb.append("(null)");
      return;
    }

    DagNode here = hereNode.dereference();
    int corefNo = getCorefNo(here);
    if (corefNo < 0) { // already printed, only coref
      sb.append(" #").append(-corefNo).append(' ');
      return;
    }
    if (corefNo > 0) {
      sb.append(" #").append(corefNo).append(' ');
    }

    sb.append('[');
    // sb.append(readable ? here.getTypeName() : here.getType());
    DagNode.EdgeIterator fvListIt = here.getTransitionalEdgeIterator();
    if (fvListIt != null && fvListIt.hasNext()) {
      while(fvListIt.hasNext()) {
        DagEdge edge = fvListIt.next();
        short feature = edge.getFeature();
        sb.append(' ');
        if (feature == DagNode.ID_FEAT_ID) {
          specialEdgeString(edge, sb).append(':');
        } else if (feature == DagNode.TYPE_FEAT_ID) {
          sb.append(':'); specialEdgeString(edge, sb);
        } else if (feature == DagNode.PROP_FEAT_ID) {
          specialEdgeString(edge, sb);
        } else {
          sb.append(readable ? edge.getName() : edge.getFeature());
          toStringRec(edge.getValue(), readable, sb);
        }
      }
    }
    else {
      sb.append('@').append(here.getTypeName()).append('@');
    }
    sb.append(']');
  }
}
