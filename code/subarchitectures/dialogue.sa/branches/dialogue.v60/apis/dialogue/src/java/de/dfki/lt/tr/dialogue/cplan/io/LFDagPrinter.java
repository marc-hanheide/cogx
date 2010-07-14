package de.dfki.lt.tr.dialogue.cplan.io;

import gnu.trove.TObjectIntHashMap;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.SpecialDagNode;

public class LFDagPrinter implements DagPrinter {

  private int _nextNominal;

  private String newNominalName(int corefNo) {
    return "nom" + ((corefNo == 0) ? (++ _nextNominal) : Math.abs(corefNo));
  }

  public void toStringRec(DagNode here, boolean readable, StringBuilder sb,
      TObjectIntHashMap<DagNode> corefs) {
    if (here == null) return;

    here = here.dereference();

    boolean root = (sb.length() == 0);

    // subclasses will print themselves
    if (here instanceof SpecialDagNode) {
      ((SpecialDagNode)here).toStringSpecial(sb);
      return;
    }

    // arrange for _isNominal nodes without id that the nominal name is
    // derived from the coref no instead.

    if (here.isNominal()) {
      DagNode prop = null ;
      DagNode id = null;
      DagNode type = null;

      DagNode.EdgeIterator it = here.getTransitionalEdgeIterator();
      DagEdge edge = null;
      while (it.hasNext()) {
        edge = it.next();
        short feature = edge.getFeature();
        if (feature == DagNode.ID_FEAT_ID) {
          id = edge.getValue().dereference(); edge = null;
        } else if (feature == DagNode.TYPE_FEAT_ID) {
          type = edge.getValue().dereference(); edge = null;
        } else if (feature == DagNode.PROP_FEAT_ID) {
          prop = edge.getValue().dereference(); edge = null;
        } else {
          break;
        }
      }

      int corefNo = corefs.get(here);
      String idName =
        ((id != null) ? id.getTypeName() : newNominalName(corefNo));

      if (corefNo < 0) { // already printed, only id
        sb.append(idName).append(':');
        if (type != null) {
          toStringRec(type, readable, sb, corefs);
        }
        return;
      }
      corefs.put(here, -corefNo);

      boolean printCaret = false;
      if (root) {
        sb.append('@').append(idName).append(':');
        if (type != null) {
          toStringRec(type, readable, sb, corefs);
        }
        sb.append('(');
      }
      else {
        sb.append('(').append(idName).append(':');
        if (type != null) {
          toStringRec(type, readable, sb, corefs);
        }
        printCaret = true;
      }

      if (prop != null) {
        if (printCaret)
          sb.append(" ^ ");
        else
          printCaret = true;
        toStringRec(prop, readable, sb, corefs);
      }

      //sb.append(readable ? here.getTypeName() : here.getType());
      while(edge != null) {
        if (printCaret)
          sb.append(" ^ ");
        else
          printCaret = true;
        sb.append("<")
        .append(readable ? edge.getName() : edge.getFeature())
        .append(">");
        toStringRec(edge.getValue(), readable, sb, corefs);
        if (it.hasNext()) {
          edge = it.next();
        } else {
          edge = null;
        }
      }
      sb.append(')');
    } else {
      // a "feature" feature: exactly one edge: prop
      DagNode.EdgeIterator it = here.getTransitionalEdgeIterator();
      if (it.hasNext()) {
        DagEdge edge = it.next();
        assert(edge.getFeature() == DagNode.PROP_FEAT_ID);
        assert(! it.hasNext());
        DagNode sub = edge.getValue();
        if (sub != null) {
          sb.append(sub.dereference().getTypeName());
        }
      }
      else {
        sb.append(here.getTypeName());
      }
    }
 }

  @Override
  public void maxCoref(int corefs) {
    _nextNominal = corefs;
  }
}
