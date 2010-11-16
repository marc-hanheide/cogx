package de.dfki.lt.tr.dialogue.cplan;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;

public class DagEdge {

  short _feature;
  DagNode _value;

  public DagEdge(short feature, DagNode value) {
    _feature = feature;
    _value = value;
  }

  public short getFeature() {
    return _feature;
  }

  public DagNode getValue() {
    return _value;
  }

  public String getName() {
    return DagNode.getFeatureName(_feature);
  }

  @Override
  public String toString() {
    return '<' + DagNode.getFeatureName(_feature) + '>' + _value;
  }

  /** Get the last edge of the given path.
   *  Works correctly only on non-temporary dags.
   */
  public DagEdge getLastEdge(Iterator<Short> path) {
    DagEdge current = this;
    while (path.hasNext() && current != null) {
      current = current._value.getEdge(path.next());
    }
    return current;
  }

  /** return all edges that emerge from the target node of this edge */
  public Iterator<DagEdge> getEdges(short feature) {
    return _value.getEdges(feature);
  }

  /** Set the target node of this edge to \a newTarget */
  public void setValue(DagNode newTarget) {
    _value = newTarget;
  }
  /** Almost identical to copyResultRec, but does not use the
   *  generation-protected slots.
   */

  public DagEdge copyIntermediate(Map<Object, Object> origToCopy) {
    DagEdge edgeCopy = (DagEdge) origToCopy.get(this);
    if (edgeCopy != null) {
      return edgeCopy;
    }

    DagNode in = this.getValue().dereference();
    DagNode newCopy = (DagNode) origToCopy.get(in);
    if (newCopy != null) {
      edgeCopy = new DagEdge(_feature, newCopy);
      origToCopy.put(this, edgeCopy);
      return edgeCopy;
    }

    newCopy = in.clone(in.getNewType());
    origToCopy.put(in, newCopy);
    edgeCopy = new DagEdge(_feature, newCopy);
    origToCopy.put(this, edgeCopy);

    int newsize = 0;
    int cursorArcs = -1, cursorCompArcs = -1;
    if (in._outedges != null && ! in._outedges.isEmpty()) {
      cursorArcs = 0;
      newsize = in._outedges.size();
    }
    if (in._generation == DagNode.currentGeneration
        && (in._compArcs != null && ! in._compArcs.isEmpty())) {
      cursorCompArcs = 0;
      newsize += in._compArcs.size();
    }

    if (cursorArcs != -1 || cursorCompArcs != -1) {
      newCopy._outedges = new ArrayList<DagEdge>(newsize);
    }
    while (cursorArcs != -1 || cursorCompArcs != -1) {
      DagEdge arc = null ;
      if (cursorArcs != -1 &&
          (cursorCompArcs == -1
           || (in._compArcs.get(cursorCompArcs)._feature
               > in._outedges.get(cursorArcs)._feature))) {
        int curr = cursorArcs;
        if (++cursorArcs == in._outedges.size()) {
          cursorArcs = -1;
        }
        arc = in._outedges.get(curr);
      } else {
        int curr = cursorCompArcs;
        if (++cursorCompArcs == in._compArcs.size()) {
          cursorCompArcs = -1;
        }
        arc = in._compArcs.get(curr);
      }
      newCopy._outedges.add(arc.copyIntermediate(origToCopy));
    }
    return edgeCopy;
  }

}
