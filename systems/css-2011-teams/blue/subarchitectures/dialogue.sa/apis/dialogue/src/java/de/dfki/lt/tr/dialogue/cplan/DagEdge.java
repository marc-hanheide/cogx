package de.dfki.lt.tr.dialogue.cplan;

import java.util.Iterator;

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

}
