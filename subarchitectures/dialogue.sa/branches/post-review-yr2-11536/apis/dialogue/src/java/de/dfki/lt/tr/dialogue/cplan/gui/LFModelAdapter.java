package de.dfki.lt.tr.dialogue.cplan.gui;

import java.util.Arrays;
import java.util.Iterator;


import de.dfki.lt.loot.Pair;
import de.dfki.lt.loot.gui.adapters.MapAdapterIterator;
import de.dfki.lt.loot.gui.adapters.ModelAdapter;
import de.dfki.lt.loot.gui.adapters.ModelAdapterFactory;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.matches.Match;

public class LFModelAdapter extends ModelAdapter {

  public static short[] _excludedFeatures;

  public static void init() {
    ModelAdapterFactory.register(DagNode.class, LFModelAdapter.class);
    _excludedFeatures = new short[3];
    _excludedFeatures[0] = DagNode.ID_FEAT_ID;
    _excludedFeatures[1] = DagNode.TYPE_FEAT_ID;
    _excludedFeatures[2] = DagNode.PROP_FEAT_ID;
  }

  private class EdgeAdapterIterator implements MapAdapterIterator {
    Pair<String, Object> _current;
    private Iterator<? extends DagEdge> _iterator;

    /** precondition: _iterator != null */
    private void advance() {
      _current = null;
      while (_current == null && _iterator.hasNext()) {
        DagEdge edge = _iterator.next();
        if (_excludedFeatures == null ||
            Arrays.binarySearch(_excludedFeatures, edge.getFeature()) < 0)
          _current = new Pair<String, Object>(edge.getName(), edge.getValue());
      }
    }

    public EdgeAdapterIterator(DagNode node) {
      _current = null;
      _iterator = node.getEdgeIterator();
      if (_iterator != null) {
        advance();
      }
    }

    public boolean hasNext() { return _current != null; }

    public Pair<String, Object> next() {
      Pair<String, Object> result = _current;
      advance();
      return result;
    }
  }

  private class EdgeProjectionIterator implements Iterator {
    Iterator<? extends DagEdge> _iterator;

    public EdgeProjectionIterator(Iterator<? extends DagEdge> it) {
      _iterator = it;
    }

    @Override
    public boolean hasNext() {
      return _iterator != null && _iterator.hasNext();
    }

    @Override
    public Object next() { return _iterator.next().getValue(); }

    @Override
    public void remove() { throw new UnsupportedOperationException(); }
  }

  private class TreeMarker {
    DagNode _root;
    TreeMarker(DagNode root) { _root = root; }
  }

  @Override
  public int facets(Object model) {
    return ModelAdapter.MAP;
  }

  @Override
  public String getAttribute(Object model, String name) {
    DagNode node;
    if (model instanceof TreeMarker) {
      node = ((TreeMarker) model)._root;
    } else {
      node = (DagNode) model;
    }

    /* */
    if (name.charAt(0) == 't') {
      if (node.isNominal()) {
        DagNode.EdgeIterator it = node.getTransitionalEdgeIterator();
        DagEdge edge = null;
        DagNode prop = null ;
        DagNode id = null;
        DagNode type = null;

        while (it.hasNext()) {
          edge = it.next();
          short feature = edge.getFeature();
          if (feature == _excludedFeatures[0]) {
            id = edge.getValue().dereference(); edge = null;
          } else if (feature == _excludedFeatures[1]) {
            type = edge.getValue().dereference(); edge = null;
          } else if (feature == _excludedFeatures[2]) {
            prop = edge.getValue().dereference(); edge = null;
          } else {
            break;
          }
        }
        if (id == null && type == null && prop == null) return null;

        StringBuilder sb = new StringBuilder();
        if (id != null) sb.append(id.getTypeName());
        sb.append(":");
        if (type != null) sb.append(type.getTypeName());
        if (prop != null) sb.append(" ^ ").append(prop.getTypeName());
        return sb.toString();
      }
      else {
        return
        node.getEdge(_excludedFeatures[2]).getValue().dereference().getTypeName();
        //return node.getTypeName();
      }
    }
    /* */
    return null;
  }

  @Override
  public MapAdapterIterator mapIterator(Object model) {
    if (model instanceof TreeMarker) {
      return new EdgeAdapterIterator(((TreeMarker) model)._root);
    }
    return new EdgeAdapterIterator((DagNode) model);
  }

}
