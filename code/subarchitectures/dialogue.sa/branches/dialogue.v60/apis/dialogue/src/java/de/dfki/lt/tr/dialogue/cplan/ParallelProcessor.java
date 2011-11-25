package de.dfki.lt.tr.dialogue.cplan;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Stack;
import java.util.Vector;

public class ParallelProcessor implements Processor {

  private List<Rule> _rules;

  public ParallelProcessor(List<Rule> rules) {
    _rules = rules;
  }

  private class RuleAction {
    Rule _rule;
    DagEdge _applicationPoint;
    Bindings _bindings;

    public RuleAction(Rule r, Vector<DagEdge> path, Bindings bindings) {
      _rule = r;
      _applicationPoint = path.lastElement();
      _bindings = bindings.transferLocalBindings();
    }

    public void apply(Bindings bindings) {
      bindings.restoreLocalBindings(_bindings);
      _rule.executeActions(_applicationPoint, bindings);
    }
  }

  private List<RuleAction> computeMatches(DagNode lf, Bindings bindings) {
    List<RuleAction> result = new LinkedList<RuleAction>();

    Stack<DagEdge> path = new Stack<DagEdge>();
    Stack<Iterator<DagEdge>> iterators = new Stack<Iterator<DagEdge>>();
    {
      List<DagEdge> root = new ArrayList<DagEdge>();
      root.add(new DagEdge((short)-1, lf));
      iterators.push(root.iterator());
    }
    path.push(null);
    while (! iterators.isEmpty()) {
      DagEdge nextEdge = null;
      while (nextEdge == null && ! iterators.isEmpty()) {
        if (iterators.peek().hasNext()) {
          nextEdge = iterators.peek().next();
          path.pop();
          path.push(nextEdge);
        }
        else {
          path.pop();
          iterators.pop();
        }
      }
      if (nextEdge != null) {
        // apply rules to the node under path in the modified structure
        for (Rule r : _rules) {
          if (r.matches(path.lastElement(), bindings)) {
            result.add(new RuleAction(r, path, bindings));
          }
        }
        iterators.push(nextEdge.getValue().getEdgeIterator());
        path.push(null);
      }
    }
    return result;
  }


  @Override
  public DagNode applyRules(DagNode lf, Bindings bindings) {
    List<RuleAction> actions = computeMatches(lf, bindings);
    if (actions != null) {
      for (RuleAction action : actions) {
        action.apply(bindings);
      }
      return lf.copyResult(false);
    }
    else {
      return lf;
    }
  }

}