package de.dfki.lt.tr.dialogue.cplan;

import java.util.Iterator;
import java.util.List;

import org.apache.log4j.Logger;

import de.dfki.lt.tr.dialogue.cplan.actions.Action;

public class Rule {

  private static boolean WARN_APPLY_FAILURE = true;

  private static Logger logger = Logger.getLogger("UtterancePlanner");

  /** The left hand side of the rule */
  private List<VarMatch> _matches;

  /** The right hand side of the rule */
  private List<Action> _replace;

  /** An object to trace matching / application of rules */
  private static RuleTracer _tracer;

  public static void setTracing(RuleTracer t) {
    _tracer = t;
  }

  /** Create a new rule: bind the left and right hand side */
  public Rule(List<VarMatch> match, List<Action> right) {
    _matches = match;
    _replace = right;
  }

  public static void matchesToString(StringBuilder sb,
      List<VarMatch> matches) {
    for (VarMatch vm : matches) {
      sb.append(vm);
    }
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    matchesToString(sb, _matches);
    sb.append(" -> ");
    Iterator<Action> it = _replace.iterator();
    sb.append(it.next());
    while (it.hasNext()) {
      sb.append(", ").append(it.next());
    }
    return sb.toString();
  }

  public boolean matches(DagEdge here, Bindings bindings) {
    bindings.resetLocalBindings();
    boolean result = true;
    for (VarMatch varMatch : _matches) {
      result = varMatch.match(here, bindings);
      if (! result) break;
    }
    if (result && _tracer != null) {
      _tracer.traceMatch(here, _matches, bindings);
    }
    return result;
  }

  public boolean executeActions(DagEdge current, Bindings bindings) {
    for (Action action : _replace) {
      if (_tracer != null) {
        _tracer.traceBeforeApplication(current, action);
      }
      if (! action.apply(current, bindings)) {
        assert(false);
        DagNode.invalidate();
        if (WARN_APPLY_FAILURE) {
          logger.warn("Unification failure in application phase of " + this
              + " to " + current);
        }
        return false;
      }
      if (_tracer != null) {
        _tracer.traceAfterApplication(current, action);
      }
    }
    return true;
  }

  /** This function should only be used to simulate application of single rules
   *  in test cases. In the overall system, multiple rules are applied in
   *  a sequence before copying the result, using applyLocally for a single
   *  application.
   */
  public DagNode applyLocallyAndCopy(DagNode currentNode) {
    DagEdge current = new DagEdge((short) -1, currentNode);
    Bindings bindings = new Bindings();
    DagNode result = null;
    if (matches(current, bindings)) {
      // the special variable "#" should now be bound to current
      assert(bindings.getBinding("#", Bindings.LOCAL) == current);
      executeActions(current, bindings);
      result = current.getValue().copyResult(false);
    }
    return result;
  }

  /** A function to test the matching part of the rule, only for test
   *  purposes.
   */
  public boolean match(DagNode lf) {
    return matches(new DagEdge((short)-1, lf), new Bindings());
  }
}
