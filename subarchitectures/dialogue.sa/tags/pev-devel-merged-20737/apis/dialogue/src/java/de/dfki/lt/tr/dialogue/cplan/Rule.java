package de.dfki.lt.tr.dialogue.cplan;

import java.util.Iterator;
import java.util.List;

import org.apache.log4j.Logger;

import de.dfki.lt.tr.dialogue.cplan.actions.Action;
import de.dfki.lt.tr.dialogue.cplan.util.Position;

public class Rule {

  private static boolean WARN_APPLY_FAILURE = true;

  private static Logger logger = Logger.getLogger("UtterancePlanner");

  /** The left hand side of the rule */
  private List<VarMatch> _matches;

  /** The right hand side of the rule */
  private List<Action> _replace;

  /** Where was this rule defined */
  private Position _position;

  /** Create a new rule: bind the left and right hand side */
  public Rule(List<VarMatch> match, List<Action> right, RuleParser.Location pos) {
    _matches = match;
    _replace = right;
    _position = new Position(pos.begin.line, 0, pos.begin.msg);
  }

  public StringBuilder appendMatches(StringBuilder sb) {
    for (VarMatch vm : _matches) {
      sb.append(vm);
    }
    return sb;
  }

  public StringBuilder appendActions(StringBuilder sb) {
    Iterator<Action> it = _replace.iterator();
    sb.append(it.next());
    while (it.hasNext()) {
      sb.append(", ").append(it.next());
    }
    return sb;
  }

  public Position getPosition() {
    return _position;
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    return appendActions(appendMatches(sb).append(" -> ")).toString();
  }

  public boolean matches(DagEdge here, Bindings bindings) {
    bindings.resetLocalBindings();
    boolean result = true;
    for (VarMatch varMatch : _matches) {
      result = varMatch.match(here, bindings);
      if (! result) break;
    }
    return result;
  }

  public boolean executeActions(DagEdge current, Bindings bindings) {
    for (Action action : _replace) {
      if (! action.apply(current, bindings)) {
        DagNode.invalidate();
        if (WARN_APPLY_FAILURE) {
          logger.warn("Unification failure in application phase of " + this
              + " to " + current);
        }
        assert(false);
        return false;
      }
    }
    return true;
  }

  /** This function should only be used to simulate application of single rules
   *  in test cases. In the overall system, multiple rules are applied in
   *  a sequence before copying the result, using applyLocally for a single
   *  application.
   */
  DagNode applyLocallyAndCopy(DagNode currentNode) {
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
  boolean match(DagNode lf) {
    return matches(new DagEdge((short)-1, lf), new Bindings());
  }
}
