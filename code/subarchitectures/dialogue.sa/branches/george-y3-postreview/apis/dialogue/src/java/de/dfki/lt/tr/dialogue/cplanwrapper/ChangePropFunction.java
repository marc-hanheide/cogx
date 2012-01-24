package de.dfki.lt.tr.dialogue.cplanwrapper;

import java.util.List;

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;

public class ChangePropFunction extends LFFunction {

  public String name() { return "changeprop"; }

  /** Change prop of arg1 to arg3 if it is equal to arg2 */
  @SuppressWarnings("unchecked")
  @Override
  protected LogicalForm applyLfFunction(List args) {

    LogicalForm lf = (LogicalForm) args.get(0);
    String from, to;
    if (args.get(1) instanceof LogicalForm) {
      from = ((LogicalForm) args.get(1)).root.prop.prop;
    }
    else {
      from = (String) args.get(1);
    }
    if (args.get(2) instanceof LogicalForm) {
      to = ((LogicalForm) args.get(2)).root.prop.prop;
    }
    else {
      to = (String) args.get(2);
    }

    if (lf.root.prop.prop == from) {
      lf.root.prop.prop = to;
    }

    return lf;
  }

  public int arity() { return  3; }
}