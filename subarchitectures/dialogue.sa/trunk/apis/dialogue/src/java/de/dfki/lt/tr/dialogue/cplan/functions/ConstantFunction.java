package de.dfki.lt.tr.dialogue.cplan.functions;

import java.util.List;

public class ConstantFunction implements Function {

  private String _constant;

  public ConstantFunction(String constant) {
    _constant = constant;
  }

  @SuppressWarnings("unchecked")
  @Override
  public Object apply(List args) {
    return _constant;
  }

  @Override
  public String name() {
    return "constant"+_constant;
  }

  @Override
  public int arity() {
    return 0;
  }
}
