package de.dfki.lt.tr.dialogue.cplan.functions;

import java.util.List;
import java.util.Random;

public class DiscreteRandomFunction implements Function {

  private Random rand;

  public DiscreteRandomFunction() {
    rand = new Random(System.currentTimeMillis());
  }

  @SuppressWarnings("unchecked")
  @Override
  public Object apply(List args) {
    int i = rand.nextInt(args.size());
    return args.get(i);
  }

  @Override
  public int arity() {
    return -1;
  }

  @Override
  public String name() {
    return "random";
  }
}
