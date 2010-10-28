package de.dfki.lt.tr.dialogue.cplan.functions;

import java.util.List;

/** A generic function object */
public interface Function {

  /** to call this function */
  @SuppressWarnings("unchecked")
  public abstract Object apply(List args);

  /** The name of this function, for registration in the factory */
  public abstract String name();

  /** The arity, a negative value that a variable amount of arguments
   * is possible.
   */
  public abstract int arity();
}
