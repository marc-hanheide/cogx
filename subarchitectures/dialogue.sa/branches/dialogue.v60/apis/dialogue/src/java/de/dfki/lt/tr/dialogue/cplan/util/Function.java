package de.dfki.lt.tr.dialogue.cplan.util;

import java.util.List;

import de.dfki.lt.tr.dialogue.cplan.Bindings;

/** A generic function object */
public interface Function {
  @SuppressWarnings("unchecked")
  public abstract Object apply(Bindings bindings, List args);
}
