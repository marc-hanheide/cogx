package de.dfki.lt.tr.dialogue.cplan.matches;

import java.util.List;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.util.FunctionFactory;

/** A syntax tree node representing a function application of some custom
 *  (internal or external) function
 */
public class FunCall extends Match {

  /** The name of the function.
   *  TODO it might be good to get the function object directly and cache that
   */
  private String _name;

  /** the list of arguments to the function call */
  @SuppressWarnings("unchecked")
  private List _args;

  /** Create the function call object, giving its name and arguments */
  @SuppressWarnings("unchecked")
  public FunCall(String name, List args) {
    _name = name;
    _args = args;
  }

  @Override
  public String toString() {
    return " " + _name + "(" + _args + ")";
  }

  /** Applying a function on the left hand side means that it has to be a
   *  function returning a boolean value, otherwise, a run time error will
   *  result and throw a ClassCastException
   */
  @Override
  public boolean match(DagEdge input, Bindings bindings) {
    return (Boolean) FunctionFactory.get(_name).apply(bindings, _args);
  }
}