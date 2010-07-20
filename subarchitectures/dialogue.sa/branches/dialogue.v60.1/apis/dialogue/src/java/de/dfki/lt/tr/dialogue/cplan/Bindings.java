package de.dfki.lt.tr.dialogue.cplan;

import java.util.HashMap;

/** This class stores bindings of global and local variables during matching
 *  and modification phases of the graph transformation algorithm
 */
public class Bindings {
  private HashMap<String, DagEdge>[] _bindings;

  /** to count the global bindings stored in this structure */
  private int _globalBindings;

  public static final int LOCAL = 0;
  public static final int GLOBAL = 1;
  public static final int ABSOLUTE = 2;


  @SuppressWarnings("unchecked")
  public Bindings() {
    _bindings = new HashMap[3];
    for (int i = LOCAL; i <= ABSOLUTE; ++i) {
      _bindings[i] = new HashMap<String, DagEdge>();
    }

  }

  /** Establish a binding between a variable and a value */
  public void bind(String name, DagEdge value, int status) {
    if (status == GLOBAL && ! _bindings[status].containsKey(name)) {
      ++_globalBindings;
    }
    _bindings[status].put(name, value);
  }

  /** Return the bound value of variable \p name, if there is one, or \c null */
  public DagEdge getBinding(String name, int status) {
    return _bindings[status].get(name);
  }

  /** Reset the local variable bindings. This function must be called before
   *  processing a rule to remove old local bindings.
   */
  public void resetLocalBindings() {
    _bindings[LOCAL].clear();
  }

  /** Return the number of global bindings in this structure */
  public int getNumberOfGlobalBindings() {
    return _globalBindings;
  }

  /** Transfer the local bindings of this Bindings object into a new Bindings,
   *  and delete the current local bindings
   */
  public Bindings transferLocalBindings() {
    Bindings result = new Bindings();
    HashMap<String, DagEdge> local = _bindings[LOCAL];
    _bindings[LOCAL] = result._bindings[LOCAL];
    result._bindings[LOCAL] = local;
    return result;
  }

  /** Overwrite the local bindings of \c this with those of \a bindings */
  public void restoreLocalBindings(Bindings bindings) {
    _bindings[LOCAL] = bindings._bindings[LOCAL];
  }
}
