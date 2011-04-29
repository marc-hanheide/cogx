package de.dfki.lt.tr.dialogue.cplan;

import java.util.Iterator;
import java.util.Stack;

/** This class stores bindings of global and local variables during matching
 *  and modification phases of the graph transformation algorithm
 */
public class Bindings {
  private Stack<Object []> _bindings;

  /** to count the global bindings stored in this structure */
  private int _globalBindings;

  /** LOCAL bindings are only valid during one rule match/application */
  public static final int LOCAL = 0;
  /** GLOBAL bindings are valid from the moment they are established until the
   *  end of the fixpoint computation.
   */
  public static final int GLOBAL = 1;
  /** Absolute bindings are nominal names, not variables at all */
  public static final int ABSOLUTE = 2;


    public Bindings() {
    _bindings = new Stack<Object []>();
    _globalBindings = 0;
  }

  public Object[] findBinding(String name, int status) {
    for (Object[] triple : _bindings) {
      if (((Integer)triple[0] == status) && ((String)triple[1]).equals(name))
        return triple;
    }
    return null;
  }

  /** Establish a binding between a variable and a value */
  public void bind(String name, DagEdge value, int status) {
    Object[] res = findBinding(name, status);
    if (res == null) {
      if (status == GLOBAL) {
        ++ _globalBindings;
      }
      Object[] newTriple = { Integer.valueOf(status), name, value };
      _bindings.push(newTriple);
    }
  }

  /** Return the bound value of variable \p name, if there is one, or \c null */
  public DagEdge getBinding(String name, int status) {
    Object[] triple = findBinding(name, status);
    return (triple == null ? null : (DagEdge)triple[2]);
  }

  /** Reset the local variable bindings. This function must be called before
   *  processing a rule to remove old local bindings.
   */
  public void resetLocalBindings() {
    Iterator<Object[]> it = _bindings.iterator();
    while (it.hasNext()) {
      Object[] triple = it.next();
      if ((Integer)triple[0] == LOCAL) {
        it.remove();
      }
    }
  }

  /** Return the number of global bindings in this structure */
  public int getNumberOfGlobalBindings() {
    return _globalBindings;
  }

  /** Copy all local bindings from this to target */
  private void copyLocalBindingsTo(Bindings target) {
    for (Object[] triple : _bindings) {
      if ((Integer)triple[0] == LOCAL) {
        target._bindings.push(triple);
      }
    }
  }

  /** Transfer the local bindings of this Bindings object into a new Bindings,
   *  and delete the current local bindings
   */
  public Bindings transferLocalBindings() {
    Bindings result = new Bindings();
    copyLocalBindingsTo(result);
    resetLocalBindings();
    return result;
  }

  /** Overwrite the local bindings of \c this with those of \a bindings */
  public void restoreLocalBindings(Bindings bindings) {
    resetLocalBindings();
    bindings.copyLocalBindingsTo(this);
  }

  /** Get a pointer that points past the last binding */
  public int getLevel() {
    return _bindings.size();
  }

  public void retractToLevel(int level) {
    while (_bindings.size() > level) {
      Object[] triple = _bindings.pop();
      if ((Integer)triple[0] == GLOBAL) {
        --_globalBindings;
      }
    }
  }
}
