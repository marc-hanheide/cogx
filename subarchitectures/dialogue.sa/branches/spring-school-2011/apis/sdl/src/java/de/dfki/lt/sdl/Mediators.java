package de.dfki.lt.sdl;

import java.util.*;

/**
 * this is the default (super)class of the three mediators
 *   o seq: sequence or concatenation,
 *   o par: parallelism or concurrency,
 *   o fix: fixpoint or unrestricted iteration,
 * providing some default behavior;
 *
 * I originaly thought that this functionality (without the trival default
 * sequence mediator) had to be part of the module interface;
 * however, a module should usually NOT know what a fixpoint is (in case of
 * module iteration);
 * moreover, when moving the mediators into the module interface, an asymmetry
 * for the parallelism pattern will occur;
 *
 * NOTE: the mediators do NOT set the input NOR the output of their module
 * parameters!!
 *
 * note that in case we want to have specific mediators for certain modules/
 * module combinations, we can easily achieve this by providing additional
 * instance methods with different signatures in subclasses of Mediators
 *
 * @see IMediator
 * @see IModule
 * @see Modules
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.1.3, Mon May 10 15:17:05 CEST 2004
 */
public class Mediators implements IMediator {

  /**
   * no functionality in this constructor, only needed to call specialized
   * instance mediators
   */
  public Mediators() {
  }

  /**
   * the implemented default behavior for the sequence mediator is identity,
   * returning the output of module1
   */
  public Object seq(IModule module1, IModule module2) {
    // default: ignore module2
    return module1.getOutput();
  }

  /**
   * the default parallelism mediator simply groups the output of the modules
   * in an array (of length equals to the number of incoming modules)
   */
  public Object par(IModule[] modules) {
    Object[] result = new Object[modules.length];
    for (int i = 0; i < modules.length; i++)
      result[i] = modules[i].getOutput();
    return result;
  }

  /**
   * the fixpoint mediator checks whether the application of the unary module
   * method run() to the input is equivalent to the result value;
   * if not, it calls run() on the output again, until a fixpoint has been
   * reached (which must, of course, not exist, i.e., the computation must
   * not terminate);
   * note that this default method assumes that input AND output of a module
   * are of the same data type and that the equals() method on the type of
   * input/output works properly; in order to guarantee this, the compareTo()
   * and the equals() method of elements in a structured input/output must
   * also work properly, perhaps even the hashCode() method;
   * note further that fix() does NOT set the input NOR the output of the
   * module!!
   */
  public Object fix(IModule module) {
    return fixpoint(module, module.getInput());
  }

    /**
     * does the recursive job for fix()
     */
  private Object fixpoint(IModule module, Object input) {
    Object output = module.run(input);
    if (output.equals(input))
      return output;
    else
      return fixpoint(module, output);
  }

}
