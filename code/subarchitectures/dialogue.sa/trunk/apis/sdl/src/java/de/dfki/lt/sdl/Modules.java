package de.dfki.lt.sdl;

/**
 * the abstract superclass of all modules, implementing some default functionality
 * which can be inherited by a concrete implementation;
 *
 * default implementation is given for
 *   o clear()
 *   o init()
 *   o getInput()
 *   o setInput()
 *   o getOutput()
 *   o setOutput()
 *
 * NO useful implementation is given for the unary method
 *   o run()
 * only an UnsupportedOperationException is thrown
 *
 * @see IMediator
 * @see Mediators
 * @see IModule
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.1.4, Mon May 10 15:58:41 CEST 2004
 */
public abstract class Modules implements IModule {

  /**
   * the private instance field which stores the input
   */
  private Object _input;

  /**
   * the private instance field which stores the output
   */
  private Object _output;

  /**
   * since Modules() is an abstract class, this constructor is only indirectly
   * called from subclasses by using super();
   * effects (at the moment): the constructor clears input and output (null value)
   */
  protected Modules() {
    this._input = null;
    this._output = null;
  }

  /**
   * this default method does NOT provide a useful implementation for run();
   * instead, an UnsupportedOperationException is thrown
   */
  public Object run(Object input) throws UnsupportedOperationException {
    throw new UnsupportedOperationException("run(_) is NOT implemented for this module");
  }

  /**
   * the default implementation of clear() assigns the null value to both input
   * and output;
   * the domain and range of this module is NOT changed;
   * additional effects must be implemented by overriding clear() in subclasses
   * of this class
   */
  public void clear() {
    this._input = null;
    this._output = null;
  }

  /**
   * the default implementation of init() has no effect on the module;
   * effects must be implemented by overriding init() in subclasses of
   * this class (if this is needed)
   */
  public void init(String[] initArgs) {
  }

  /**
   * returns the input of this module
   */
  public Object getInput() {
    return this._input;
  }

  /**
   * sets the input of this module and returns the value
   */
  public Object setInput(Object input) {
    return (this._input = input);
  }

  /**
   * returns the result of the computation of this module
   */
  public Object getOutput() {
    return this._output;
  }

  /**
   * sets the output of this module and returns the value
   */
  public Object setOutput(Object output) {
    return (this._output = output);
  }

}
