package de.dfki.lt.sdl;

/**
 * this is the interface which every module (may it be global or local
 * composite) MUST fulfill!!
 *
 * @see IMediator
 * @see Mediators
 * @see Modules
 * @see ModuleRunError
 * @see ModuleClearError
 * @see ModuleInitError
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.3.2, Mon May 10 15:56:37 CEST 2004
 */
public interface IModule {

  /*
   * clear() clears the internal state of the module
   */
  public void clear() throws ModuleClearError;

  /**
   * init() initializes the module, specifying an array of init arguments
   * which the module has to interpret
   */
  public void init(String[] initArgs) throws ModuleInitError;

  /**
   * run(_) MUST realize the computation of the module;
   * this unary method must be distinguished from the nullary method run()
   * in the Runnable interface;
   * note that run(_) is NOT forced to set its input nor its output field
   * and should usually avoid this!!
   */
  public Object run(Object input) throws ModuleRunError;

  /**
   * sets the input of the module to parameter input and returns input
   */
  public Object setInput(Object input);

  /**
   * returns the input of the module
   */
  public Object getInput();

  /**
   * sets the output of the module to parameter output and returns output
   */
  public Object setOutput(Object output);

  /**
   * returns the output of the module which has been computed through the
   * use of run()
   */
  public Object getOutput();

}
