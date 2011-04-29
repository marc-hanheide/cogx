package de.dfki.lt.sdl;

/**
 * this abstract class serves as the abstract superclass for the three
 * error classes possible in SDL
 *
 * @see ModuleClearError
 * @see ModuleInitError
 * @see ModuleRunError
 * @see IModule
 * @see Modules
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.0.3, Mon May 10 16:22:46 CEST 2004
 */
public abstract class SdlError extends Error {

  /**
   * the nullary creator
   * @return a ModuleClearError object
   */
  public SdlError() {
    super();
  }

  /**
   * the unary creator
   * @param errorMessage the error message
   * @return a ModuleClearError object
   */
  public SdlError(String errorMessage) {
    super(errorMessage);
  }
  
}
