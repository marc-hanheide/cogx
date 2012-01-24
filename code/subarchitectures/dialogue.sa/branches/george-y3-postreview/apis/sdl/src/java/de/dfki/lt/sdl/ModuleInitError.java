package de.dfki.lt.sdl;

/**
 * instances of this class might be thrown during the initialization phase
 * of a module, caused by the calling init()
 *
 * @see IModule
 * @see Modules
 * @see SdlError
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.0.2, Mon May 10 16:23:58 CEST 2004
 */
public final class ModuleInitError extends SdlError {

  /**
   * the nullary creator
   * @return a ModuleInitError object
   */
  public ModuleInitError() {
    super();
  }

  /**
   * the unary creator
   * @param errorMessage the error message
   * @return a ModuleInitError object
   */
  public ModuleInitError(String errorMessage) {
    super(errorMessage);
  }

}
