package de.dfki.lt.sdl;

/**
 * instances of this class might be thrown during the clearing phase
 * of a module, caused by the calling clear()
 *
 * @see IModule
 * @see Modules
 * @see SdlError
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.0.4, Mon May 10 16:19:30 CEST 2004
 */
public final class ModuleClearError extends SdlError {

  /**
   * the nullary creator
   * @return a ModuleClearError object
   */
  public ModuleClearError() {
    super();
  }

  /**
   * the unary creator
   * @param errorMessage the error message
   * @return a ModuleClearError object
   */
  public ModuleClearError(String errorMessage) {
    super(errorMessage);
  }

}
