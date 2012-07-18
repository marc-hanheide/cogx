package de.dfki.lt.sdl;

/**
 * instances of this class might be thrown during the running/computation
 * phase of a module, caused by the calling run()
 *
 * @see IModule
 * @see Modules
 * @see SdlError
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.0.3, Mon May 10 16:24:47 CEST 2004
 */
public final class ModuleRunError extends SdlError {

  /**
   * the nullary creator
   * @return a ModuleRunError object
   */
  public ModuleRunError() {
    super();
  }

  /**
   * the unary creator
   * @param errorMessage the error message
   * @return a ModuleRunError object
   */
  public ModuleRunError(String errorMessage) {
    super(errorMessage);
  }

}
