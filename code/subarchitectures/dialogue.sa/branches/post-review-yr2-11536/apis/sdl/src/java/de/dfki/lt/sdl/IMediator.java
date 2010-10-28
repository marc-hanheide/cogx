package de.dfki.lt.sdl;

/**
 * this is the interface which every mediator must fulfill!!
 *
 * @see Mediators
 * @see IModule
 * @see Modules
 *
 * @author (C) Hans-Ulrich Krieger
 * @since JDK 1.3
 * @version 1.1.2, Mon May 10 15:08:07 CEST 2004
 */
public interface IMediator {

  /**
   * given two modules, sequence mediates the input to module2,
   * computed from the output of module1;
   * @see Mediators#seq for a possible implementation
   */
  public Object seq(IModule module1, IModule module2);

  /**
   * given an array of modules, parallelism constructs the final
   * result of a (quasi-)parallel independent computation of an
   * arbitrary number of modules, from the output of these modules;
   * @see Mediators#par for a possible implementation
   */
  public Object par(IModule[] modules);

  /**
   * given a single module, fixpoint construct a final result
   * by feeding the module with its own output; this computation,
   * of course, must not terminate;
   * @see Mediators#fix for a possible implementation
   */
  public Object fix(IModule module);

}
