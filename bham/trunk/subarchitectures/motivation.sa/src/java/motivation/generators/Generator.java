/**
 * 
 */
package motivation.generators;

import cast.architecture.ManagedComponent;

/**
 * @author marc
 *
 */
abstract class Generator extends ManagedComponent {

	/**
	 * 
	 */
	public Generator() {
		// TODO Auto-generated constructor stub
	}

	/** actually start the component
	 * 
	 */
	@Override
	protected void runComponent() {
	    println("Look out world, here I come...");
	}


}
