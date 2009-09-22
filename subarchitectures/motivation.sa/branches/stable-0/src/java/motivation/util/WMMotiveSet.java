/**
 * 
 */
package motivation.util;

import motivation.slice.Motive;
import cast.architecture.ManagedComponent;

/**
 * @author marc
 * 
 */
public class WMMotiveSet extends WMEntrySet {
	protected WMMotiveSet(ManagedComponent c) {
		super(c);
	}
	
	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMMotiveSet create(ManagedComponent c) {
		WMMotiveSet s=new WMMotiveSet(c);
		s.addType(Motive.class);
		return s;
	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMMotiveSet create(ManagedComponent c,
			final Class<? extends Motive> specificType) {
		WMMotiveSet s=new WMMotiveSet(c);
		s.addType(specificType);
		return s;
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

}
