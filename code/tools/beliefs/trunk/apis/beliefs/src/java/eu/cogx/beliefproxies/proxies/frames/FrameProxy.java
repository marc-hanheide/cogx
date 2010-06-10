/**
 * 
 */
package eu.cogx.beliefproxies.proxies.frames;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import eu.cogx.beliefproxies.proxies.Proxy;

/**
 * @author marc
 *
 */
public class FrameProxy<T extends AbstractFrame> extends Proxy<T> {

	public FrameProxy(Class<? extends T> class1, Object content) {
		super(class1, content);

	}

}
