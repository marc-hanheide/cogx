/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.genericproxies.GenericFrame;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class SpaptioTemporalFrame extends GenericFrame<SpatioTemporalFrame> {

	protected SpaptioTemporalFrame(AbstractFrame content) {
		super(SpatioTemporalFrame.class, content);
	}

}
