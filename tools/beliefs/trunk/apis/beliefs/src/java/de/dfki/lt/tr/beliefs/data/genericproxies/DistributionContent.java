/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.genericproxies;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author marc
 * 
 */
public class DistributionContent<T extends DistributionValues>
		extends Proxy<T> {
	public DistributionContent(Class<? extends T> type, DistributionValues content) {
		super(type, content);
	}

}
