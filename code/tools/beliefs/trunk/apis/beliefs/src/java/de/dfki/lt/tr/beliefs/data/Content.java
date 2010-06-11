package de.dfki.lt.tr.beliefs.data;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

public class Content<T extends ProbDistribution> extends
		Proxy<T> {

	public Content(Class<? extends T> class1, Object content) {
		super(class1, content);
	}

}
