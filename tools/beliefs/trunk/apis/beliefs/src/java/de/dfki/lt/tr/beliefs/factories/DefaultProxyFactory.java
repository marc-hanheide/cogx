package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;

public interface DefaultProxyFactory<T extends Proxy<? extends Ice.Object>> extends ProxyFactory<T> {
	public T create();
}
