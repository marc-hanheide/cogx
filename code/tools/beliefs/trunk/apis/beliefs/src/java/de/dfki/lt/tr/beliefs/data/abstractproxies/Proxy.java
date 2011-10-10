package de.dfki.lt.tr.beliefs.data.abstractproxies;

import org.apache.log4j.Logger;

import Ice.Object;

/**
 * provide a type safe encapsulation of a slice data type
 * 
 * @author marc
 * 
 * @param <T>
 *            the type of slice object this proxy encapsulates
 */
public abstract class Proxy<T extends Ice.Object> {
	protected final Class<? extends T> _proxyFor;
	protected final T _content;
	protected final Logger _logger;
	
	public static <Tin extends Ice.Object, P2 extends Proxy<? extends Tin>> P2 create(ProxyFactory<Tin, P2> factory, Tin object) {
		return factory.create(object);
	}
	
	public <P2 extends Proxy<? extends T>> P2 getAs(ProxyFactory<T, P2> factory) {
		return factory.create(this);
	}
	
	public static <Tin extends Ice.Object, P2 extends Proxy<? extends Tin>> P2 create(ProxyFactory<Tin, P2> factory, Proxy<? extends Tin> object) {
		return factory.create(object);
	}
	
	
	
	
	/** create a Proxy
	 * @param class1
	 *            the type this proxy shall encapsulate
	 * @param content
	 *            the Ice.Object that is encapsulated. The object is casted
	 *            internally on creation to ensure all operations are valid.
	 * @throws ClassCastException
	 *             if the content is not of designated type (your code is broken
	 *             by design in this case)
	 */
	protected Proxy(Class<? extends T> class1, Ice.Object content) {
		super();
		_logger = Logger.getLogger(this.getClass());
		assert (class1 != null);
		assert (content != null);

		_proxyFor = class1;
		try {
			_content = class1.cast(content);
		} catch (ClassCastException e) {
//			_logger.warn("Proxy: tried to cast "
//					+ content.getClass().getName() + " into "
//					+ _proxyFor.getName() + " which is not valid!", e);
			_logger.warn("Proxy: tried to cast "
					+ content.getClass().getName() + " into "
					+ _proxyFor.getName() + " which is not valid!");

			throw (e);
		}
	}

	
	
	/**
	 * @return the content of this proxy
	 */
	public T get() {
		return _content;
	}

	/**
	 * @return the class this proxy is for
	 */
	public Class<? extends Object> proxyFor() {
		return _content.getClass();
	}

	public boolean isFor(Class<? extends T> type) {
		return (type.isAssignableFrom(proxyFor()));
	}
	
	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return this.getClass().getName() + " [_content=" + _content + " ]";
	}
}
