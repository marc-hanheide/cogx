package eu.cogx.beliefproxies.proxies;

public abstract class Proxy<T extends Ice.Object> {
	protected final Class<? extends T> _proxyFor;
	protected final T _content;

	/**
	 * @param class1
	 * @param content
	 */
	public Proxy(Class<? extends T> class1, Ice.Object content) {
		super();
		assert (class1 != null);
		assert (content != null);

		_proxyFor = class1;
		_content = class1.cast(content);
	}

	public Class<? extends T> proxyFor() {
		return _proxyFor;
	}

	public T get() {
		return _content;
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
