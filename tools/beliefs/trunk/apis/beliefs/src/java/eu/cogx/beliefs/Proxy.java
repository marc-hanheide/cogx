package eu.cogx.beliefs;

public interface Proxy<T extends Ice.Object> {
	public T get();
}
