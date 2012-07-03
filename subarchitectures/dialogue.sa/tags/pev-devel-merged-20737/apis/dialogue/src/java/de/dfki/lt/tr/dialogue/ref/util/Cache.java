package de.dfki.lt.tr.dialogue.ref.util;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class Cache<A, O> {

	private ConcurrentHashMap<A, O> cache = null;
		
	public Cache() {
		cache = new ConcurrentHashMap<A, O>();
	}

	public void add(A addr, O obj) {
		cache.put(addr, obj);
	}
	
	public void overwrite(A addr, O obj) {
		cache.put(addr, obj);
	}

	public void delete(A addr) {
		cache.remove(addr);
	}

	public int size() {
		return cache.size();
	}

	public O get(A addr) {
		return cache.get(addr);
	}

	public Map<A, O> asMap() {
		return new HashMap<A, O>(cache);
	}

}
