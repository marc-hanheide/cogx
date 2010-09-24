package binder.ml;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class BiMap<K, V> implements Map<K, V> {
	
	private Map<K, V> map_kv;
	private Map<V, K> map_vk;
	
	public BiMap() {
		map_kv = new HashMap<K, V>();
		map_vk = new HashMap<V, K>();
	}
	
	public void clear() {
		map_kv.clear();
		map_vk.clear();
	}
	
	public boolean containsKey(Object key) {
		return map_kv.containsKey(key);
	}

	public boolean containsValue(Object value) {
		return map_vk.containsKey(value);
	}

	public Set<java.util.Map.Entry<K, V>> entrySet() {
		return map_kv.entrySet();
	}

	public V get(Object key) {
		return map_kv.get(key);
	}
	
	public K getKeyFromValue(V value) {
		return map_vk.get(value);
	}

	public boolean isEmpty() {
		return map_kv.isEmpty();
	}

	public Set<K> keySet() {
		return map_kv.keySet();
	}
	
	public Set<V> valueSet() {
		return map_vk.keySet();
	}

	public V put(K key, V value) {
		map_vk.put(value, key);
		return map_kv.put(key, value);
	}

	public void putAll(Map<? extends K, ? extends V> t) {
		V value = null;
		for(K key : t.keySet()) {
			value = t.get(key);
			map_kv.put(key, value);
			map_vk.put(value, key);
		}
	}

	public V remove(Object key) {
		V value = map_kv.remove(key);
		map_vk.remove(value);
		return value;
	}

	public int size() {
		return map_kv.size();
	}

	public Collection<V> values() {
		return map_kv.values();
	}
}
